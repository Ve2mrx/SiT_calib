#!/usr/bin/env python3
"""
Phase 1 parser for SiT5721 calibration log -> per-day records.

Pure parsing, no Excel. Reads SiT-calib_output.txt and returns one record
per 24 h block. Each block is a verbose section terminated by a single
summary line of the form:

    2412,  69400, 2012050756.441, +0.1232058366, flags(freq: Valid, phase: Valid), SiT status(good, stabilized)

Fields on that line: WNO, iTOW, phaseOffset(ns), totalOffsetWritten(ppm), flags, status.
Pull Value / Aging compensation / UTC date are pulled from the verbose lines
above the summary so we can flag re-tune days and stamp a real calendar date.

Note for Cowork (spreadsheet/VBA side): `regenerate()` below is called
both from this file's own CLI (`main()`) *and* automatically from
get-data.py's capture loop after every daily record, wrapped in a
try/except there so a parsing bug can't take down live GNSS capture.
If you add a new field for the spreadsheet: extend `parse()`'s `rec`
dict, `CSV_FIELDS`, and the CSV row-building loop in `regenerate()`
together, keep `regenerate()` itself side-effect-free beyond writing
parsed_records.json/.csv (no new file paths without updating
get-data.py's caller too), and keep it safe to call once a day forever
without leaking memory/handles. `regenerate()` also refuses to write a
parsed_records.json with fewer records than the one already on disk
(raises RuntimeError instead, caught upstream) - SiT-calib_output.txt is
append-only, so a drop means something's wrong, not real data; pass
force=True (or CLI --force) to override deliberately.

Power-loss handling (new 2026-07-04): when the SiT5721 loses power,
restart-SiT5721.py (SiT5721 repo) recalculates and reloads an
aging-corrected Pull Value, then mbt-ubx-apps/restart-calib.sh archives
the *previous* SiT-calib_output.txt/parsed_records.json/.csv to
~/SiT-calib_archive/ (as SiT-calib_output_<timestamp>.txt/
parsed_records_<timestamp>.json/.csv) and starts a fresh
SiT-calib_output.txt (with a leading `#`-comment noting the event -
harmless to this parser, it just accumulates into an unmatched `buf` and
is discarded) before capture resumes. There is deliberately no in-band
field/flag for this in parse()/CSV_FIELDS: the archive boundary *is* the
mark. For Cowork: a new calibration epoch is signaled by
parsed_records.json/.csv simply containing fewer/newer records than
before with a corresponding new archive file appearing alongside them -
treat that as "start a new epoch" in the spreadsheet, and consult the
archived files (same schema) for the prior epoch's history if needed.

Unit-label fix (2026-07-04): get-data.py's verbose "TIM-SMEAS ... freq
offset"/"freq uncertainty" lines in SiT-calib_output.txt were mislabeled
"ns" - those fields are actually ps/s (per u-blox's TIM-SMEAS scaling,
2**-8 ps/s per LSB), not a time offset like phaseOffset/phaseUnc genuinely
are. Fixed to read "ps/s" going forward. For Cowork: this is a display-only
fix in the raw log text - parse()/CSV_FIELDS never captured freqOffset/
freqUnc (only phase_ns), so parsed_records.json/.csv and anything built
from them are unaffected; only raw SiT-calib_output.txt/archived copies
from before this date show the old "ns" mislabel on those two lines.

CSV record + reference-quality fields (2026-07-18): get-data.py now embeds
a machine-parseable `CSV,<version>,...` line in every block of
SiT-calib_output.txt (see its build_csv_line()), in addition to - not
replacing - the human summary line described above. `parse()` tries
`parse_csv_line()` first: when a block has a CSV line, it is the *sole*
source for every field of that block (no mixing with SUMMARY_RE/legacy
regex groups). When absent (any block predating this change), parsing
falls back unchanged to SUMMARY_RE/UTC_RE/PULL_RE/AGING_RE.

Four new GNSS-reference-quality fields ride along, appended to
`CSV_FIELDS` after `total_log` per the field-position contract above:
`phase_unc_ns`/`freq_unc_ps_s` (TIM-SMEAS phase/freq uncertainty),
`time_acc_ns` (TIM-TOS GNSS time uncertainty - the reference side, unlike
the SiT status which is oscillator-side only), and `sv_count` (NAV-PVT SV
count, best-effort - may be empty even in a CSV-line block if NAV-PVT
hadn't arrived recently enough when the snapshot was taken). For blocks
with no CSV line: `phase_unc_ns`/`freq_unc_ps_s` still backfill via
`SMEAS_UNC_RE` (that text already existed, unparsed, in every historic
entry); `time_acc_ns`/`sv_count` can't backfill and come back empty -
TIM-TOS never printed a GNSS-uncertainty field and NAV-PVT never existed
as a source before this change.

The one-line summary format described at the top of this docstring is now
**DEPRECATED and frozen** - no new fields are ever added to it again.
Once there's enough production history on the CSV-line format to trust it
(a handful of daily cycles), the plan is to: drop the summary-line print
from get-data.py, switch this file's block-boundary detection from
SUMMARY_RE to the CSV,<version>,... line itself, and retire
SUMMARY_RE/UTC_RE/PULL_RE/AGING_RE from live parsing entirely (kept only
if re-parsing pre-CSV-line archives is ever needed). Not done yet because
pulling the summary line now would break parsing of every block that
predates the CSV line.

Versioning: get-data.py's `CSV_LINE_VERSION` must match a
`CSV_LINE_FIELDS_V<N>` branch in `parse_csv_line()` below - bump the
version and add a new branch together whenever the CSV line's field list
changes. An unrecognized version falls through to the legacy (non-CSV-line)
parsing path rather than crashing.
"""

import io
import os
import re
import sys
import csv
import json
import struct
import tempfile
from datetime import datetime


def _read_version() -> str:
    """Reads the sibling VERSION file (repo root); "unknown" if missing."""
    version_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "VERSION")
    try:
        with open(version_path) as f:
            return f.read().strip()
    except OSError:
        return "unknown"


__version__ = _read_version()


def atomic_write_text(path, text, newline=None):
    """Write text to path via a same-dir temp file + os.replace(), so an
    interrupted write can't leave a truncated/corrupt output file."""
    directory = os.path.dirname(os.path.abspath(path)) or "."
    fd, tmp_path = tempfile.mkstemp(dir=directory, prefix=os.path.basename(path) + ".")
    try:
        os.chmod(tmp_path, 0o644)
        with os.fdopen(fd, "w", newline=newline) as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, path)
    except BaseException:
        os.unlink(tmp_path)
        raise

# pull_ppm / aging_pps / total_ppm hold the SiT5721 register values recovered to
# FULL float32 register precision (the log rounds them). pull_log8g / aging_log8g /
# total_log keep the raw log values for reference. dow_fr (index 14) and
# total_log (index 15) are the macro's fixed field positions - never move or
# insert before them. New fields always append after total_log, as done here
# for the four GNSS-reference-quality fields (see module docstring).
CSV_FIELDS = ["idx", "date", "time", "wno", "itow", "phase_ns",
              "total_ppm", "pull_ppm", "aging_pps", "retune", "flags", "status",
              "pull_log8g", "aging_log8g", "dow_fr", "total_log",
              "phase_unc_ns", "freq_unc_ps_s", "time_acc_ns", "sv_count"]

# French weekday names, indexed by datetime.weekday() (Mon=0 .. Sun=6).
# Row 14 ("Day of week") in Calc-new uses these.
DOW_FR = ["lundi", "mardi", "mercredi", "jeudi", "vendredi", "samedi", "dimanche"]


def weekday_fr(date_str):
    """French weekday name for an ISO date string, or '' if unparseable."""
    if not date_str:
        return ""
    try:
        return DOW_FR[datetime.strptime(date_str, "%Y-%m-%d").weekday()]
    except ValueError:
        return ""


def f32(x):
    """Round-trip through a 32-bit float == exactly what the SiT register holds."""
    if x is None:
        return None
    return struct.unpack("f", struct.pack("f", x))[0]


def pull_full_ppm(ppm_8g):
    """Recover full float32 register precision for a ppm-of-fractional-offset value:
    Pull Value (0x61) and Total offset written (0xAB). The register stores the
    *fractional* offset as float32; the log prints fractional/1e-6, rounded."""
    if ppm_8g is None:
        return None
    return f32(ppm_8g * 1e-6) / 1e-6


def aging_full(pps_8g):
    """Aging compensation is stored directly as float32 part/s."""
    return f32(pps_8g)

# --- regexes -----------------------------------------------------------------
# DEPRECATED, legacy fallback only (see module docstring): SUMMARY_RE still
# fires unconditionally to detect a block boundary, but its captured groups
# are only used - along with UTC_RE/PULL_RE/AGING_RE - when a block has no
# CSV,<version>,... line (i.e. it predates this change).
SUMMARY_RE = re.compile(
    r"^\s*(?P<wno>\d+)\s*,"
    r"\s*(?P<itow>\d+)\s*,"
    r"\s*(?P<phase>[-+]?\d+(?:\.\d+)?)\s*,"
    r"\s*(?P<total>[-+]?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)\s*,"
    r"\s*flags\((?P<flags>[^)]*)\)\s*,"
    r"\s*SiT status\((?P<status>[^)]*)\)\s*$"
)
UTC_RE   = re.compile(r"TIM-TOS\s+UTC\s+(\d{4}-\d{2}-\d{2}),\s*([0-9:]+)\+?\S*")
PULL_RE  = re.compile(r"SiT Pull Value\s+([-+]?[\d.eE+-]+)\s*ppm")
AGING_RE = re.compile(r"SiT Aging compensation\s+([-+]?[\d.eE+-]+)\s*part/s")
TOS_RE   = re.compile(r"TIM-TOS\s+week, TOW, system\s+(\d+)\s*,\s*(\d+)")

# Legacy-only backfill: matches get-data.py's pre-CSV-line "TIM-SMEAS phase
# uncertainty: ... ns, freq uncertainty: ... ps/s" line, which already
# exists (unparsed until now) in every historic block. Not consulted once a
# block has a CSV line.
SMEAS_UNC_RE = re.compile(
    r"TIM-SMEAS\s+phase uncertainty:\s*(?P<phase_unc>[-+]?\d+(?:\.\d+)?)\s*ns\s*,"
    r"\s*freq uncertainty:\s*(?P<freq_unc>[-+]?\d+(?:\.\d+)?)\s*ps/s"
)

# The versioned machine-readable record - see get-data.py's build_csv_line().
CSV_LINE_RE = re.compile(r"^CSV,(?P<version>\d+),(?P<rest>.*)$", re.MULTILINE)

CSV_LINE_FIELDS_V1 = [
    "date", "time", "wno", "itow", "phase_ns", "total_log",
    "pull_log8g", "aging_log8g", "flags_freq", "flags_phase",
    "status_error", "status_stability",
    "phase_unc_ns", "freq_unc_ps_s", "time_acc_ns", "sv_count",
]


def parse_csv_line(block):
    """
    Parses the versioned CSV,<version>,... record embedded in a block, if
    present. Dispatches on version (only v1 exists today); an unrecognized
    future version falls through to legacy parsing rather than crashing.

    :param str block: joined verbose lines accumulated for one block

    :return dict | None: {field_name: raw_str_value} from CSV_LINE_FIELDS_V1,
        or None if the block has no CSV line or an unrecognized version
    """
    m = CSV_LINE_RE.search(block)
    if not m:
        return None
    version = int(m.group("version"))
    # elif version == 2: ... CSV_LINE_FIELDS_V2 ... is where a future field
    # bump gets a branch (see module docstring's versioning note).
    if version != 1:
        return None
    # csv.reader rather than a plain .split(",") - none of build_csv_line()'s
    # fields currently embed a comma (numbers/enum strings only), but this
    # stays correct if that ever changes, matching this file's CSV writer
    # use elsewhere (regenerate()) instead of hand-rolled comma-joining.
    row = next(csv.reader([m.group("rest")]))
    if len(row) != len(CSV_LINE_FIELDS_V1):
        return None
    return dict(zip(CSV_LINE_FIELDS_V1, row))


def parse(path):
    records = []
    buf = []  # verbose lines accumulated for the current block
    with open(path, "r", encoding="utf-8", errors="replace") as fh:
        for raw in fh:
            line = raw.rstrip("\n")
            m = SUMMARY_RE.match(line)
            if not m:
                buf.append(line)
                continue

            # We hit a summary line -> close out this block.
            block = "\n".join(buf)
            csv_rec = parse_csv_line(block)

            if csv_rec:
                # CSV,<version>,... line present - sole source for every
                # field of this block, no mixing with SUMMARY_RE/legacy
                # regex groups even though they'd usually agree.
                date = csv_rec["date"] or None
                time = csv_rec["time"] or None
                wno = int(csv_rec["wno"])
                itow = float(csv_rec["itow"])
                phase_ns = float(csv_rec["phase_ns"])
                total_log = float(csv_rec["total_log"])
                pull_8g = float(csv_rec["pull_log8g"]) if csv_rec["pull_log8g"] else None
                aging_8g = float(csv_rec["aging_log8g"]) if csv_rec["aging_log8g"] else None
                flags = f"freq: {csv_rec['flags_freq']}, phase: {csv_rec['flags_phase']}"
                status = f"{csv_rec['status_error']}, {csv_rec['status_stability']}"
                phase_unc_ns = float(csv_rec["phase_unc_ns"]) if csv_rec["phase_unc_ns"] else None
                freq_unc_ps_s = float(csv_rec["freq_unc_ps_s"]) if csv_rec["freq_unc_ps_s"] else None
                time_acc_ns = float(csv_rec["time_acc_ns"]) if csv_rec["time_acc_ns"] else None
                sv_count = int(csv_rec["sv_count"]) if csv_rec["sv_count"] else None
            else:
                # No CSV line - block predates this change. Legacy
                # extraction, unchanged, plus SMEAS_UNC_RE backfill for the
                # two fields whose text already existed pre-change.
                utc = UTC_RE.search(block)
                pull = PULL_RE.search(block)
                aging = AGING_RE.search(block)
                smeas_unc = SMEAS_UNC_RE.search(block)

                date = utc.group(1) if utc else None
                time = utc.group(2) if utc else None
                wno = int(m.group("wno"))
                itow = float(m.group("itow"))
                phase_ns = float(m.group("phase"))
                total_log = float(m.group("total"))
                flags = m.group("flags").strip()
                status = m.group("status").strip()
                pull_8g = float(pull.group(1)) if pull else None
                aging_8g = float(aging.group(1)) if aging else None
                phase_unc_ns = float(smeas_unc.group("phase_unc")) if smeas_unc else None
                freq_unc_ps_s = float(smeas_unc.group("freq_unc")) if smeas_unc else None
                time_acc_ns = None    # TIM-TOS never printed this before the CSV line
                sv_count = None       # NAV-PVT didn't exist as a source before the CSV line

            rec = {
                "date":          date,                                 # calendar date (UTC)
                "time":          time,
                "wno":           wno,                                  # -> row 9 (WNO end, measured)
                "itow":          itow,                                 # -> row 10 (iTOW end, measured)
                "phase_ns":      phase_ns,                             # -> row 11 (u-blox TIM-SMEAS ns)
                "total_ppm":     pull_full_ppm(total_log),             # -> row 12, full f32 register value
                "pull_ppm":      pull_full_ppm(pull_8g),               # -> row 1 (re-tune only) full f32
                "aging_pps":     aging_full(aging_8g),                 # -> row 2 (re-tune only) full f32
                "pull_log8g":    pull_8g,                              # raw 8-sig-fig log value (reference)
                "aging_log8g":   aging_8g,
                "dow_fr":        weekday_fr(date),                     # -> row 14 (Day of week, French)
                "flags":         flags,
                "status":        status,
                "total_log":     total_log,                           # raw log total (reference)
                "phase_unc_ns":  phase_unc_ns,                        # TIM-SMEAS phase uncertainty (ns)
                "freq_unc_ps_s": freq_unc_ps_s,                       # TIM-SMEAS freq uncertainty (ps/s)
                "time_acc_ns":   time_acc_ns,                         # TIM-TOS GNSS time uncertainty (ns)
                "sv_count":      sv_count,                            # NAV-PVT SV count (best-effort)
            }
            records.append(rec)
            buf = []  # reset for next block
    return records


def annotate_retune(records):
    """Mark a day as a re-tune day when Pull Value or Aging comp changed."""
    prev_pull = prev_aging = None
    for r in records:
        retune = False
        if r["pull_ppm"] is not None and prev_pull is not None and r["pull_ppm"] != prev_pull:
            retune = True
        if r["aging_pps"] is not None and prev_aging is not None and r["aging_pps"] != prev_aging:
            retune = True
        r["retune"] = retune
        if r["pull_ppm"] is not None:
            prev_pull = r["pull_ppm"]
        if r["aging_pps"] is not None:
            prev_aging = r["aging_pps"]
    return records


def regenerate(path, verbose=True, force=False):
    """
    Parse `path` and (re)write parsed_records.json/.csv next to it.

    Output files are written alongside `path` itself (not the caller's
    cwd), so this is safe to call regardless of where the process
    invoking it happens to be running from.

    Refuses to overwrite an existing parsed_records.json with fewer
    records than it already has, unless `force` is set - SiT-calib_output.txt
    is append-only, so a record-count drop means a truncated/corrupted
    read or a parsing regression, not legitimate new data. Nothing is
    written if this check fails.

    :param str path: SiT-calib_output.txt (or equivalent) to parse
    :param bool verbose: print the full record table + sanity checks
        (standalone CLI use); if False, only a one-line summary
    :param bool force: skip the record-count regression check
    :return list: the parsed/annotated records
    :raises RuntimeError: if the regression check fails and force=False
    """
    recs = annotate_retune(parse(path))
    out_dir = os.path.dirname(os.path.abspath(path)) or "."
    json_path = os.path.join(out_dir, "parsed_records.json")

    if not force:
        try:
            with open(json_path) as f:
                existing_count = len(json.load(f))
        except (FileNotFoundError, json.JSONDecodeError):
            existing_count = 0

        if len(recs) < existing_count:
            raise RuntimeError(
                f"refusing to overwrite {json_path} ({existing_count} records) "
                f"with only {len(recs)} newly parsed from {path} - looks like "
                f"a truncated/corrupted read rather than real data (pass "
                f"force=True to override)"
            )

    if verbose:
        print(f"Parsed {len(recs)} daily records from {path} (parse_sit.py v{__version__})\n")
        hdr = f"{'#':>3} {'date':10} {'WNO':>5} {'iTOW':>8} {'phase_ns':>16} {'total_ppm':>14} {'pull_ppm':>14} {'aging_pps':>14} {'retune':>6}"
        print(hdr)
        print("-" * len(hdr))
        for i, r in enumerate(recs, 1):
            print(f"{i:>3} {str(r['date']):10} {r['wno']:>5} {r['itow']:>8.0f} "
                  f"{r['phase_ns']:>16.3f} {r['total_ppm']:>14.10g} "
                  f"{(r['pull_ppm'] if r['pull_ppm'] is not None else float('nan')):>14.11g} "
                  f"{(r['aging_pps'] if r['aging_pps'] is not None else float('nan')):>14.6g} "
                  f"{'YES' if r['retune'] else '':>6}")

        # sanity checks
        missing_date = [i for i, r in enumerate(recs, 1) if r["date"] is None]
        dates = [r["date"] for r in recs if r["date"]]
        dupes = sorted({d for d in dates if dates.count(d) > 1})
        retunes = [i for i, r in enumerate(recs, 1) if r["retune"]]
        print("\n--- sanity ---")
        print(f"records              : {len(recs)}")
        print(f"missing date         : {missing_date or 'none'}")
        print(f"duplicate dates      : {dupes or 'none'}")
        print(f"re-tune day indices  : {retunes or 'none'}")
        if dates:
            print(f"date range           : {dates[0]}  ->  {dates[-1]}")

    json_buf = io.StringIO()
    json.dump(recs, json_buf, indent=2)
    atomic_write_text(json_path, json_buf.getvalue())

    # CSV for the Excel VBA macro. repr() keeps full round-trip precision so
    # Excel receives the exact measured + full-register-precision values.
    csv_buf = io.StringIO(newline="")
    w = csv.writer(csv_buf)
    w.writerow(CSV_FIELDS)
    for i, r in enumerate(recs, 1):
        w.writerow([
            i, r["date"], r["time"], r["wno"], int(r["itow"]),
            repr(r["phase_ns"]), repr(r["total_ppm"]),
            ("" if r["pull_ppm"] is None else repr(r["pull_ppm"])),
            ("" if r["aging_pps"] is None else repr(r["aging_pps"])),
            (1 if r["retune"] else 0), r["flags"], r["status"],
            ("" if r["pull_log8g"] is None else repr(r["pull_log8g"])),
            ("" if r["aging_log8g"] is None else repr(r["aging_log8g"])),
            r["dow_fr"],
            repr(r["total_log"]),
            ("" if r["phase_unc_ns"] is None else repr(r["phase_unc_ns"])),
            ("" if r["freq_unc_ps_s"] is None else repr(r["freq_unc_ps_s"])),
            ("" if r["time_acc_ns"] is None else repr(r["time_acc_ns"])),
            ("" if r["sv_count"] is None else r["sv_count"]),
        ])
    atomic_write_text(os.path.join(out_dir, "parsed_records.csv"), csv_buf.getvalue(), newline="")

    if verbose:
        print("\nWrote parsed_records.json and parsed_records.csv")
    else:
        print(f"parse_sit: regenerated parsed_records.json/csv ({len(recs)} records) from {path}")

    return recs


def main():
    force = "--force" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--force"]
    path = args[0] if args else "SiT-calib_output.txt"
    regenerate(path, verbose=True, force=force)


if __name__ == "__main__":
    main()
