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
"ns" - those fields are not a time offset like phaseOffset/phaseUnc
genuinely are. Relabeled to "ps/s" going forward. For Cowork: this is a
display-only fix in the raw log text - parse()/CSV_FIELDS never captured
freqOffset/freqUnc (only phase_ns), so parsed_records.json/.csv and
anything built from them are unaffected; only raw SiT-calib_output.txt/
archived copies from before this date show the old "ns" mislabel.

!! SUPERSEDED 2026-08-01: "ps/s" was ALSO wrong. The scaling is 2**-8 **ppb**
   per LSB, not 2**-8 ps/s, so that fix replaced one wrong unit with another
   that is off by 1000x in the other direction. Corrected to "ppb" in
   get-data.py on 2026-08-01 (commit cf4be2f). Log text written between
   2026-07-04 and 2026-08-01 says "ps/s" and is wrong; text before it says
   "ns" and is also wrong. The numbers themselves were always right.

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
`phase_unc_ns`/`freq_unc_ppb` (TIM-SMEAS phase/freq uncertainty),
`time_acc_ns` (TIM-TOS GNSS time uncertainty - the reference side, unlike
the SiT status which is oscillator-side only), and `sv_count` (NAV-PVT SV
count, best-effort - may be empty even in a CSV-line block if NAV-PVT
hadn't arrived recently enough when the snapshot was taken). For blocks
with no CSV line: `phase_unc_ns`/`freq_unc_ppb` still backfill via
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

SiT health telemetry + CM4 SoC temp (2026-08-01, CSV_LINE_VERSION 2): the
SiT "good, stabilized" status is oscillator-side only and doesn't reveal
e.g. a drifting thermal control loop, so `get-data.py` now also reads and
logs the SiT5721's own health registers (resonator temp, supply voltage,
heater power + target, temp error - all already read by
`read_SiT_operation()`, no new I2C work), CM4 SoC temperature (a proxy for
enclosure-interior temperature, relevant to the LEA-M8F's own, uncompensated
temperature sensitivity), and TIM-SMEAS `freqOffset` (an independent
cross-check of the same quantity the phase chain computes - previously
captured into the text log and then discarded at CSV time). `CSV_FIELDS`
gained seven columns at the end to match: `freq_offset_ppb`,
`resonator_temp_c`, `supply_v`, `heater_power_w`, `heater_power_target_w`,
`temp_error_c`, `cm4_soc_temp_c`. For Cowork: these come back empty for any
block parsed as V1 (or with no CSV line at all) - there is no back-fill,
since none of these registers were ever read into the log before this
change (`freqOffset` is the one partial exception - present in the raw
text since 2026-07-19, but not yet wired into a back-fill regex here; see
`claude-code-health-logging-patch.md` sec. 5b-bis if that's wanted later).

Also note: TIM-SMEAS `freqOffset`/`freqUnc` are u-blox `2^-8 ppb`, scaled
by pyubx2 - i.e. **ppb**, not `ps/s`/`ns` as get-data.py's human-readable
block previously labeled them (fixed the same day as this CSV bump).

COLUMN RENAMED (2026-08-03): `freq_unc_ps_s` -> `freq_unc_ppb`.
The *values* were never wrong - only the name was, by a factor of 1000. The
rename was deferred at the time on the belief that it "breaks every consumer
that reads by name"; an audit showed that to be false. The main consumer,
Excel/ImportSiTCalib-*.bas, reads parsed_records.csv **by column index** and
skips the header entirely, and get-data.py's build_csv_line() emits a
positional list. A rename moves nothing, so nothing there notices.

For Cowork / anyone reading archived data:
  * Column ORDER is unchanged - `freq_unc_ppb` still sits at index 17 of
    CSV_FIELDS, exactly where `freq_unc_ps_s` was. The field-position
    contract above is intact.
  * Archived parsed_records.csv/.json (NAS, Excel/_Archive/) keep the OLD
    name forever. Name-based readers should accept both for a release or
    two:  val = row.get("freq_unc_ppb") or row.get("freq_unc_ps_s")
  * The live file needs no migration: regenerate() rewrites the whole CSV
    from SiT-calib_output.txt on every run, so the header flips on the
    first run after this change.
  * SMEAS_UNC_RE still matches the literal text "ps/s" on purpose - see the
    comment on it below before touching it.
Read every historic `freq_unc_ps_s` value as ppb. Typical value ~4.2 ppb.
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
              # freq_unc_ppb was named freq_unc_ps_s until 2026-08-03. Same
              # index (17), same values - the old name was wrong by 1000x.
              # Archived CSVs keep it; readers should accept either.
              "phase_unc_ns", "freq_unc_ppb", "time_acc_ns", "sv_count",
              # v2 additions (2026-08-01) - APPEND ONLY, ImportSiTCalib
              # indexes this file. Empty for blocks parsed from a CSV,1,...
              # line or no CSV line at all (see CSV_LINE_FIELDS_V2 below).
              "freq_offset_ppb", "resonator_temp_c", "supply_v",
              "heater_power_w", "heater_power_target_w", "temp_error_c",
              "cm4_soc_temp_c"]

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
# block has a CSV line - and since every block since 2026-07-18 carries one,
# this path is in practice dead. Kept only so a pre-2026-07-18 log can still
# be re-parsed.
#
# !! THE "ps/s" IN THIS PATTERN IS A WRONG UNIT, AND IT IS WRONG ON PURPOSE.
#    DO NOT "FIX" IT TO ppb.
#
#    u-blox TIM-SMEAS freqOffset/freqUnc are 2^-8 **ppb** and pyubx2 already
#    applies the scale, so the number these lines carry has always been ppb.
#    get-data.py printed the label as "ps/s" until 2026-08-01, when the
#    display was corrected (commit cf4be2f). This regex does not describe what
#    the value *is* - it matches the literal text that older logs on disk
#    actually contain. Those bytes say "ps/s" and will say "ps/s" forever.
#
#    Change this to "ppb" and the backfill silently stops matching every
#    historic block, which is the only thing it exists for. The captured value
#    still lands in the field CSV_FIELDS calls freq_unc_ppb, which was
#    renamed from freq_unc_ps_s on 2026-08-03 precisely because the old name
#    was misread three times. The regex text and the field name disagree on
#    purpose: the text describes bytes on disk, the name describes the unit.
#
#    There is no version ambiguity to worry about: the label changed on
#    2026-08-01, long after CSV lines existed, so no block can both lack a
#    CSV line and print "ppb".
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
    "phase_unc_ns", "freq_unc_ppb", "time_acc_ns", "sv_count",
]

# v2 = v1 + appended SiT health telemetry, CM4 SoC temp, and TIM-SMEAS
# freqOffset (2026-08-01). Append-only: see get-data.py's build_csv_line().
CSV_LINE_FIELDS_V2 = CSV_LINE_FIELDS_V1 + [
    "freq_offset_ppb", "resonator_temp_c", "supply_v",
    "heater_power_w", "heater_power_target_w", "temp_error_c", "cm4_soc_temp_c",
]


def parse_csv_line(block):
    """
    Parses the versioned CSV,<version>,... record embedded in a block, if
    present. Dispatches on version (v1 or v2 today); an unrecognized future
    version falls through to legacy parsing rather than crashing.

    :param str block: joined verbose lines accumulated for one block

    :return dict | None: {field_name: raw_str_value} from CSV_LINE_FIELDS_V1
        or CSV_LINE_FIELDS_V2 (matching the record's own version), or None
        if the block has no CSV line or an unrecognized version
    """
    m = CSV_LINE_RE.search(block)
    if not m:
        return None
    version = int(m.group("version"))
    if version == 1:
        fields = CSV_LINE_FIELDS_V1
    elif version == 2:
        fields = CSV_LINE_FIELDS_V2
    else:
        return None      # unrecognized future version -> legacy path, never crash
    # csv.reader rather than a plain .split(",") - none of build_csv_line()'s
    # fields currently embed a comma (numbers/enum strings only), but this
    # stays correct if that ever changes, matching this file's CSV writer
    # use elsewhere (regenerate()) instead of hand-rolled comma-joining.
    row = next(csv.reader([m.group("rest")]))
    if len(row) != len(fields):
        return None
    return dict(zip(fields, row))


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
                freq_unc_ppb = float(csv_rec["freq_unc_ppb"]) if csv_rec["freq_unc_ppb"] else None
                time_acc_ns = float(csv_rec["time_acc_ns"]) if csv_rec["time_acc_ns"] else None
                sv_count = int(csv_rec["sv_count"]) if csv_rec["sv_count"] else None
                # v2-only fields (see CSV_LINE_FIELDS_V2) - absent from a
                # V1 csv_rec dict, hence .get() rather than [...]; a V1
                # block simply has nothing to report for these yet.
                freq_offset_ppb = float(csv_rec["freq_offset_ppb"]) if csv_rec.get("freq_offset_ppb") else None
                resonator_temp_c = float(csv_rec["resonator_temp_c"]) if csv_rec.get("resonator_temp_c") else None
                supply_v = float(csv_rec["supply_v"]) if csv_rec.get("supply_v") else None
                heater_power_w = float(csv_rec["heater_power_w"]) if csv_rec.get("heater_power_w") else None
                heater_power_target_w = float(csv_rec["heater_power_target_w"]) if csv_rec.get("heater_power_target_w") else None
                temp_error_c = float(csv_rec["temp_error_c"]) if csv_rec.get("temp_error_c") else None
                cm4_soc_temp_c = float(csv_rec["cm4_soc_temp_c"]) if csv_rec.get("cm4_soc_temp_c") else None
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
                freq_unc_ppb = float(smeas_unc.group("freq_unc")) if smeas_unc else None
                time_acc_ns = None    # TIM-TOS never printed this before the CSV line
                sv_count = None       # NAV-PVT didn't exist as a source before the CSV line
                freq_offset_ppb = None        # v2-only, see CSV_LINE_FIELDS_V2
                resonator_temp_c = None
                supply_v = None
                heater_power_w = None
                heater_power_target_w = None
                temp_error_c = None
                cm4_soc_temp_c = None

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
                "freq_unc_ppb":  freq_unc_ppb,                       # TIM-SMEAS freq uncertainty (ppb)
                "time_acc_ns":   time_acc_ns,                         # TIM-TOS GNSS time uncertainty (ns)
                "sv_count":      sv_count,                            # NAV-PVT SV count (best-effort)
                "freq_offset_ppb":       freq_offset_ppb,             # TIM-SMEAS freq offset (ppb) - v2
                "resonator_temp_c":      resonator_temp_c,            # SiT 0xA1 - v2
                "supply_v":              supply_v,                   # SiT 0xA3 - v2
                "heater_power_w":        heater_power_w,              # SiT 0xA7 - v2
                "heater_power_target_w": heater_power_target_w,       # SiT 0xB1 - v2
                "temp_error_c":          temp_error_c,                # SiT 0xB0 - v2
                "cm4_soc_temp_c":        cm4_soc_temp_c,              # CM4 SoC temp (best-effort) - v2
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
            ("" if r["freq_unc_ppb"] is None else repr(r["freq_unc_ppb"])),
            ("" if r["time_acc_ns"] is None else repr(r["time_acc_ns"])),
            ("" if r["sv_count"] is None else r["sv_count"]),
            ("" if r["freq_offset_ppb"] is None else repr(r["freq_offset_ppb"])),
            ("" if r["resonator_temp_c"] is None else repr(r["resonator_temp_c"])),
            ("" if r["supply_v"] is None else repr(r["supply_v"])),
            ("" if r["heater_power_w"] is None else repr(r["heater_power_w"])),
            ("" if r["heater_power_target_w"] is None else repr(r["heater_power_target_w"])),
            ("" if r["temp_error_c"] is None else repr(r["temp_error_c"])),
            ("" if r["cm4_soc_temp_c"] is None else repr(r["cm4_soc_temp_c"])),
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
