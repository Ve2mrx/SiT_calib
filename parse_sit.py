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
# total_log keep the raw log values for reference. total_log is appended last so
# the macro's field positions (dow_fr at index 14) are unchanged.
CSV_FIELDS = ["idx", "date", "time", "wno", "itow", "phase_ns",
              "total_ppm", "pull_ppm", "aging_pps", "retune", "flags", "status",
              "pull_log8g", "aging_log8g", "dow_fr", "total_log"]

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
            utc = UTC_RE.search(block)
            pull = PULL_RE.search(block)
            aging = AGING_RE.search(block)

            date = utc.group(1) if utc else None
            time = utc.group(2) if utc else None

            pull_8g = float(pull.group(1)) if pull else None
            aging_8g = float(aging.group(1)) if aging else None
            total_log = float(m.group("total"))                       # raw log total (10 sig figs)

            rec = {
                "date":        date,                                   # calendar date (UTC)
                "time":        time,
                "wno":         int(m.group("wno")),                    # -> row 9 (WNO end, measured)
                "itow":        float(m.group("itow")),                 # -> row 10 (iTOW end, measured)
                "phase_ns":    float(m.group("phase")),                # -> row 11 (u-blox TIM-SMEAS ns)
                "total_ppm":   pull_full_ppm(total_log),               # -> row 12, full f32 register value
                "pull_ppm":    pull_full_ppm(pull_8g),                 # -> row 1 (re-tune only) full f32
                "aging_pps":   aging_full(aging_8g),                   # -> row 2 (re-tune only) full f32
                "pull_log8g":  pull_8g,                                # raw 8-sig-fig log value (reference)
                "aging_log8g": aging_8g,
                "dow_fr":      weekday_fr(date),                       # -> row 14 (Day of week, French)
                "flags":       m.group("flags").strip(),
                "status":      m.group("status").strip(),
                "total_log":   total_log,                             # raw log total (reference)
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
        print(f"Parsed {len(recs)} daily records from {path}\n")
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
