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
"""

import re
import sys
import csv
import json
import struct
from datetime import datetime

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


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "SiT-calib_output.txt"
    recs = annotate_retune(parse(path))

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

    with open("parsed_records.json", "w") as fh:
        json.dump(recs, fh, indent=2)

    # CSV for the Excel VBA macro. repr() keeps full round-trip precision so
    # Excel receives the exact measured + full-register-precision values.
    with open("parsed_records.csv", "w", newline="") as fh:
        w = csv.writer(fh)
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
    print("\nWrote parsed_records.json and parsed_records.csv")


if __name__ == "__main__":
    main()
