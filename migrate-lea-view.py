#!/usr/bin/env python3
"""
One-off (2026-10-14): convert ~/lea-view.csv to the LEAVIEW,<version>,...
line format that lea_view.py writes since then.

lea_view.py first wrote (14:40Z-15:xxZ, 2026-10-14) a plain CSV with a
header row and a bare version column, and on the v1 -> v2 column change it
renamed the old file aside as ~/lea-view.v1-<stamp>Z.csv. Neither matched
how the other device logs work (SiT5721's sit-health.csv HEALTH,<v>,...;
parse_sit.py's CSV,<v>,...): tagged, self-versioned lines, no header, all
versions in one file, archives as ~/SiT-calib_archive/<stem>_<UTC>.<ext>.

This merges every legacy line from ~/lea-view.csv and ~/lea-view.v*-*.csv
into ~/lea-view.csv as LEAVIEW lines, in time order:
- header lines ("version,utc,...") are checked against the known v1/v2
  layout and dropped; a header that matches neither -> refuse;
- legacy data lines "<v>,<ts>Z,..." become "LEAVIEW,<v>,<ts>+00:00,...";
- lines that are already LEAVIEW are kept byte-for-byte (so a second run,
  or a run after the new logger has started appending, is safe);
- anything else -> refuse, nothing written.
Every output line must parse with lea_view.parse_lea_view_line(), and the
output line count must equal the input data-line count.

Nothing is deleted: the original files are MOVED to ~/SiT-calib_archive/ as
lea-view-pre-tag_<stamp>.csv / lea-view-v1-pre-tag_<stamp>.csv (archive
naming convention; nas-sync pushes that folder).

The capture keeps running: get-data.py appends one line a minute with
open(..., "a"). The rewrite waits until the file was just written, and
re-checks size+mtime right before the atomic os.replace(); if the logger
wrote in between, it starts over.

Usage (from the repo dir on the device):
    ../env/bin/python migrate-lea-view.py            # dry run (default)
    ../env/bin/python migrate-lea-view.py --commit
"""

import argparse
import glob
import os
import sys
import time
from datetime import datetime, timezone

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lea_view  # noqa: E402

ARCHIVE_DIR = os.path.expanduser("~/SiT-calib_archive")

# The header lines the legacy writer produced, per version.
LEGACY_HEADERS = {
    v: ",".join(["version", "utc"] + fields)
    for v, fields in lea_view.LEA_VIEW_FIELDS.items()
}


class MigrationError(Exception):
    pass


def convert_lines(lines, source=""):
    """
    Legacy/LEAVIEW lines -> list of (timestamp, LEAVIEW line). Raises
    MigrationError on anything it does not recognise.
    """
    out = []
    for n, raw in enumerate(lines, start=1):
        line = raw.rstrip("\r\n")
        where = f"{source}:{n}"
        if not line:
            continue
        if line.startswith(lea_view.LINE_TAG + ","):
            parsed = lea_view.parse_lea_view_line(line)
            if parsed is None:
                raise MigrationError(f"{where}: malformed LEAVIEW line: {line[:60]}")
            out.append((parsed[0], line))
            continue
        if line.startswith("version,"):
            if line not in LEGACY_HEADERS.values():
                raise MigrationError(f"{where}: unknown header: {line[:60]}")
            continue
        parts = line.split(",")
        if not parts[0].isdigit():
            raise MigrationError(f"{where}: unrecognised line: {line[:60]}")
        version = int(parts[0])
        fields = lea_view.LEA_VIEW_FIELDS.get(version)
        if fields is None or len(parts) != 2 + len(fields):
            raise MigrationError(
                f"{where}: v{version} line has {len(parts)} fields, "
                f"expected {2 + len(fields) if fields else '?'}")
        try:
            ts = datetime.fromisoformat(parts[1])
        except ValueError as err:
            raise MigrationError(f"{where}: bad timestamp {parts[1]!r}") from err
        if ts.tzinfo is None:
            raise MigrationError(f"{where}: timestamp without timezone {parts[1]!r}")
        ts = ts.astimezone(timezone.utc)
        new = ",".join([lea_view.LINE_TAG, str(version), ts.isoformat()] + parts[2:])
        if lea_view.parse_lea_view_line(new) is None:
            raise MigrationError(f"{where}: converted line does not parse: {new[:60]}")
        out.append((ts, new))
    return out


def data_line_count(lines):
    return sum(1 for l in lines if l.strip() and not l.startswith("version,"))


def build(main_path, aside_paths):
    """Returns (output_lines, input_data_count, per-file summary)."""
    entries, count, summary = [], 0, []
    for path in [main_path] + aside_paths:
        if not os.path.exists(path):
            continue
        with open(path) as f:
            lines = f.readlines()
        conv = convert_lines(lines, os.path.basename(path))
        n_data = data_line_count(lines)
        if len(conv) != n_data:
            raise MigrationError(f"{path}: {n_data} data lines but {len(conv)} converted")
        count += n_data
        summary.append((path, len(lines), n_data))
        entries.extend(conv)
    # Stable sort: equal timestamps keep file order.
    entries.sort(key=lambda e: e[0])
    out = [line for _, line in entries]
    if len(out) != count:
        raise MigrationError(f"output {len(out)} lines != input {count}")
    return out, count, summary


def wait_for_fresh_write(path, max_wait=75.0, fresh=5.0):
    """Waits (<= max_wait s) until `path` was modified < `fresh` s ago."""
    deadline = time.time() + max_wait
    while time.time() < deadline:
        if time.time() - os.stat(path).st_mtime < fresh:
            return
        time.sleep(0.5)
    raise MigrationError(f"{path} not written in {max_wait:.0f} s - is the logger running?")


def archive_name(stem, stamp):
    return os.path.join(ARCHIVE_DIR, f"{stem}_{stamp}.csv")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--file", default=os.path.expanduser("~/lea-view.csv"))
    ap.add_argument("--commit", action="store_true", help="rewrite (default: dry run)")
    ap.add_argument("--no-wait", action="store_true",
                    help="do not wait for a fresh logger write (logger not running)")
    args = ap.parse_args(argv)

    main_path = args.file
    stem, _ = os.path.splitext(main_path)
    aside = sorted(glob.glob(f"{stem}.v*-*.csv"))

    out, count, summary = build(main_path, aside)
    for path, n_lines, n_data in summary:
        print(f"  {path}: {n_lines} lines, {n_data} data")
    legacy = sum(1 for l in out if not l.startswith(lea_view.LINE_TAG + ","))
    print(f"  -> {len(out)} LEAVIEW lines ({legacy} untagged - must be 0), "
          f"{out[0][:40] if out else ''} ... {out[-1][:40] if out else ''}")
    if legacy:
        raise MigrationError("untagged lines in output")

    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H%M%S")
    backups = [(main_path, archive_name("lea-view-pre-tag", stamp))]
    for i, path in enumerate(aside):
        tag = os.path.basename(path)[len(os.path.basename(stem)) + 1:].split("-", 1)[0]
        backups.append((path, archive_name(f"lea-view-{tag}-pre-tag" + (f"-{i}" if i else ""), stamp)))
    for src, dst in backups:
        print(f"  archive {src} -> {dst}")

    if not args.commit:
        print("DRY RUN - nothing changed. Re-run with --commit.")
        return 0

    os.makedirs(ARCHIVE_DIR, exist_ok=True)
    for _, dst in backups:
        if os.path.exists(dst):
            raise MigrationError(f"{dst} exists - refusing to overwrite")

    for attempt in range(1, 4):
        if not args.no_wait:
            wait_for_fresh_write(main_path)
        before = os.stat(main_path)
        out, count, _ = build(main_path, aside)
        tmp = main_path + ".migrating"
        with open(tmp, "w") as f:
            f.write("".join(l + "\n" for l in out))
        # Keep the original by hard link before the swap, so the archive copy
        # is the exact pre-migration file (no window where it exists nowhere).
        os.link(main_path, backups[0][1])
        after = os.stat(main_path)
        if (after.st_size, after.st_mtime_ns) != (before.st_size, before.st_mtime_ns):
            os.unlink(backups[0][1])
            os.unlink(tmp)
            print(f"  logger wrote during attempt {attempt} - retrying")
            continue
        os.replace(tmp, main_path)
        break
    else:
        raise MigrationError("logger kept writing during the rewrite - gave up")

    for src, dst in backups[1:]:
        os.rename(src, dst)

    with open(main_path) as f:
        final = f.readlines()
    bad = [l for l in final if lea_view.parse_lea_view_line(l) is None]
    print(f"DONE  {main_path}: {len(final)} lines, {len(bad)} unparseable")
    return 1 if bad else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except MigrationError as err:
        print(f"REFUSED: {err}", file=sys.stderr)
        sys.exit(2)
