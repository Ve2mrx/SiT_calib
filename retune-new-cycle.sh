#!/bin/bash
# Start a new calibration cycle after a MANUAL Pull/Aging re-tune.
#
# Archives ~/SiT-calib_output.txt + ~/parsed_records.json/.csv to
# ~/SiT-calib_archive/ and opens a fresh capture log carrying a
# `#`-comment header describing the re-tune - the same end state
# restart-calib.sh produces after a power loss, but for the manual path,
# which had no equivalent step and was being done by hand.
#
# WHY THIS EXISTS: restart-calib.sh's archive/rotate block is gated on
# ~/SiT-power-loss-mark.json and lives only in the power-loss path. A
# manual re-tune (write-SiT5721.py --commit) leaves the capture log
# appending across the register change, so parsed_records.csv spans two
# cycles and the analysis spreadsheet gets no clean boundary. Cycle 60707
# began at a power loss, so the automation hid this gap until the manual
# re-tune of 2026-08-18.
#
# DO NOT use restart-calib.sh for this. It is the reboot/power-loss path,
# is gated on a mark file that does not exist here, and would report a
# power failure that did not happen.
#
# DRY RUN IS THE DEFAULT - prints the plan and touches nothing. Pass
# --commit to act. This matches write-SiT5721.py, deliberately: both
# scripts destroy calibration history if run wrong.
#
# SAFE WHILE CAPTURE IS RUNNING, and the SiT-calib screen needs no
# restart: get-data.py writes the log with open(file, "a") per capture
# rather than holding it open, so the next capture recreates it by name.
#
# Metadata is read from the last row of ~/SiT5721-write-log.csv, so
# nothing has to be retyped - and the archive timestamp is derived from
# the write itself, which is what makes a re-run idempotent.
#
# Usage:
#   retune-new-cycle.sh                 # dry run - print the plan
#   retune-new-cycle.sh --commit        # archive + open a fresh log
#   retune-new-cycle.sh --commit --force  # bypass the capture-window guard
#
# Companion: manual-retune-new-cycle-procedure.md (Excel side + naming).

set -uo pipefail

COMMIT=0
FORCE=0
CALIB_LOG=~/SiT-calib_output.txt
PARSED_JSON=~/parsed_records.json
PARSED_CSV=~/parsed_records.csv
STATE_FILE=~/SiT-calib_state.json
WRITE_LOG=~/SiT5721-write-log.csv
MARK_FILE=~/SiT-power-loss-mark.json
ARCHIVE_DIR=~/SiT-calib_archive

# Refuse to rotate within this many minutes of the next capture. Rotating
# in that window risks the capture landing in whichever file it opened
# first, which is exactly the ambiguity this script exists to prevent.
GUARD_MIN=15

usage() {
	sed -n '2,42p' "$0" | sed 's/^# \{0,1\}//'
	exit "${1:-0}"
}

while [ $# -gt 0 ]; do
	case "$1" in
	--commit) COMMIT=1 ;;
	--force) FORCE=1 ;;
	--log) CALIB_LOG="$2"; shift ;;
	--write-log) WRITE_LOG="$2"; shift ;;
	--archive) ARCHIVE_DIR="$2"; shift ;;
	-h | --help) usage 0 ;;
	*) echo "ERROR  unknown argument: $1" >&2; usage 1 >&2 ;;
	esac
	shift
done

die() { echo "REFUSED  $*" >&2; exit 1; }

# ---------------------------------------------------------------- checks
[ -f "$CALIB_LOG" ] || die "capture log not found: $CALIB_LOG"
[ -f "$WRITE_LOG" ] || die "write log not found: $WRITE_LOG
         Nothing to derive a new epoch from. Run write-SiT5721.py --commit first."

# A fresh power-loss mark means restart-calib.sh owns this boundary, not us.
if [ -f "$MARK_FILE" ]; then
	die "$MARK_FILE exists - a power loss is pending processing.
         restart-calib.sh handles that path. Resolve it before using this script."
fi

# Pull the last write-log row. Done in python3 (not cut/awk) because the
# CSV is written by csv.writer and the note field may be quoted and
# contain commas - splitting on ',' in shell silently misreads it.
meta=$(python3 - "$WRITE_LOG" <<'PY'
import csv, sys, datetime
rows = list(csv.DictReader(open(sys.argv[1])))
if not rows:
    print("ERR|write log has a header but no data rows"); raise SystemExit
r = rows[-1]
if (r.get("write_error") or "").strip():
    print("ERR|last write FAILED (write_error=%s) - do not open a new cycle on it"
          % r["write_error"]); raise SystemExit
try:
    ts = datetime.datetime.fromisoformat(r["written_utc"])
except Exception as exc:
    print("ERR|cannot parse written_utc=%r (%s)" % (r.get("written_utc"), exc)); raise SystemExit
print("OK|%s|%s|%s|%s|%s" % (
    ts.strftime("%Y-%m-%dT%H%M%S"),      # archive stamp
    r["written_utc"],
    r.get("after_pull") or r.get("pull") or "?",
    r.get("after_aging") or r.get("aging") or "?",
    (r.get("note") or "").replace("|", "/").strip() or "(no note)"))
PY
)
case "$meta" in
ERR\|*) die "${meta#ERR|}" ;;
OK\|*) ;;
*) die "could not read $WRITE_LOG" ;;
esac
IFS='|' read -r _ TS WRITTEN_UTC PULL AGING NOTE <<<"$meta"

# Idempotence: refuse if this exact write has already been rotated.
if ls "$ARCHIVE_DIR"/*_"$TS".* >/dev/null 2>&1; then
	die "already rotated for this write - $ARCHIVE_DIR holds *_$TS.*
         Re-running would archive the NEW cycle's captures under a stale name."
fi
if grep -qF "$WRITTEN_UTC" "$CALIB_LOG" 2>/dev/null; then
	die "$CALIB_LOG already carries a header for this write ($WRITTEN_UTC).
         The rotation appears to have been done already."
fi

# The seed must be the first capture AFTER the write. If a capture has
# already landed post-write, it belongs in the NEW log - rotating now
# would archive the seed away, which is unrecoverable without editing
# files by hand.
last_cap=$(grep -oE '[0-9]{4}-[0-9]{2}-[0-9]{2}, [0-9]{2}:[0-9]{2}:[0-9]{2}\+00:00' "$CALIB_LOG" 2>/dev/null | tail -1)
if [ -n "$last_cap" ]; then
	late=$(python3 - "$last_cap" "$WRITTEN_UTC" <<'PY'
import sys, datetime
d, t = sys.argv[1].split(", ")
cap = datetime.datetime.fromisoformat(d + "T" + t)
print("1" if cap > datetime.datetime.fromisoformat(sys.argv[2]) else "0")
PY
)
	if [ "$late" = "1" ]; then
		die "the capture log already contains a capture ($last_cap) LATER than
         the write ($WRITTEN_UTC). That capture is the day-0 seed and belongs
         in the new log. Rotating now would archive it away.
         Split the log by hand, or seed the spreadsheet from the archived file."
	fi
fi

# Capture-window guard: derive the next capture from the saved state.
next_cap=""
if [ -f "$STATE_FILE" ]; then
	next_cap=$(python3 - "$STATE_FILE" <<'PY'
import json, sys, datetime
try:
    d = json.load(open(sys.argv[1]))
    t = datetime.datetime.fromisoformat(d["saved_at"]) + datetime.timedelta(seconds=int(d["interval"]))
    mins = (t - datetime.datetime.now(datetime.timezone.utc)).total_seconds() / 60.0
    print("%s|%.0f" % (t.isoformat(), mins))
except Exception:
    print("|")
PY
)
fi
NEXT_AT=${next_cap%%|*}
NEXT_MIN=${next_cap##*|}
if [ -n "$NEXT_MIN" ] && [ "$NEXT_MIN" -lt "$GUARD_MIN" ] 2>/dev/null; then
	if [ "$FORCE" -eq 0 ]; then
		die "next capture is in ${NEXT_MIN} min (< ${GUARD_MIN}) at $NEXT_AT.
         Rotating this close risks the capture landing in the archived file.
         Wait for it and rotate after, or re-run with --force."
	fi
	echo "WARNING  next capture in ${NEXT_MIN} min - proceeding because --force"
fi

# ------------------------------------------------------------------ plan
echo "New calibration cycle from a manual re-tune"
echo "  write            $WRITTEN_UTC"
echo "  pull (on device) $PULL"
echo "  aging            $AGING"
echo "  note             $NOTE"
echo "  archive stamp    $TS"
[ -n "$NEXT_AT" ] && echo "  next capture     $NEXT_AT  (in ${NEXT_MIN} min) <- becomes day-0 seed"
echo
echo "  archive to $ARCHIVE_DIR/"
for f in "$CALIB_LOG" "$PARSED_JSON" "$PARSED_CSV"; do
	base=$(basename "$f"); stem=${base%.*}; ext=${base##*.}
	if [ -f "$f" ]; then
		echo "    $base  ->  ${stem}_$TS.$ext"
	else
		echo "    $base  (absent - skipped)"
	fi
done
echo "  then open a fresh $CALIB_LOG with a 3-line '#' header"

if [ "$COMMIT" -eq 0 ]; then
	echo
	echo "DRY RUN - nothing was changed. Re-run with --commit to apply."
	exit 0
fi

# ---------------------------------------------------------------- commit
# All three files must move together. regenerate() refuses to overwrite a
# parsed_records.json holding MORE records than it is about to write, and
# a fresh log parses to 1 record against the old file's N - so leaving the
# parsed files behind trips the regression guard on the next capture.
# Archiving them makes the guard see a nonexistent file and count from
# zero, with no --force needed. Never use --force to paper over that.
mkdir -p "$ARCHIVE_DIR" || die "cannot create $ARCHIVE_DIR"

moved=0
for f in "$CALIB_LOG" "$PARSED_JSON" "$PARSED_CSV"; do
	[ -f "$f" ] || continue
	base=$(basename "$f"); stem=${base%.*}; ext=${base##*.}
	if mv "$f" "$ARCHIVE_DIR/${stem}_$TS.$ext"; then
		moved=$((moved + 1))
	else
		echo "ERROR  failed to archive $f" >&2
		echo "       $moved file(s) already moved - state is PARTIAL." >&2
		echo "       Move the rest by hand before the next capture." >&2
		exit 1
	fi
done

{
	echo "# Manual Pull re-tune - previous capture history archived to"
	echo "# $ARCHIVE_DIR/*_$TS.*. New calibration epoch:"
	echo "# written_at=$WRITTEN_UTC pull_value=$PULL aging_compensation=$AGING note=$NOTE"
} >"$CALIB_LOG" || die "archived $moved file(s) but could not write $CALIB_LOG"

echo
echo "DONE  archived $moved file(s), fresh log opened."
echo
echo "Acceptance:"
ls -l "$ARCHIVE_DIR"/*_"$TS".* 2>/dev/null | sed 's/^/  /'
echo "  --- $CALIB_LOG ---"
sed 's/^/  /' "$CALIB_LOG"
if ls ~/parsed_records.* >/dev/null 2>&1; then
	echo "  WARNING  parsed_records.* still present in \$HOME - expected none" >&2
else
	echo "  parsed_records.*  absent (correct - rebuilt at the next capture)"
fi
echo
echo "Next: the capture at ${NEXT_AT:-the next interval} becomes day 0."
echo "      Cycle suffix = that capture's UTC date as Y-MM-DD."
echo "      Then CreateNewTrio in the workbook - see"
echo "      manual-retune-new-cycle-procedure.md."
