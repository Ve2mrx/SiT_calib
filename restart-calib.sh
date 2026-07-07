#!/bin/bash
# Run after an OS restart to resume calibration capture.
#
# Checks whether the SiT5721 kept its calibration through the restart:
# - Retained (simple OS reboot, chip stayed powered): verify/start the
#   capture screen (resumes its TOW automatically).
# - Reset to defaults (the chip itself lost power), but restart-SiT5721.py's
#   aging-corrected recalc already succeeded (fresh ~/SiT-power-loss-mark.json
#   left this boot): archive the pre-power-loss SiT-calib_output.txt and
#   parsed_records.json/.csv, start a new calibration epoch, then verify/start
#   the capture screen same as above.
# - Reset to defaults and the recalc did NOT succeed (no fresh mark file):
#   report it and stop. Nothing is written here - recalibrating is then a
#   manual decision once fresh capture data is available (see write-SiT5721.py).
#
# systemd/restart-calib.service runs this After= SiT5721's own
# restart-sit5721-pull.service, which runs restart-SiT5721.py - so by the
# time we check here, a real power loss has already been recalculated and
# reloaded if it's going to be. See project memory power-loss-mark-todo.
#
# The SiT5721 Pull Value restore is done separately at reboot via
# restart-SiT5721-pull.sh (in the SiT5721 project).

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$(readlink -f -- "$0")")" && pwd)

python3 "$SCRIPT_DIR/check-SiT5721-defaults.py"
status=$?

if [ $status -eq 0 ]; then
	echo
	echo "Power failure detected: SiT5721 calibration was reset to defaults,"
	echo "and restart-SiT5721.py's automatic recalc did not succeed (no fresh"
	echo "mark file found). Not starting anything automatically. Once you have"
	echo "a few days of fresh capture data, compute a new pull_value/aging_compensation"
	echo "and apply it manually (see write-SiT5721.py)."
	exit 0
fi

CALIB_LOG=~/SiT-calib_output.txt
PARSED_JSON=~/parsed_records.json
PARSED_CSV=~/parsed_records.csv
MARK_FILE=~/SiT-power-loss-mark.json
ARCHIVE_DIR=~/SiT-calib_archive

# Only treat the mark file as "this boot's event" if it postdates boot -
# a mark left over from a prior, incompletely-processed boot must not
# retrigger archiving on an unrelated later reboot.
mark_info=$(python3 -c "
import json
from datetime import datetime, timezone, timedelta

try:
    with open('$MARK_FILE') as f:
        d = json.load(f)
except FileNotFoundError:
    raise SystemExit(0)

with open('/proc/uptime') as f:
    boot_time = datetime.now(timezone.utc) - timedelta(seconds=float(f.read().split()[0]))

detected_at = datetime.fromisoformat(d['detected_at'])
if detected_at < boot_time:
    print('STALE')
else:
    ts = detected_at.strftime('%Y-%m-%dT%H%M%S')  # UTC, to the second - unique enough, no colons/extension games
    print('FRESH')
    print(ts)
    print(f\"detected_at={d['detected_at']} delta_t={d['delta_t_seconds']:.0f}s \"
          f\"restart_pull_value={d['restart_pull_value']!r} \"
          f\"aging_compensation={d['aging_compensation']!r}\")
")
mark_status=$(echo "$mark_info" | sed -n 1p)

power_loss_event=0
if [ "$mark_status" = "FRESH" ]; then
	power_loss_event=1
	ts=$(echo "$mark_info" | sed -n 2p)
	mark_summary=$(echo "$mark_info" | sed -n 3p)

	echo
	echo "SiT5721 calibration retained: power loss detected and automatically"
	echo "recalculated/reloaded by restart-SiT5721.py ($mark_summary)."
	echo "Archiving prior capture history and starting a new calibration epoch..."

	mkdir -p "$ARCHIVE_DIR"
	[ -f "$CALIB_LOG" ] && mv "$CALIB_LOG" "$ARCHIVE_DIR/SiT-calib_output_$ts.txt"
	[ -f "$PARSED_JSON" ] && mv "$PARSED_JSON" "$ARCHIVE_DIR/parsed_records_$ts.json"
	[ -f "$PARSED_CSV" ] && mv "$PARSED_CSV" "$ARCHIVE_DIR/parsed_records_$ts.csv"
	{
		echo "# Power-loss recalibration - previous capture history archived to"
		echo "# $ARCHIVE_DIR/*_$ts.*. New calibration epoch:"
		echo "# $mark_summary"
	} >"$CALIB_LOG"
	mv "$MARK_FILE" "$ARCHIVE_DIR/SiT-power-loss-mark_$ts.json"
else
	if [ "$mark_status" = "STALE" ]; then
		echo
		echo "Ignoring stale power-loss mark file from before this boot: $MARK_FILE" >&2
		mkdir -p "$ARCHIVE_DIR"
		mv "$MARK_FILE" "$ARCHIVE_DIR/SiT-power-loss-mark-stale_$(date -u +%Y-%m-%dT%H%M%S).json"
	fi
	echo
	echo "SiT5721 calibration retained: simple reboot. Verifying screens..."
fi

"$SCRIPT_DIR/set-calib-screen.sh"
if ! screen -list | grep -qE '\.SiT-calib[[:space:]]'; then
	echo "SiT-calib screen failed to start!" >&2
	exit 1
fi
echo "SiT-calib screen running."

STATEFILE=~/SiT-calib_state.json  # must match start-get-data.sh
INTERVAL=86400                    # must match start-get-data.sh
# Relies on msmtp's own `aliases /etc/aliases` directive in /etc/msmtprc
# (root -> real address) - see project memory alert-config-vs-aliases-todo.
# Overridable via env var for testing without sending a real alert.
ALERT_RECIPIENT="${ALERT_RECIPIENT:-root}"
MAIL_FAIL_LOG=~/restart-calib_mail-failures.log

send_mail() {
	local subject="$1" body="$2"
	local attempt=0 delay=5 max_attempts=6 sent=1
	while [ "$attempt" -lt "$max_attempts" ]; do
		attempt=$((attempt + 1))
		if printf 'Subject: %s\nTo: %s\n\n%s\n' "$subject" "$ALERT_RECIPIENT" "$body" \
				| msmtp "$ALERT_RECIPIENT" 2>>"$MAIL_FAIL_LOG"; then
			sent=0
			break
		fi
		echo "$(date -Is) attempt $attempt/$max_attempts failed, retrying in ${delay}s" >>"$MAIL_FAIL_LOG"
		sleep "$delay"
		delay=$((delay * 2))
	done
	if [ "$sent" -ne 0 ]; then
		echo "WARNING: failed to send email (subject: $subject), see $MAIL_FAIL_LOG" >&2
	fi
}

if [ "$power_loss_event" -eq 1 ]; then
	subject="SiT-calib: power loss detected & auto-recalculated on $(hostname)"
	body="Device rebooted at $(date -Is). SiT5721 lost power and reset to defaults; restart-SiT5721.py recalculated and reloaded an aging-corrected Pull Value ($mark_summary). Prior capture history archived to $ARCHIVE_DIR. A new calibration epoch has started in $CALIB_LOG - update the analysis spreadsheet accordingly."
	send_mail "$subject" "$body"
elif [ -f "$STATEFILE" ] && [ $(($(date +%s) - $(stat -c %Y "$STATEFILE"))) -le "$INTERVAL" ]; then
	# Normal-priority reboot confirmation (distinct from the URGENT alerts):
	# only sent when the previous TOW/state was fresh enough to resume
	# automatically - start-get-data.sh will make the same freshness check
	# itself moments later inside the screen; nothing touches STATEFILE in
	# between, so this is a faithful preview of which path it will take.
	state_age_sec=$(( $(date +%s) - $(stat -c %Y "$STATEFILE") ))
	state_age_min=$(( state_age_sec / 60 ))
	state_age_hms=$(printf '%dh%02dm%02ds' $((state_age_sec / 3600)) $((state_age_sec % 3600 / 60)) $((state_age_sec % 60)))
	limit_min=$((INTERVAL / 60))
	tow_info=$(python3 -c "
import json
with open('$STATEFILE') as f:
    d = json.load(f)
print(f\"TOW {d['TOW_selected']} (week {d['week']}), saved {d['saved_at']}\")
" 2>/dev/null)
	subject="SiT-calib: reboot resumed capture on $(hostname)"
	body="Device rebooted at $(date -Is). SiT5721 calibration retained (no reset). Previous ${tow_info:-state in $STATEFILE} had not expired (saved ${state_age_hms} ago, ${state_age_min}m of ${limit_min}m limit) and was used automatically to resume capture - no manual TOW entry needed."
	send_mail "$subject" "$body"
fi
