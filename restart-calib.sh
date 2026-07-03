#!/bin/bash
# Run after an OS restart to resume calibration capture.
#
# Checks whether the SiT5721 kept its calibration through the restart:
# - Retained (simple OS reboot, chip stayed powered): verify/start the
#   capture screen (resumes its TOW automatically).
# - Reset to defaults (the chip itself lost power): report it and stop.
#   Recalibrating (new pull_value/aging_compensation) is a manual decision
#   made once fresh capture data is available - nothing is written here.
#
# The SiT5721 save-state screen is started separately at reboot via
# restart-SiT-screen.sh (in the SiT5721 project).

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

python3 "$SCRIPT_DIR/check-SiT5721-defaults.py"
status=$?

if [ $status -eq 0 ]; then
	echo
	echo "Power failure detected: SiT5721 calibration was reset to defaults."
	echo "Not starting anything automatically. Once you have a few days of"
	echo "fresh capture data, compute a new pull_value/aging_compensation"
	echo "and apply it manually (see write-SiT5721.py)."
	exit 0
fi

echo
echo "SiT5721 calibration retained: simple reboot. Verifying screens..."

"$SCRIPT_DIR/set-calib-screen.sh"
if ! screen -list | grep -qE '\.SiT-calib[[:space:]]'; then
	echo "SiT-calib screen failed to start!" >&2
	exit 1
fi
echo "SiT-calib screen running."

# Normal-priority reboot confirmation (distinct from the URGENT alerts):
# only sent when the previous TOW/state was fresh enough to resume
# automatically - start-get-data.sh will make the same freshness check
# itself moments later inside the screen; nothing touches STATEFILE in
# between, so this is a faithful preview of which path it will take.
STATEFILE=~/SiT-calib_state.json  # must match start-get-data.sh
INTERVAL=86400                    # must match start-get-data.sh
ALERT_CONFIG="${ALERT_CONFIG:-/home/ve2mrx/.config/sit-alerts.conf}"
[ -f "$ALERT_CONFIG" ] && . "$ALERT_CONFIG"
MAIL_FAIL_LOG=~/restart-calib_mail-failures.log

if [ -f "$STATEFILE" ] && [ $(($(date +%s) - $(stat -c %Y "$STATEFILE"))) -le "$INTERVAL" ]; then
	state_age_min=$(( ($(date +%s) - $(stat -c %Y "$STATEFILE")) / 60 ))
	tow_info=$(python3 -c "
import json
with open('$STATEFILE') as f:
    d = json.load(f)
print(f\"TOW {d['TOW_selected']} (week {d['week']}), saved {d['saved_at']}\")
" 2>/dev/null)
	subject="SiT-calib: reboot resumed capture on $(hostname)"
	body="Device rebooted at $(date -Is). SiT5721 calibration retained (no reset). Previous ${tow_info:-state in $STATEFILE} had not expired (saved ${state_age_min}m ago, limit $((INTERVAL / 60))m) and was used automatically to resume capture - no manual TOW entry needed."

	if [ -z "$ALERT_RECIPIENT" ]; then
		echo "$(date -Is) ALERT_RECIPIENT not set - create $ALERT_CONFIG" >>"$MAIL_FAIL_LOG"
	else
		attempt=0
		delay=5
		max_attempts=6
		sent=1
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
			echo "WARNING: failed to send reboot confirmation email, see $MAIL_FAIL_LOG" >&2
		fi
	fi
fi
