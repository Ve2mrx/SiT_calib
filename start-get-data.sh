#!/bin/bash
echo "$@"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

source "$SCRIPT_DIR/../env/bin/activate"

cd "$SCRIPT_DIR"
STATEFILE=~/SiT-calib_state.json
INTERVAL=86400
MAIL_FAIL_LOG=~/SiT-calib_mail-failures.log
# Relies on msmtp's own `aliases /etc/aliases` directive in /etc/msmtprc
# (root -> real address) - see project memory alert-config-vs-aliases-todo.
# Overridable via env var for testing without sending a real alert.
ALERT_RECIPIENT="${ALERT_RECIPIENT:-root}"

# `mail`/bsd-mailx always exits 0 even when the underlying sendmail (msmtp)
# fails to deliver, so it can't be used to detect a failed send. Call msmtp
# directly instead, whose exit code is reliable, and retry with backoff to
# ride out the DNS-not-ready-yet window right after boot (observed cause of
# a silently dropped alert: `journalctl --user -u restart-calib.service`
# showed msmtp's own "Temporary failure in name resolution" at that boot).
send_urgent_mail() {
	local subject="$1" body="$2"
	local recipient="$ALERT_RECIPIENT"
	local attempt delay=5 max_attempts=6
	for attempt in $(seq 1 "$max_attempts"); do
		if printf 'Subject: %s\nTo: %s\nImportance: high\nX-Priority: 1 (Highest)\nX-MSMail-Priority: High\n\n%s\n' \
				"$subject" "$recipient" "$body" \
				| msmtp "$recipient" 2>>"$MAIL_FAIL_LOG"; then
			return 0
		fi
		echo "$(date -Is) attempt $attempt/$max_attempts failed, retrying in ${delay}s" >>"$MAIL_FAIL_LOG"
		sleep "$delay"
		delay=$((delay * 2))
	done
	return 1
}

# Reads TOW_selected from STATEFILE ourselves and prints it, but only if
# it's not stale by the state's *own* recorded interval (same check
# get-data.py's load_tow_state() does internally). This used to be left
# for get-data.py to figure out on its own by passing no -W - but that ran
# a second, independent freshness check moments later, which could
# disagree with this one right around the interval boundary (e.g. a slow
# boot). When they disagreed the wrong way (this script trusting the
# state, get-data.py then rejecting it), get-data.py would silently run
# forever with no capture target and no error, since args.WaitTOW stayed
# False and all the wait/capture logic is gated on it. Doing the read
# (and the freshness check) once, here, and passing -W explicitly removes
# the race entirely - get-data.py's own auto-resume path is now only ever
# exercised when someone runs it directly without -S/-W for troubleshooting.
read_state_tow() {
	python3 -c "
import json
from datetime import datetime, timezone
with open('$STATEFILE') as f:
    state = json.load(f)
saved_at = datetime.fromisoformat(state['saved_at'])
age = (datetime.now(timezone.utc) - saved_at).total_seconds()
if age > state['interval']:
    raise SystemExit(1)
print(state['TOW_selected'])
" 2>/dev/null
}

if [ -z "$1" ]; then
	WaitforTow=
	if [ -f "$STATEFILE" ]; then
		WaitforTow=$(read_state_tow)
	fi
	if [ -z "$WaitforTow" ]; then
		if [ -f "$STATEFILE" ]; then
			reason="state file $STATEFILE is stale (older than its recorded interval) or unreadable"
		else
			reason="no state file found at $STATEFILE"
		fi
		if ! send_urgent_mail "⚠ URGENT SiT-calib: manual TOW entry needed on $(hostname)" \
				"$reason; manual TOW entry required to resume SiT-calib capture on $(hostname)."; then
			echo "WARNING: failed to send manual-TOW alert email, see $MAIL_FAIL_LOG" >&2
		fi
		read -p "Enter TOW: " WaitforTow
	fi
else
	WaitforTow=$1
fi
echo
args=(-F True -i "$INTERVAL" -P "/dev/ttyS0" -O ~/SiT-calib_output.txt -S "$STATEFILE")
if [ -n "$WaitforTow" ]; then
	args+=(-W "$WaitforTow")
fi
python3 ./get-data.py "${args[@]}" "${@:2}"
status=$?

deactivate

if [ $status -ne 0 ]; then
	echo "get-data.py exited with an error (status $status)."
	if ! send_urgent_mail "⚠ URGENT SiT-calib: get-data.py exited with an error on $(hostname)" \
			"get-data.py exited with status $status on $(hostname) at $(date -Is). SiT-calib capture has stopped. Check the SiT-calib screen output."; then
		echo "WARNING: failed to send get-data.py failure alert email, see $MAIL_FAIL_LOG" >&2
	fi
fi

#echo $0 " <TOW>"
while read -r -t 0.001; do :; done # dump the buffer
read -rsp $'Press any key to exit...\n' -n1 key
