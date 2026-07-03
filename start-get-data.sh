#!/bin/bash
echo "$@"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

source "$SCRIPT_DIR/../env/bin/activate"

cd "$SCRIPT_DIR"
STATEFILE=~/SiT-calib_state.json
INTERVAL=86400
MAIL_FAIL_LOG=~/SiT-calib_mail-failures.log

# `mail`/bsd-mailx always exits 0 even when the underlying sendmail (msmtp)
# fails to deliver, so it can't be used to detect a failed send. Call msmtp
# directly instead, whose exit code is reliable, and retry with backoff to
# ride out the DNS-not-ready-yet window right after boot (observed cause of
# a silently dropped alert: `journalctl --user -u restart-calib.service`
# showed msmtp's own "Temporary failure in name resolution" at that boot).
send_urgent_mail() {
	local subject="$1" body="$2" recipient="virusmsg@ve2mrx.dyndns.info"
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

if [ -z "$1" ]; then
	if [ -f "$STATEFILE" ] && [ $(($(date +%s) - $(stat -c %Y "$STATEFILE"))) -le "$INTERVAL" ]; then
		WaitforTow=
	else
		if [ -f "$STATEFILE" ]; then
			reason="state file $STATEFILE is stale (older than $((INTERVAL / 3600))h)"
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
fi

#echo $0 " <TOW>"
while read -r -t 0.001; do :; done # dump the buffer
read -rsp $'Press any key to exit...\n' -n1 key
