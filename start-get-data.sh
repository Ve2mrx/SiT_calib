#!/bin/bash
echo "$@"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

source "$SCRIPT_DIR/../env/bin/activate"

cd "$SCRIPT_DIR"
STATEFILE=~/SiT-calib_state.json
INTERVAL=86400
if [ -z "$1" ]; then
	if [ -f "$STATEFILE" ] && [ $(($(date +%s) - $(stat -c %Y "$STATEFILE"))) -le "$INTERVAL" ]; then
		WaitforTow=
	else
		if [ -f "$STATEFILE" ]; then
			reason="state file $STATEFILE is stale (older than $((INTERVAL / 3600))h)"
		else
			reason="no state file found at $STATEFILE"
		fi
		echo "$reason; manual TOW entry required to resume SiT-calib capture on $(hostname)." \
			| mail -s "⚠ URGENT SiT-calib: manual TOW entry needed on $(hostname)" \
				-a "Importance: high" -a "X-Priority: 1 (Highest)" -a "X-MSMail-Priority: High" \
				"virusmsg@ve2mrx.dyndns.info"
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
