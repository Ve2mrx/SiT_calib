#!/bin/bash
echo "$@"

source /home/ve2mrx/project/ubx-data/env/bin/activate

cd /home/ve2mrx/project/ubx-data/mbt-ubx-apps
STATEFILE=~/SiT-calib_state.json
INTERVAL=86400
if [ -z "$1" ]; then
	if [ -f "$STATEFILE" ] && [ $(($(date +%s) - $(stat -c %Y "$STATEFILE"))) -le "$INTERVAL" ]; then
		WaitforTow=
	else
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
