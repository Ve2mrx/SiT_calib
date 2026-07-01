#!/bin/bash
echo "$@"

source /home/ve2mrx/project/ubx-data/env/bin/activate

cd /home/ve2mrx/project/ubx-data/mbt-ubx-apps
if [ -z "$1" ]; then
	read -p "Enter TOW: " WaitforTow
else
	WaitforTow=$1
fi
echo
python3 ./get-data.py -F True -W "$WaitforTow" -i 86400 -P "/dev/ttyS0" -O ~/SiT-calib_output.txt "${@:2}"
status=$?

deactivate

if [ $status -ne 0 ]; then
	echo "get-data.py exited with an error (status $status)."
fi

#echo $0 " <TOW>"
while read -r -t 0.001; do :; done # dump the buffer
read -rsp $'Press any key to exit...\n' -n1 key
