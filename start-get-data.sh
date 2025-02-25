#!/bin/bash
echo $@

source /home/ve2mrx/project/ubx-data/env/bin/activate

cd /home/ve2mrx/project/ubx-data/mbt-ubx-apps
if [ "$1mark" == "mark" ]; then
	read -p "Enter TOW: " WaitforTow
else
	WaitforTow=$1
fi
echo 
python3 ./get-data.py -F True -W $WaitforTow -i 86400 -O ~/SiT-calib_output.txt $2 $3 $4 $5 $6 $7 $8 $9

deactivate

#echo $0 " <TOW>"
while read -r -t 0.001; do :; done # dump the buffer
read -rsp $'Press any key to exit...\n' -n1 key
