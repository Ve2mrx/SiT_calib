#!/bin/sh
screen -S SiT-calib -t SiT-calib -U /bin/bash -c '/home/ve2mrx/project/ubx-data/mbt-ubx-apps/start-get-data.sh '$@';'
#watch -n 600 /bin/bash
