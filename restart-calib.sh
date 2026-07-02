#!/bin/sh
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
if screen -list | grep -qE '\.SiT-calib[[:space:]]'; then
	echo "SiT-calib screen running."
else
	echo "SiT-calib screen failed to start!" >&2
	exit 1
fi
