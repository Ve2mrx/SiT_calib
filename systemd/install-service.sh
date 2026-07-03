#!/bin/sh
# Installs restart-calib.service (+ its OnFailure= alert unit) as
# system-wide systemd units and enables the main one, so restart-calib.sh
# runs automatically after every reboot - ordered after real network
# availability (After=network-online.target), with an email alert if it
# ever fails.
#
# Needs root - run with sudo. Also disables/removes the old --user unit
# this replaces, so it doesn't run a second time at boot.

if [ "$(id -u)" -ne 0 ]; then
	echo "This installs system units; re-run with sudo." >&2
	exit 1
fi

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
UNIT_DIR="/etc/systemd/system"

cp "$SCRIPT_DIR/restart-calib.service" "$UNIT_DIR/restart-calib.service"
cp "$SCRIPT_DIR/restart-calib-alert.service" "$UNIT_DIR/restart-calib-alert.service"

systemctl daemon-reload
systemctl enable restart-calib.service

OLD_USER_UNIT="/home/ve2mrx/.config/systemd/user/restart-calib.service"
if [ -f "$OLD_USER_UNIT" ]; then
	echo "Disabling superseded --user unit..."
	# sudo -u alone doesn't give systemctl --user a session bus; XDG_RUNTIME_DIR
	# has to be set explicitly or it fails with "Failed to connect to bus".
	sudo -u ve2mrx XDG_RUNTIME_DIR=/run/user/1000 systemctl --user disable restart-calib.service
	rm -f "$OLD_USER_UNIT"
	sudo -u ve2mrx XDG_RUNTIME_DIR=/run/user/1000 systemctl --user daemon-reload
fi

echo "Installed and enabled restart-calib.service (system unit)."
echo "Run 'systemctl start restart-calib.service' to test it now,"
echo "or 'journalctl -u restart-calib.service' to see its output."
echo "restart-calib-alert.service fires automatically via OnFailure=;"
echo "it is not enabled/started directly."
