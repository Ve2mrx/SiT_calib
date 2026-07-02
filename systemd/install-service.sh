#!/bin/sh
# Installs restart-calib.service as a systemd user unit and enables it,
# so restart-calib.sh runs automatically after every reboot.
#
# No sudo needed, but the service can only start at boot without an
# active login session if lingering is enabled for this user:
#   loginctl enable-linger "$USER"

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
UNIT_DIR="$HOME/.config/systemd/user"

mkdir -p "$UNIT_DIR"
cp "$SCRIPT_DIR/restart-calib.service" "$UNIT_DIR/restart-calib.service"

systemctl --user daemon-reload
systemctl --user enable restart-calib.service

echo "Installed and enabled restart-calib.service."
echo "Run 'systemctl --user start restart-calib.service' to test it now,"
echo "or 'journalctl --user -u restart-calib.service' to see its output."
