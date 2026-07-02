#!/bin/sh
SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)

if screen -list | grep -qE '\.SiT-calib[[:space:]]'; then
	echo "A SiT-calib screen session is already running." >&2
	echo "Use get-calib-screen.sh to reattach, or 'screen -X -S SiT-calib quit' to stop it first." >&2
	exit 1
fi

screen -d -m -S SiT-calib -t SiT-calib -U /bin/bash -c 'exec "$0" "$@"' "$SCRIPT_DIR/start-get-data.sh" "$@"
