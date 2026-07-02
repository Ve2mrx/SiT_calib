#!/bin/bash
# Symlink this project's executable scripts into ~/bin.
# Safe to re-run: skips links that already point here, backs up and
# replaces anything else (stale real file, wrong target, etc).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN_DIR="$HOME/bin"

SCRIPTS=(
    get-calib-screen.sh
    set-calib-screen.sh
    start-get-data.sh
    restart-calib.sh
    check-SiT5721-defaults.py
)

mkdir -p "$BIN_DIR"

for name in "${SCRIPTS[@]}"; do
    src="$SCRIPT_DIR/$name"
    dest="$BIN_DIR/$name"

    if [ ! -e "$src" ]; then
        echo "SKIP   $name: not found in $SCRIPT_DIR"
        continue
    fi

    if [ -L "$dest" ] && [ "$(readlink -f "$dest")" = "$(readlink -f "$src")" ]; then
        echo "OK     $name already linked"
        continue
    fi

    if [ -e "$dest" ] || [ -L "$dest" ]; then
        backup="$dest.bak-$(date +%Y%m%d%H%M%S)"
        echo "BACKUP $dest -> $backup"
        mv "$dest" "$backup"
    fi

    ln -s "$src" "$dest"
    echo "LINK   $dest -> $src"
done
