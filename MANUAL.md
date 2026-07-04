# mbt-ubx-apps — install & operations manual

GPS time-transfer calibration capture: reads u-blox TIM-TOS/TIM-SMEAS/PUBX04
messages from a GNSS receiver plus SiT5721 registers over I2C, and once per
24h at a fixed GPS TOW, snapshots both to `~/SiT-calib_output.txt`. Runs
continuously in a detached `screen` session named `SiT-calib`.

After every daily snapshot, `get-data.py` also calls `parse_sit.py`'s
`regenerate()` in-process to refresh `parsed_records.json`/`.csv` (the
Excel analysis workbook's input) - no separate timer/service parses the
log. This call is wrapped in try/except: a parsing failure logs a
warning and appends to `parsed_records_errors.log` (next to the output
files) but never stops live capture. `regenerate()` also refuses to
overwrite `parsed_records.json` with fewer records than it already has
(append-only log, so a drop means a truncated/corrupted read, not real
data) - pass `--force` to `./parse_sit.py` if you ever need to
deliberately shrink it (e.g. after hand-editing the log). Run
`parse_sit.py` by hand any time to regenerate those files standalone
(e.g. after editing historical log entries).

Companion project: [SiT5721](../../SiT5721/) (the GPSDO chip's own restart/
calibration-save side). Shared alert email config is documented in both
manuals identically — see [Alert email configuration](#alert-email-configuration).

**Keep this file up to date** whenever install steps, file paths, or the
systemd/email setup change.

## Hardware & OS prerequisites

- Raspberry Pi (or similar) with:
  - A u-blox GNSS receiver on `/dev/ttyS0`
  - SiT5721 GPSDO on I2C bus 0, address `0x60`
- OS packages: `python3-venv`, `python3-smbus`, `screen`, `msmtp`, `msmtp-mta`
  (`apt install python3-venv python3-smbus screen msmtp msmtp-mta`)
- `/etc/msmtprc` configured with a working SMTP account (this host uses a
  Gmail relay). Contains a plaintext password — never echo/paste its
  contents into chat, commits, or logs.

## Directory layout

```
project/ubx-data/            <- parent folder, NOT a git repo
├── env/                     <- shared Python venv (see below)
├── env-setup.sh             <- (re)creates the venv
└── mbt-ubx-apps/            <- this repo
```

`start-get-data.sh` activates `../env/bin/activate` relative to its own
location, so the venv **must** exist as a sibling of `mbt-ubx-apps/`, at
`project/ubx-data/env`.

## 1. Install

```sh
cd project/ubx-data
git clone --recurse-submodules <mbt-ubx-apps-url> mbt-ubx-apps
./env-setup.sh                     # creates ./env, pip-installs pynmeagps/
                                    # pyubx2/pyrtcm/pyserial from PyPI
cd mbt-ubx-apps
./install.sh                       # symlinks this repo's scripts into ~/bin
```

`get-data.py` and `check-SiT5721-defaults.py` import the SiT5721 I2C
library from `lib/mbt-SiT5721-lib`, a git submodule shared with the
SiT5721 repo (single source of truth — see that submodule's own README).
If you cloned without `--recurse-submodules`, run
`git submodule update --init` before running either script.

`env-setup.sh` installs from PyPI. Periodically check for upstream updates
(`./env/bin/pip list --outdated`, or compare against
`https://pypi.org/pypi/<pkg>/json`) and review each package's
`RELEASE_NOTES.md` on GitHub for breaking changes before upgrading — last
checked/upgraded 2026-07-03 (pynmeagps 1.0.43→1.1.5, pyubx2 1.2.48→1.3.3,
pyrtcm 1.1.2→1.1.12; no breaking changes affected this codebase's usage).

## 2. Alert email configuration

Shared with the SiT5721 project. Create (once, host-wide, **not** tracked
by either git repo):

```sh
mkdir -p ~/.config
cat > ~/.config/sit-alerts.conf <<'EOF'
ALERT_RECIPIENT="you@example.com"
EOF
```

All alert-sending scripts in this repo (`start-get-data.sh`,
`systemd/restart-calib-alert.sh`) source this file and refuse to send
(fail loud in the log, not silently) if `ALERT_RECIPIENT` is unset. The
recipient **must** be a real, directly-deliverable address — `msmtp`
does not consult `/etc/aliases`, so a bare local username will fail.

Emails sent by this project:
- **Normal priority**: reboot confirmation from `restart-calib.sh`, sent
  only when the previous TOW/state was fresh and reused automatically.
- **Urgent** (`Importance: high`): manual TOW entry needed (state
  missing/stale), or `get-data.py`/`restart-calib.service` failing outright.

## 3. systemd services (boot-time automation)

```sh
cd systemd
sudo ./install-service.sh
```

Installs and enables:
- `restart-calib.service` — runs `restart-calib.sh` after every boot,
  ordered after `network-online.target`. Checks whether the SiT5721 kept
  its calibration (simple reboot) or reset to defaults (real power loss);
  if retained, verifies/starts the `SiT-calib` screen and emails a
  reboot-confirmation (see above).
- `restart-calib-alert.service` — fires automatically via
  `restart-calib.service`'s `OnFailure=`; not started directly.

## 4. First-time run (no prior state)

There's no automated first-time setup — start it manually once, with a
real TOW value in mind:

```sh
~/get-calib-screen.sh    # or: cd mbt-ubx-apps && ./set-calib-screen.sh <TOW>
```

If no TOW is given and no valid `~/SiT-calib_state.json` exists yet, it
will prompt interactively (`Enter TOW:`) and email an urgent alert while
waiting — this is intentional (see project memory
`restart-calib-manual-tow`), not a bug.

## Operations

- **Check it's running**: `screen -list | grep SiT-calib`, or
  `~/get-calib-screen.sh` to reattach (detach again with `Ctrl-A d`,
  don't kill the session).
- **See recent output without attaching**:
  `screen -S SiT-calib -X hardcopy /tmp/out.txt && cat /tmp/out.txt`
- **Check boot-time service logs**: `journalctl -u restart-calib.service`
- **Restart safely**: only stop/restart `get-data.py` when it isn't near
  its target TOW (it captures exactly once per 24h; disturbing it near
  that instant loses that day's sample). Compute the current target's UTC
  time from `~/SiT-calib_state.json` (`week`, `TOW_selected`) — see the
  GPS-TOW-to-UTC note in project memory. Then:
  ```sh
  screen -S SiT-calib -X quit     # stop cleanly
  ./restart-calib.sh              # or: ./set-calib-screen.sh <TOW>
  ```
- **Mail failures**: check `~/SiT-calib_mail-failures.log` and
  `~/restart-calib_mail-failures.log` if an expected alert never arrived.

## Key files/paths

| Path | Purpose |
|---|---|
| `~/SiT-calib_state.json` | Persisted `TOW_selected`/`week`/`interval`/`saved_at`, used to auto-resume across restarts |
| `~/SiT-calib_output.txt` | Append-only capture history (input to `parse_sit.py`) |
| `~/parsed_records.json`, `~/parsed_records.csv` | Auto-regenerated by `get-data.py` after each daily record - Excel workbook input |
| `~/parsed_records_errors.log` | Append-only log of `parse_sit.regenerate()` failures (capture itself is unaffected) |
| `lib/mbt-SiT5721-lib/` | Git submodule (shared with SiT5721) - `SiT5721` I2C class |
| `~/SiT-calib_mail-failures.log` | `send_urgent_mail()` retry/failure log (from `start-get-data.sh`) |
| `~/restart-calib_mail-failures.log` | Retry/failure log for `restart-calib.sh`'s own mail sends |
| `~/.config/sit-alerts.conf` | Shared alert recipient config (see above) |
