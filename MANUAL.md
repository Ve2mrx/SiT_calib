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
See also [capture-status](../capture-status/) for a one-shot Go/No-go
health check spanning both projects.

**Keep this file up to date** whenever install steps, file paths, or the
systemd/email setup change.

## Hardware & OS prerequisites

- Raspberry Pi (or similar) with:
  - A u-blox GNSS receiver on `/dev/ttyS0`
  - SiT5721 GPSDO on I2C bus 0, address `0x60` — needs the `i2c-dev`
    kernel module loaded (creates `/dev/i2c-0`) *and* persisted across
    reboots. On a fresh Trixie image this is easy to miss if the I2C
    buses were hand-added to `config.txt` (as on this hardware:
    `i2c_vc`/`i2c5`/`i2c_csi_dsi`) rather than enabled through
    `raspi-config`, since it's specifically `raspi-config`'s I2C toggle
    that also arranges for `i2c-dev` to autoload
    (`sudo raspi-config` → Interface Options → I2C → Enable). Symptom if
    missed: `get-data.py`'s `smbus.SMBus(0)` (and `reinstall.sh`'s I2C
    check) fail because `/dev/i2c-0` doesn't exist yet — see the
    2026-07-06/07 incident below.
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

For provisioning a whole fresh device (OS reinstall/SD-card swap) rather
than just this repo, see `../reinstall.sh` — it drives the steps above
plus the SiT5721/capture-status repos, OS packages, I2C/serial checks,
and systemd units in one idempotent, re-runnable pass.

`env-setup.sh` creates `./env` with the stdlib `python3 -m venv
--system-site-packages` (not the third-party `virtualenv` package this
used before 2026-07-06) — avoids a `pip install --user` at the system
level, which Trixie's PEP 668 (`externally-managed-environment`) would
otherwise block.

`env-setup.sh` installs from PyPI. Periodically check for upstream updates
(`./env/bin/pip list --outdated`, or compare against
`https://pypi.org/pypi/<pkg>/json`) and review each package's
`RELEASE_NOTES.md` on GitHub for breaking changes before upgrading — last
checked/upgraded 2026-07-03 (pynmeagps 1.0.43→1.1.5, pyubx2 1.2.48→1.3.3,
pyrtcm 1.1.2→1.1.12; no breaking changes affected this codebase's usage).

## 2. Alert email configuration

All alert-sending scripts in this repo (`start-get-data.sh`,
`restart-calib.sh`, `systemd/restart-calib-alert.sh`) default
`ALERT_RECIPIENT` to `root` and rely on `/etc/msmtprc`'s
`aliases /etc/aliases` directive (added 2026-07-04, see project memory
`alert-config-vs-aliases-todo`) to resolve that to a real deliverable
address - no per-host config file needed anymore (retired
`~/.config/sit-alerts.conf` the same day, once this was live-verified).
On a host without that directive configured, `msmtp` does not consult
`/etc/aliases` on its own, so `root` would fail - either add the
directive there too, or override the default:
`ALERT_RECIPIENT="you@example.com" ./start-get-data.sh` (the env var is
also how to redirect a test send without touching the real alias).

Emails sent by this project:
- **Normal priority**: reboot confirmation from `restart-calib.sh`, sent
  only when the previous TOW/state was fresh and reused automatically; or
  a power-loss-detected-and-auto-recalculated notice (see
  [Power-loss handling](#power-loss-handling) below) - the two are
  mutually exclusive per boot.
- **Urgent** (`Importance: high`): manual TOW entry needed (state
  missing/stale), or `get-data.py`/`restart-calib.service` failing outright.

## 3. systemd services (boot-time automation)

```sh
cd systemd
sudo ./install-service.sh
```

Installs and enables:
- `restart-calib.service` — runs `restart-calib.sh` after every boot,
  ordered after `network-online.target` *and* SiT5721's
  `restart-sit5721-pull.service` (so a real power-loss recalc, if any, has
  already happened and left its mark before this runs). Checks whether
  the SiT5721 kept its calibration (simple reboot) or reset to defaults
  (real power loss); if retained/recovered, verifies/starts the
  `SiT-calib` screen and emails the appropriate confirmation (see
  [Power-loss handling](#power-loss-handling) below).
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

## 5. NAS sync (calibration files → Synology)

`../ubx-data/nas-sync/` one-way pushes `~/parsed_records.csv`/`.json`,
`~/SiT-calib_output.txt`, `~/SiT-calib_state.json`, `~/SiT-settings2.ini`
(read-only reference; relocated 2026-07-07 from inside the SiT5721 repo
to `$HOME` — see that project's manual), and any archived
`~/SiT-calib_output_*.txt`/`~/SiT-calib_archive/` files to the Synology NAS
(`nas-2.ve2mrx`, share `ubx-data-live`, mounted via CIFS at
`/mnt/ubx-data-live`) — from there Synology Drive syncs it to
`D:\SynologyDrive\ubx-data-live` on the PC for the manual Model B Excel
workflow (`ImportSiTCalib`). Plain `rsync -a`, never `--delete` - the Pi is
the source of truth, the NAS/Excel side only ever reads.

**Trigger is a `systemd.path` unit (`nas-sync.path`), not a timer** —
it fires `nas-sync.service` whenever `~/parsed_records.csv` actually
changes (i.e. right after `parse_sit.regenerate()` runs, every real capture
completion). This was a deliberate choice over computing a fixed
clock-time from the target TOW: the target TOW changes across calibration
epochs (manual re-entry, power-loss recalc), which would silently drift a
once-computed schedule — watching the file itself is correct for any TOW,
forever, with nothing to recompute.

`nas-sync.sh` retries the push 3 times (backoff) before giving up, then
sends a **non-urgent** alert email (same `ALERT_RECIPIENT=root`/msmtp
pattern as the rest of this manual) - failure log at
`~/nas-sync_mail-failures.log`.

Install/verify via `../ubx-data/reinstall.sh` (calls
`nas-sync/install-nas-sync.sh` as one of its stages) or run that script
directly. Same verify-only boundary as msmtp: the mount/credentials
themselves are restored by the user via
`~/staging/samba/install-samba-files.sh` (fill in the real password in
`~/staging/samba/etc/cifs-credentials-nas` first) - `install-nas-sync.sh`
never installs `cifs-utils` or writes `/etc/fstab`/credentials itself, it
only verifies the mount and installs the systemd units.

## Operations

- **Quick Go/No-go check**: `~/bin/sit-status.sh` (see
  [capture-status](../capture-status/)) checks both this repo's
  `SiT-calib` capture and the SiT5721 repo's register-save loop in one shot.
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

## Power-loss handling

When the SiT5721 actually loses power (not just an OS reboot), the chip
resets and `restart-SiT5721.py` (SiT5721 repo) detects this, recalculates
an aging-corrected Pull Value, reloads it, verifies the write, and leaves
`~/SiT-power-loss-mark.json`. `restart-calib.sh` picks that up (only if
its timestamp postdates this boot - a stale unconsumed mark from an
earlier, incompletely-processed boot is ignored and moved aside instead):

1. Archives `~/SiT-calib_output.txt`, `~/parsed_records.json`, and
   `~/parsed_records.csv` to `~/SiT-calib_archive/` as
   `SiT-calib_output_<timestamp>.txt`/`parsed_records_<timestamp>.json`/
   `.csv` (timestamp = UTC `detected_at` from the mark file, to the
   second, e.g. `2026-07-04T174234`) - extension stays last so file-type
   tools/editors still recognize them.
2. Writes a fresh `~/SiT-calib_output.txt` with a leading `#`-comment
   summarizing the event (harmless to `parse_sit.py` - see the Cowork
   note in that file).
3. Starts the `SiT-calib` screen as usual (capture auto-resumes - no
   manual step needed).
4. Emails a normal-priority notice instead of the plain reboot
   confirmation, so the power loss and the new calibration epoch aren't
   silent.

`parse_sit.py` needs no code changes for this: since
`parsed_records.json`/`.csv` are archived away too, `regenerate()`'s
record-count regression guard sees a fresh (nonexistent) file and simply
starts counting from zero - no `--force` needed. See project memory
`power-loss-mark-todo` for the full design rationale.

**Verified live 2026-07-06/07**: the Trixie reflash genuinely power-cycled
the SiT5721, and this exact chain ran for real - `~/SiT-calib_archive/`
holds the matching `SiT-power-loss-mark_2026-07-07T021006.json`,
`SiT-calib_output_2026-07-07T021006.txt`, and `parsed_records_*` archive
set, and the new `~/SiT-calib_output.txt` opens with the expected
recalibration header. See SiT5721's `MANUAL.md` "Known issues" for the
full writeup (that event's confirmation email failed to send, unrelated
first-boot msmtprc timing, since fixed - not a bug in this logic).

## Known issues / troubleshooting log

**2026-07-06/07 — capture chain down after the Trixie flash; initial
"no TOW given" theory was wrong.** `restart-calib.service` failed at
boot (`SiT-calib screen failed to start!`) during the Bookworm→Trixie
migration. First hypothesis (from the user, and initially accepted
here) was that no TOW had been supplied yet. That doesn't hold up:
`~/SiT-calib_state.json` was still well within its 24h freshness window
at that boot, so `start-get-data.sh` would have auto-resumed from it
without ever reaching the interactive `read -p "Enter TOW:"` prompt —
and `~/SiT-calib_mail-failures.log` has zero entries from around that
boot, which is where the urgent pre-prompt alert (see
`restart-calib-manual-tow` in project memory) would have logged a
retry/failure. So the interactive-prompt path was never hit at all.

Actual root cause: **`i2c-dev` wasn't loaded yet** at that boot (the
prerequisite above wasn't satisfied until `raspi-config`'s I2C toggle was
run later that same session) — `get-data.py`'s `bus = smbus.SMBus(0)`
call fails immediately with no `/dev/i2c-0` to open, killing the screen
near-instantly, well before any TOW logic or mail-sending code runs.
This was the same underlying cause as `reinstall.sh` separately reporting
`FAIL SiT5721 not responding on I2C bus 0 (0x60)` that same session — one
root cause, two symptoms. Resolved once `i2c-dev` was persisted via
`raspi-config`; should not recur on future boots since it's now in
`/etc/modules`.

Recovery used at the time: `screen -X -S SiT-calib quit` to clear the
dead session, then `./set-calib-screen.sh <TOW>` with a freshly
hand-computed TOW (see `gps-tow-to-utc` in project memory for the
reverse direction — converting a target wall-clock time *to* a TOW is
the same GPS-epoch/leap-second math run forwards).

**Same session — `SCRIPT_DIR` broke when invoked via its `~/bin/`
symlink.** `start-get-data.sh`, `set-calib-screen.sh`, and
`restart-calib.sh` all derived their own directory with
`dirname -- "$0"`, which doesn't resolve symlinks. Running
`~/bin/start-get-data.sh` directly (rather than the real path) made
`SCRIPT_DIR` resolve to `~/bin` instead of `mbt-ubx-apps/`, so it went
looking for `~/bin/../env/bin/activate` and `~/bin/get-data.py` — neither
exists there. `set-calib-screen.sh`/`restart-calib.sh` had the identical
fragile idiom but hadn't actually broken yet, only because every sibling
file they reference happens to also have a matching `~/bin` symlink (by
coincidence, not by design). `restart-calib.service` itself was never
exposed to this — its `ExecStart=` uses the real absolute path, not a
symlink. Fixed in all three (plus SiT5721's `restart-SiT5721-pull.sh`,
same idiom, same coincidental luck) by resolving `$0` through
`readlink -f` before taking `dirname` of it. Commit `49c1f28` (this
repo), `67c225f` (SiT5721).

## Key files/paths

| Path | Purpose |
|---|---|
| `~/SiT-calib_state.json` | Persisted `TOW_selected`/`week`/`interval`/`saved_at`, used to auto-resume across restarts |
| `~/SiT-calib_output.txt` | Append-only capture history (input to `parse_sit.py`) |
| `~/parsed_records.json`, `~/parsed_records.csv` | Auto-regenerated by `get-data.py` after each daily record - Excel workbook input |
| `~/parsed_records_errors.log` | Append-only log of `parse_sit.regenerate()` failures (capture itself is unaffected) |
| `~/SiT-calib_archive/` | Pre-power-loss `SiT-calib_output_*.txt`/`parsed_records_*.json`/`.csv`/mark file, archived by `restart-calib.sh` |
| `~/SiT-power-loss-mark.json` | Written by SiT5721's `restart-SiT5721.py`, consumed (renamed away) by `restart-calib.sh` |
| `lib/mbt-SiT5721-lib/` | Git submodule (shared with SiT5721) - `SiT5721` I2C class |
| `~/SiT-calib_mail-failures.log` | `send_urgent_mail()` retry/failure log (from `start-get-data.sh`) |
| `~/restart-calib_mail-failures.log` | Retry/failure log for `restart-calib.sh`'s own mail sends |
| `../env-setup.sh` | (Re)creates the shared venv (`../env/`) - stdlib `python3 -m venv --system-site-packages`, PEP-668-safe |
| `../reinstall.sh` | Whole-device provisioning/health check (OS packages, I2C/serial, venv, repos, systemd, mail) - see its own header |
| `../nas-sync/` | Pushes calibration files to the Synology NAS, triggered by `nas-sync.path` on `parsed_records.csv` changes - see [NAS sync](#5-nas-sync-calibration-files--synology) above |
