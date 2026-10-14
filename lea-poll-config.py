#!/usr/bin/env python3
"""
lea-poll-config.py - READ-ONLY poll of the LEA-M8F configuration.

Written 2026-10-13 to find out why Timebeat logs "PPS Watchdog Timer Expired"
every 10 s on PPS/Client/eth0/0/0: the 2024-02-18 config dump had TIMEPULSE2
(CFG-TP5 tpIdx=1) at a pulse length of 0 us, which would never produce an edge.
That dump is two years old, so this script asks the receiver what it holds NOW.

It sends POLL requests only (an empty or tpIdx/portID-only payload asks the
receiver to report a setting; it never changes one). Every message is checked
to be a POLL before it is written, and nothing here can build a SET or a
CFG-CFG save, so the receiver's configuration cannot be altered by this script.

The serial port is shared with get-data.py, so the SiT-calib capture must be
stopped first; the script refuses to run while anything holds the port.
Stop the capture well away from the daily 03:06:22Z capture (midday is fine):

    screen -X -S SiT-calib quit
    cd ~/project/ubx-data/mbt-ubx-apps && ../env/bin/python lea-poll-config.py
    ./set-calib-screen.sh        # resumes from ~/SiT-calib_state.json

Output: printed, and saved to ~/lea-config-poll-<UTC stamp>.txt (decoded
messages plus raw hex), so the file can be copied to the PC for analysis.
The report includes CFG-TMODE2 = the surveyed antenna position: never commit it.

First run 2026-10-14 00:57Z confirmed the 0 us TIMEPULSE2 pulse live; that was
fixed the same night. To read the LEA WITHOUT stopping the capture, use
ublox-config-backup's `ubxcfg --i2c-bus 5` (>= 0.5.2) instead.
"""

import os
import subprocess
import sys
from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from datetime import datetime, timezone
from time import monotonic

from serial import Serial

from pyubx2 import (
    NMEA_PROTOCOL,
    POLL,
    UBX_PROTOCOL,
    UBXMessage,
    UBXReader,
)

# (class, message, payload keywords). Order: identity first, then the
# pulse outputs (the reason this exists), then the sync manager and the rest.
POLLS = [
    ("MON", "MON-VER", {}),
    ("CFG", "CFG-TP5", {"tpIdx": 0}),   # TIMEPULSE
    ("CFG", "CFG-TP5", {"tpIdx": 1}),   # TIMEPULSE2 - the PPS that should reach SYNC_OUT
    ("CFG", "CFG-SMGR", {}),
    ("CFG", "CFG-ESRC", {}),            # the SiT5721 on EXTINT0
    ("CFG", "CFG-DOSC", {}),
    ("CFG", "CFG-TMODE2", {}),
    ("CFG", "CFG-GNSS", {}),
    ("CFG", "CFG-SBAS", {}),
    ("CFG", "CFG-NAV5", {}),
    ("CFG", "CFG-ANT", {}),
    ("CFG", "CFG-PRT", {"portID": 1}),  # UART1
    ("MON", "MON-SMGR", {}),
    ("TIM", "TIM-TOS", {}),
]


def port_holders(port: str) -> tuple[list[str], int]:
    """Processes with `port` open, found via /proc/*/fd. Returns (holders, unreadable)."""
    target = os.path.realpath(port)
    holders, unreadable = [], 0
    for pid in filter(str.isdigit, os.listdir("/proc")):
        fd_dir = f"/proc/{pid}/fd"
        try:
            fds = os.listdir(fd_dir)
        except PermissionError:
            unreadable += 1
            continue
        except FileNotFoundError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(os.readlink(f"{fd_dir}/{fd}")) == target:
                    try:
                        with open(f"/proc/{pid}/comm") as f:
                            comm = f.read().strip()
                    except OSError:
                        comm = "?"
                    holders.append(f"{pid} ({comm})")
                    break
            except OSError:
                continue
    return holders, unreadable


def capture_screen_running() -> bool:
    try:
        out = subprocess.run(["screen", "-list"], capture_output=True, text=True).stdout
    except FileNotFoundError:
        return False
    return any(".SiT-calib" in line for line in out.splitlines())


def matches(msg: UBXMessage, kw: dict, parsed) -> bool:
    if parsed is None or parsed.identity != msg.identity:
        return False
    return all(getattr(parsed, k, None) == v for k, v in kw.items())


def is_nak_for(msg: UBXMessage, parsed) -> bool:
    return (
        parsed is not None
        and parsed.identity == "ACK-NAK"
        and parsed.clsID == msg.msg_cls[0]
        and parsed.msgID == msg.msg_id[0]
    )


def poll_one(stream, ubr, msg, kw, timeout):
    """Send one POLL, return ("ok", raw, parsed) / ("nak", raw, parsed) / ("timeout", None, None)."""
    assert msg.msgmode == POLL, f"refusing to send non-POLL message {msg}"
    for _attempt in range(2):
        stream.reset_input_buffer()
        stream.write(msg.serialize())
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            raw, parsed = ubr.read()
            if raw is None:
                continue
            if matches(msg, kw, parsed):
                return "ok", raw, parsed
            if is_nak_for(msg, parsed):
                return "nak", raw, parsed
    return "timeout", None, None


def tp5_verdict(p) -> str:
    """One-line reading of a CFG-TP5 reply, aimed at the zero-width-pulse question."""
    name = {0: "TIMEPULSE", 1: "TIMEPULSE2"}.get(p.tpIdx, f"tpIdx {p.tpIdx}")
    if not p.active:
        return f"{name}: INACTIVE (active=0) - no pulse output at all"
    unit_len = "us" if p.isLength else "ratio (2^-32)"
    unit_per = "Hz" if p.isFreq else "us"
    s = (f"{name}: active, period/freq locked={p.freqPeriodLock} {unit_per} "
         f"(unlocked {p.freqPeriod}), pulse locked={p.pulseLenRatioLock} {unit_len} "
         f"(unlocked {p.pulseLenRatio}), lockedOtherSet={p.lockedOtherSet}, "
         f"polarity={'rising' if p.polarity else 'falling'}, alignToTow={p.alignToTow}")
    if p.lockedOtherSet and p.pulseLenRatioLock == 0:
        s += "\n    *** ZERO-WIDTH PULSE ONCE LOCKED - this output produces no edge ***"
    if p.pulseLenRatio == 0:
        s += "\n    *** zero-width pulse while NOT locked ***"
    return s


def main() -> int:
    arp = ArgumentParser(formatter_class=ArgumentDefaultsHelpFormatter, description=__doc__.split("\n")[1])
    arp.add_argument("-P", "--port", default="/dev/ttyS0", help="serial port (as start-get-data.sh)")
    arp.add_argument("-B", "--baudrate", default=19200, type=int, help="baud rate (as get-data.py)")
    arp.add_argument("-T", "--timeout", default=3.0, type=float, help="seconds to wait per reply")
    arp.add_argument("-O", "--outdir", default=os.path.expanduser("~"), help="where to save the report")
    args = arp.parse_args()

    if capture_screen_running():
        print("STOP: the SiT-calib screen session is running. Stop it first:\n"
              "    screen -X -S SiT-calib quit", file=sys.stderr)
        return 2
    holders, unreadable = port_holders(args.port)
    if holders:
        print(f"STOP: {args.port} is open by: {', '.join(holders)}", file=sys.stderr)
        return 2
    if unreadable:
        print(f"note: {unreadable} processes not inspectable (other users); "
              f"none of them is the capture, which runs as you.", file=sys.stderr)

    msgs = [(UBXMessage(c, m, POLL, **kw), kw) for c, m, kw in POLLS]
    for msg, _ in msgs:  # all checked before the first byte is written
        assert msg.msgmode == POLL, msg

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    out_path = os.path.join(args.outdir, f"lea-config-poll-{stamp}.txt")
    lines = [f"lea-poll-config.py  {stamp}  port={args.port} baud={args.baudrate}", ""]
    verdicts = []

    with Serial(args.port, args.baudrate, timeout=0.2) as stream:
        ubr = UBXReader(stream, protfilter=NMEA_PROTOCOL | UBX_PROTOCOL, quitonerror=0)
        for msg, kw in msgs:
            label = msg.identity + (f" {kw}" if kw else "")
            status, raw, parsed = poll_one(stream, ubr, msg, kw, args.timeout)
            if status == "ok":
                lines += [f"## {label}", str(parsed), f"raw {raw.hex()}", ""]
                if parsed.identity == "CFG-TP5":
                    verdicts.append(tp5_verdict(parsed))
            else:
                lines += [f"## {label}", f"-- {status.upper()} (no reply of that type)", ""]
            print(f"{label:28s} {status}")

    if verdicts:
        lines += ["## SUMMARY - pulse outputs"] + verdicts
    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")

    print()
    print("\n".join(verdicts) if verdicts else "No CFG-TP5 reply - pulse settings unknown.")
    print(f"\nSaved: {out_path}")
    print("Now restart the capture:  ./set-calib-screen.sh")
    return 0


if __name__ == "__main__":
    sys.exit(main())
