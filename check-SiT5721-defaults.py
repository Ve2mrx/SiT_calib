#!/bin/python3
"""
Report whether the SiT5721's config registers are at hardware defaults,
which only happens when the chip itself has lost power (its calibration
is volatile). Distinguishes a real power failure from a simple OS reboot,
where the chip stays powered and keeps its pull_value/aging_compensation.

Exit 0: registers are at defaults (chip was reset, calibration lost).
Exit 1: registers are not at defaults (calibration retained).
"""

import os
import sys

# mbt_SiT5721_lib lives in the lib/mbt-SiT5721-lib submodule, shared with SiT5721
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib", "mbt-SiT5721-lib"))

import smbus

from mbt_SiT5721_lib import SiT5721

# From the SiT5721 datasheet - register values after a power-on reset
DEFAULTS = {
    "pull_value": 0.0,
    "pull_range": 1e-05,
    "aging_compensation": 0.0,
    "max_freq_ramp_rate": 1e-05,
}
TOLERANCE = 1e-9


def is_default(siTime: SiT5721) -> bool:
    """True when EVERY config register still reads its power-on default.

    !! 2026-09-07: this test now effectively rests on `pull_value` ALONE, and
    that is a deliberate note, not a bug report. Of the four fields:
      * pull_range and max_freq_ramp_rate have always been left at their
        1.0E-05 defaults in normal operation - they have never discriminated;
      * aging_compensation was written to **0** on 2026-09-07 and is intended
        to stay there permanently (the drift it was compensating turned out to
        be a wander, not a constant - see TODO.md A4), so it now matches the
        default during healthy operation too.
    So only pull_value (currently ~1.26E-07 part = 0.126 ppm) separates "chip
    lost power" from "chip is fine".

    Why it is still CORRECT: a power-on reset zeroes *all four* registers at
    once, so the all() still fires exactly when it should. What is gone is the
    margin - a single field is now doing the whole job.

    The failure mode to watch is the INVERSE one: if a future re-tune ever
    computed a pull_value within TOLERANCE of 0, this would report a power
    failure that did not happen, and restart-calib.sh (which treats exit 0 as
    "power failure detected") would refuse to restart the capture. That is
    remote today - the pull has sat near 0.126 ppm for 16 months - but the
    robust fix is to test `uptime_uint` instead: it is reset by a power cycle
    and by nothing else, and it is already read and printed below. Tracked in
    TODO.md A1.
    """
    return all(
        abs(getattr(siTime, field) - value) < TOLERANCE
        for field, value in DEFAULTS.items()
    )


if __name__ == "__main__":
    bus = smbus.SMBus(0)
    address = 0x60
    siTime = SiT5721(bus, address)

    print(f"Uptime (time since SiT5721 power-up): {siTime.uptime_uint}s")

    if is_default(siTime):
        print("SiT5721 config registers are at hardware defaults: "
              "the chip lost power and its calibration was reset.")
        sys.exit(0)
    else:
        siTime.print_SiT_short()
        print("SiT5721 config registers retain prior calibration: "
              "looks like a normal restart.")
        sys.exit(1)
