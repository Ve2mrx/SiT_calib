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
