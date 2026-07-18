"""
SiT5721 GPSDO calibration capture tool.

Reads u-blox TIM-TOS/TIM-SMEAS/PUBX04/NAV-PVT messages from a GNSS
receiver and pull_value/aging_compensation/uptime/status from a SiT5721
connected over I2C. Given a target GPS TOW (-W), waits for it and then
snapshots both to screen and, if -O is given, to file; with -i, repeats
every `interval` seconds after that, looping across GPS week boundaries.
The TOW being waited for can be persisted/resumed across restarts with
-S, so a reboot doesn't need it re-entered by hand.

The file snapshot also includes GNSS-reference-quality fields (TIM-SMEAS
phase/freq uncertainty, TIM-TOS GNSS time uncertainty, NAV-PVT SV count -
the latter best-effort) as a versioned CSV,<version>,... record, so
degraded captures (e.g. ionospheric scintillation) can be flagged from the
reference side - the SiT status alone only reflects the oscillator side.
See build_csv_line() and parse_sit.py's module docstring.

Run with -h for the full list of command-line options.

Originally forked from pygnssutils' gnssapp.py skeleton (GNSSSkeletonApp);
heavily modified by Martin Boissonneault to add the SiT5721 reads and the
capture/scheduling logic above.

Requires:
https://github.com/semuconsulting/pyubx2/tree/master
https://github.com/semuconsulting/pynmeagps
https://github.com/pyserial/pyserial
apt python3-smbus
mbt_SiT5721_lib (SiT5721 over I2C at address 0x60)

Created on 2024 Dec 07

:author: semuadmin / Martin Boissonneault VE2MRX
:copyright: SEMU Consulting © 2023 / Martin Boissonneault VE2MRX © 2024
:license: BSD 3-Clause
"""

# pylint: disable=invalid-name, too-many-instance-attributes

import os
import sys

# mbt_SiT5721_lib lives in the lib/mbt-SiT5721-lib submodule, shared with SiT5721
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib", "mbt-SiT5721-lib"))


def _read_version() -> str:
    """Reads the sibling VERSION file (repo root); "unknown" if missing."""
    version_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "VERSION")
    try:
        with open(version_path) as f:
            return f.read().strip()
    except OSError:
        return "unknown"


__version__ = _read_version()

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser, ArgumentTypeError
from queue import Empty, Queue
from threading import Event, Lock, Thread
from time import monotonic, sleep

from pynmeagps import NMEAMessageError, NMEAParseError
from pyrtcm import RTCMMessage, RTCMMessageError, RTCMParseError
from serial import Serial

from pyubx2 import (
    NMEA_PROTOCOL,
    UBX_PROTOCOL,
    SET,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
    gnss2str,
    UTCSTANDARD
)

from mbt_SiT5721_lib import SiT5721
import smbus
import parse_sit

from datetime import datetime, timedelta, time, date, timezone
import json

CONNECTED = 1

GPSWEEK_SECONDS = (7 * 24 * 60 * 60)  # 7 days * 24h * 60m * 60s

# (msgClass, msgID) for every message this capture loop depends on. Used by
# the observe-then-fix check in __main__: a message not seen within
# MESSAGE_GRACE_PERIOD_S of startup gets an explicit CFG-MSG enable (legacy
# mechanism - this is an M8-class (LEA-M8F) receiver, which predates and
# doesn't support the CFG-VALSET interface enable_ubx() uses above).
REQUIRED_MESSAGES = {
    "TIM-TOS": (0x0D, 0x12),
    "TIM-SMEAS": (0x0D, 0x13),
    "PUBX04": (0xF1, 0x04),
    "NAV-PVT": (0x01, 0x07),
}
MESSAGE_GRACE_PERIOD_S = 15.0  # several nav-solution cycles

# NAV-PVT (SV count) is best-effort and not part of the strict data_valid
# gate - a value older than this is treated as unavailable rather than
# shown stale.
NAV_PVT_STALE_S = 5.0

# Bump when the CSV,<version>,... record's field list changes; add a new
# CSV_LINE_FIELDS_V<N> branch in parse_sit.py's parse_csv_line() to match.
# See that file's module docstring for the full versioning convention.
CSV_LINE_VERSION = 1


class GNSSSkeletonApp:
    """
    Skeleton GNSS application which communicates with a GNSS receiver.
    """

    def __init__(
        self, port: str, baudrate: int, timeout: float, stopevent: Event, **kwargs
    ):
        """
        Constructor.

        :param str port: serial port e.g. "/dev/ttyACM1"
        :param int baudrate: baudrate
        :param float timeout: serial timeout in seconds
        :param Event stopevent: stop event
        """

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.stopevent = stopevent
        self.sendqueue = kwargs.get("sendqueue", None)
        self.idonly = kwargs.get("idonly", True)
        self.filtered = kwargs.get("filtered", False)
        self.enableubx = kwargs.get("enableubx", False)
        self.showhacc = kwargs.get("showhacc", False)
        self.stream = None
        self.lat = 0
        self.lon = 0
        self.alt = 0
        self.sep = 0

    def __enter__(self):
        """
        Context manager enter routine.
        """

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        """
        Context manager exit routine.

        Terminates app in an orderly fashion.
        """

        self.stop()

    def run(self):
        """
        Run GNSS reader/writer.
        """

        self.enable_ubx(self.enableubx)

        self.stream = Serial(self.port, self.baudrate, timeout=self.timeout)
        self.stopevent.clear()

        read_thread = Thread(
            target=self._read_loop,
            args=(
                self.stream,
                self.stopevent,
                self.sendqueue,
            ),
            daemon=True,
        )
        read_thread.start()

    def stop(self):
        """
        Stop GNSS reader/writer.
        """

        self.stopevent.set()
        if self.stream is not None:
            self.stream.close()

    def _read_loop(self, stream: Serial, stopevent: Event, sendqueue: Queue):
        """
        THREADED
        Reads and parses incoming GNSS data from the receiver,
        and sends any queued output data to the receiver.

        :param Serial stream: serial stream
        :param Event stopevent: stop event
        :param Queue sendqueue: queue for messages to send to receiver
        """

        global data
        global siTime
        global data_lock

        ubr = UBXReader(stream, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL))
        while not stopevent.is_set():
            try:
                if stream.in_waiting:
                    _, parsed_data = ubr.read()
                    if parsed_data:
                        self._extract_coordinates(parsed_data)

                        if self.filtered:
                            with data_lock:
                                if parsed_data.identity == "TIM-TOS":
                                    data = reset_data_valid(data)
                                    data = invalidate_SiT_data(data)

                                    data = get_ubx_TIM_TOS_data(
                                        parsed_data, data)

                                    data = get_SiT_data(siTime, data)

                                    nty = f", week={data['TIM-TOS.week']}, TOW={data['TIM-TOS.TOW']}, gnssId={data['TIM-TOS.gnssId']}"

                                elif parsed_data.identity == "TIM-SMEAS":
                                    data = get_ubx_TIM_SMEAS_data(
                                        parsed_data, data)

                                    nty = f", iTOW={data['TIM-SMEAS.iTOW']}, source={data['TIM-SMEAS.source']}, freqValid={data['TIM-SMEAS.freqValid']}, phaseValid={data['TIM-SMEAS.phaseValid']}, PhaseOffset={data['TIM-SMEAS.phaseOffset']}, PhaseUnc={data['TIM-SMEAS.phaseUnc']}, freqOffset={data['TIM-SMEAS.freqOffset']}, freqUnc={data['TIM-SMEAS.freqUnc']}"

                                elif parsed_data.identity == "PUBX04":
                                    data = get_nmea_PUBX04(
                                        parsed_data, data)

                                    nty = f", utcWk={data['PUBX04.utcWk']}, utcTow={data['PUBX04.utcTow']}, leapSec={data['PUBX04.leapSec']}"

                                elif parsed_data.identity == "NAV-PVT":
                                    data = get_ubx_NAV_PVT_data(
                                        parsed_data, data)

                                    nty = f", numSV={data['NAV-PVT.numSV']}"
                                else:
                                    continue

                                if parsed_data.identity in REQUIRED_MESSAGES:
                                    data[f"_seen.{parsed_data.identity}"] = monotonic()

                            print(f"GNSS>> {parsed_data.identity}{nty}")

                        elif self.idonly:
                            print(f"GNSS>> {parsed_data.identity}")

                        else:
                            print(parsed_data)

                else:
                    sleep(0.01)

                self._send_data(ubr.datastream, sendqueue)

            except (
                UBXMessageError,
                UBXParseError,
                NMEAMessageError,
                NMEAParseError,
                RTCMMessageError,
                RTCMParseError,
            ) as err:
                print(f"Error parsing data stream {err}")
                continue

            except Exception as err:
                print(f"Fatal error in GNSS read loop, stopping reader: {err}")
                stopevent.set()
                break

    def _extract_coordinates(self, parsed_data: object):
        """
        Extract current navigation solution from NMEA or UBX message.

        :param object parsed_data: parsed NMEA or UBX navigation message
        """

        if hasattr(parsed_data, "lat"):
            self.lat = parsed_data.lat
        if hasattr(parsed_data, "lon"):
            self.lon = parsed_data.lon
        if hasattr(parsed_data, "alt"):
            self.alt = parsed_data.alt
        if hasattr(parsed_data, "hMSL"):  # UBX hMSL is in mm
            self.alt = parsed_data.hMSL / 1000
        if hasattr(parsed_data, "sep"):
            self.sep = parsed_data.sep
        if hasattr(parsed_data, "hMSL") and hasattr(parsed_data, "height"):
            self.sep = (parsed_data.height - parsed_data.hMSL) / 1000
        if self.showhacc and hasattr(parsed_data, "hAcc"):  # UBX hAcc is in mm
            unit = 1 if parsed_data.identity == "PUBX00" else 1000
            print(
                f"Estimated horizontal accuracy: {(parsed_data.hAcc / unit):.3f} m")

    def _send_data(self, stream: Serial, sendqueue: Queue):
        """
        Send any queued output data to receiver.
        Queue data is tuple of (raw_data, parsed_data).

        :param Serial stream: serial stream
        :param Queue sendqueue: queue for messages to send to receiver
        """

        if sendqueue is not None:
            try:
                while not sendqueue.empty():
                    data = sendqueue.get(False)
                    raw, parsed = data
                    source = "NTRIP>>" if isinstance(
                        parsed, RTCMMessage) else "GNSS<<"
                    if self.idonly:
                        print(f"{source} {parsed.identity}")
                    else:
                        print(parsed)
                    stream.write(raw)
                    sendqueue.task_done()
            except Empty:
                pass

    def enable_ubx(self, enable: bool):
        """
        Enable UBX output and suppress NMEA.

        :param bool enable: enable UBX and suppress NMEA output
        """

        layers = 1
        transaction = 0
        cfg_data = []
        for port_type in ("USB", "UART1"):
            cfg_data.append((f"CFG_{port_type}OUTPROT_NMEA", not enable))
            cfg_data.append((f"CFG_{port_type}OUTPROT_UBX", enable))
            cfg_data.append((f"CFG_MSGOUT_UBX_NAV_PVT_{port_type}", enable))
            cfg_data.append(
                (f"CFG_MSGOUT_UBX_NAV_SAT_{port_type}", enable * 4))
            cfg_data.append(
                (f"CFG_MSGOUT_UBX_NAV_DOP_{port_type}", enable * 4))
            cfg_data.append((f"CFG_MSGOUT_UBX_RXM_RTCM_{port_type}", enable))

        msg = UBXMessage.config_set(layers, transaction, cfg_data)
        self.sendqueue.put((msg.serialize(), msg))

    def enable_message(self, msgClass: int, msgID: int, rate: int = 1):
        """
        Request output of a single UBX/NMEA message via the legacy CFG-MSG
        mechanism - works on this M8-class (LEA-M8F) receiver, unlike
        CFG-VALSET (see enable_ubx() above, which this receiver appears to
        silently ignore). Idempotent: setting an already-enabled message's
        rate to the same value is a no-op on the receiver. RAM-layer only
        (no CFG-CFG save) - re-sent from scratch on every process start,
        which is by design: see the observe-then-fix check in __main__.

        :param int msgClass: UBX message class, or 0xF1 for NMEA-proprietary
        :param int msgID: UBX message ID
        :param int rate: output rate in messages per navigation solution
        """

        msg = UBXMessage(
            "CFG", "CFG-MSG", SET,
            msgClass=msgClass, msgID=msgID,
            rateUART1=rate, rateUSB=rate,
        )
        self.sendqueue.put((msg.serialize(), msg))

    def get_coordinates(self) -> tuple:
        """
        Return current receiver navigation solution.
        (method needed by certain pygnssutils classes)

        :return: tuple of (connection status, lat, lon, alt and sep)
        :rtype: tuple
        """

        return (CONNECTED, self.lat, self.lon, self.alt, self.sep)

    def set_event(self, eventtype: str):
        """
        Create event.
        (stub method needed by certain pygnssutils classes)

        :param str eventtype: name of event to create
        """


def invalidate_SiT_data(data: dict):
    """
    Get SiT5721 data from SiTdev

    :param object SiT5721 SiTdev: Device to extract data from
    :param dict data: name of dict to store the extracted data

    :return dict: Modified data dict
    """

    data['SiT.data_valid'] = False

    data['SiT.pull_value'] = None
    data['SiT.pull_range'] = None
    data['SiT.aging_compensation'] = None
    data['SiT.max_freq_ramp_rate'] = None
    data['SiT.uptime'] = None
    data['SiT.total_offset_written'] = None
    data['SiT.error_status_flag'] = None
    data['SiT.stability_flag'] = None

    return data


def get_SiT_data(SiTdev: object, data: dict):
    """
    Get SiT5721 data from SiTdev

    :param object SiT5721 SiTdev: Device to extract data from
    :param dict data: name of dict to store the extracted data

    :return dict data: Modified data dict
    """

    SiTdev.read_SiT_config()
    SiTdev.read_SiT_dynamic()

    data['SiT.pull_value'] = float(SiTdev.pull_value)
    data['SiT.pull_range'] = float(SiTdev.pull_range)
    data['SiT.aging_compensation'] = float(SiTdev.aging_compensation)
    data['SiT.max_freq_ramp_rate'] = float(SiTdev.max_freq_ramp_rate)
    data['SiT.uptime'] = int(SiTdev.uptime_uint)
    data['SiT.total_offset_written'] = float(SiTdev.total_offset_written)
    data['SiT.error_status_flag'] = int(SiTdev.error_status_flag_uint)
    data['SiT.stability_flag'] = int(SiTdev.stability_flag_uint)

    data['SiT.data_valid'] = True

    return data


def get_ubx_TIM_TOS_data(parsed_data: object, data: dict):
    """
    Get uBlox TIM-TOS data from GNSS

    :param object UBXReader parsed_data: Data to fetch values from
    :param dict data: name of dict to store the extracted data

    :return dict: Modified data dict
    """

    data['TIM-TOS.gnssId'] = int(parsed_data.gnssId)
    data['TIM-TOS.gnssId.str'] = str(gnss2str(parsed_data.gnssId))
    data['TIM-TOS.gnssTimeValid'] = bool(parsed_data.gnssTimeValid)
    data['TIM-TOS.UTCTimeValid'] = bool(parsed_data.UTCTimeValid)
    data['TIM-TOS.year'] = int(parsed_data.year)
    data['TIM-TOS.month'] = int(parsed_data.month)
    data['TIM-TOS.day'] = int(parsed_data.day)
    data['TIM-TOS.hour'] = int(parsed_data.hour)
    data['TIM-TOS.minute'] = int(parsed_data.minute)
    data['TIM-TOS.second'] = int(parsed_data.second)
    data['TIM-TOS.utcStandard'] = int(parsed_data.utcStandard)

    data['TIM-TOS.utcStandard.str'] = utcStdToStr(
        int(parsed_data.utcStandard))

    data['TIM-TOS.week'] = int(parsed_data.week)
    data['TIM-TOS.TOW'] = int(parsed_data.TOW)

    data['TIM-TOS.utc.date'] = date(
        parsed_data.year,
        parsed_data.month,
        parsed_data.day
    )
    data['TIM-TOS.utc.time'] = time(
        parsed_data.hour,
        parsed_data.minute,
        parsed_data.second,
        0,
        tzinfo=timezone.utc
    )

    # GNSS-reference-side time uncertainty (ns) - unlike the SiT status,
    # this reflects the *reference*'s quality, not the oscillator's. Always
    # present when this function runs (TIM-TOS is strictly gated).
    data['TIM-TOS.gnssUncertainty'] = int(parsed_data.gnssUncertainty)

    data['TIM_TOS.data_valid'] = True

    return data


def get_ubx_TIM_SMEAS_data(parsed_data: object, data: dict):
    """
    Get uBlox TIM-SMEAS data from GNSS

    :param object UBXReader parsed_data: data to fetch values from
    :param dict data: name of dict to store the extracted data

    :return dict: Modified data dict
    """

    if parsed_data.sourceId_03 == 2:  # sourceId_03 == 2 means source is EXTINT0
        phaseOffset_03_float = (
            parsed_data.phaseOffset_03
            + parsed_data.phaseOffsetFrac_03
        )
        phaseUnc_03_float = (
            parsed_data.phaseUnc_03
            + parsed_data.phaseUncFrac_03
        )

        data['TIM-SMEAS.iTOW'] = int(parsed_data.iTOW)
        data['TIM-SMEAS.source'] = str("EXTINT0")
        data['TIM-SMEAS.freqValid'] = bool(parsed_data.freqValid_03)
        data['TIM-SMEAS.phaseValid'] = bool(parsed_data.phaseValid_03)
        data['TIM-SMEAS.phaseOffset'] = float(phaseOffset_03_float)
        data['TIM-SMEAS.phaseUnc'] = float(phaseUnc_03_float)
        data['TIM-SMEAS.freqOffset'] = float(parsed_data.freqOffset_03)
        data['TIM-SMEAS.freqUnc'] = float(parsed_data.freqUnc_03)

        data['TIM-SMEAS.data_valid'] = True

    else:
        data['TIM-SMEAS.iTOW'] = int(parsed_data.iTOW)
        data['TIM-SMEAS.source'] = "other"
        data['TIM-SMEAS.freqValid'] = None
        data['TIM-SMEAS.phaseValid'] = None
        data['TIM-SMEAS.phaseOffset'] = None
        data['TIM-SMEAS.phaseUnc'] = None
        data['TIM-SMEAS.freqOffset'] = None
        data['TIM-SMEAS.freqUnc'] = None

        data['TIM-SMEAS.data_valid'] = False

    return data


def get_nmea_PUBX04(parsed_data: object, data: dict):
    """
    Get uBlox NMEA PUBX04 data from GNSS

    :param object UBXReader parsed_data: data to fetch values from
    :param dict data: name of dict to store the extracted data

    :return dict: Modified data dict
    """

    data['PUBX04.utcWk'] = int(parsed_data.utcWk)
    data['PUBX04.utcTow'] = float(parsed_data.utcTow)
    # On GNSS start, it is "16D", thus NOT int
    data['PUBX04.leapSec'] = str(parsed_data.leapSec)

    data['PUBX04.data_valid'] = True

    return data


def get_ubx_NAV_PVT_data(parsed_data: object, data: dict):
    """
    Get uBlox NAV-PVT SV count (best-effort reference-quality field).

    Unlike the other get_ubx_*/get_nmea_* functions, this is not gated by
    reset_data_valid()/the strict data_valid AND-check: NAV-PVT enablement
    on this receiver is a defensive fallback (see enable_message() and the
    observe-then-fix check in __main__), so capture must never stall
    waiting for it. Always overwrites with the latest value; nav_pvt_display()
    is what decides whether that value is still fresh enough to report.

    :param object UBXReader parsed_data: data to fetch values from
    :param dict data: name of dict to store the extracted data

    :return dict: Modified data dict
    """

    data['NAV-PVT.numSV'] = int(parsed_data.numSV)
    data['NAV-PVT.received_at'] = monotonic()

    return data


def nav_pvt_display(data: dict):
    """
    Returns the most recent NAV-PVT SV count, or None if it's never
    arrived or is older than NAV_PVT_STALE_S - so a best-effort field
    never reports a stale value as current.

    :param dict data: shared data dict

    :return int | None: SV count, or None if unavailable/stale
    """

    received_at = data.get('NAV-PVT.received_at')
    if received_at is None or (monotonic() - received_at) > NAV_PVT_STALE_S:
        return None
    return data['NAV-PVT.numSV']


def fmt_or_na(value, suffix: str = "") -> str:
    """
    Formats a value for display, or "N/A" if it's None (used for
    best-effort fields like NAV-PVT's SV count that may not have arrived).

    :param value: value to format, or None
    :param str suffix: optional unit suffix appended when value is present

    :return str: formatted value+suffix, or "N/A"
    """

    if value is None:
        return "N/A"
    return f"{value}{(' ' + suffix) if suffix else ''}"


def reset_data_valid(data: dict):
    """
    Get Reset data_valid from dict
    data_valid indicates if the message data exists and is up-to-date for this TOW.
    Resetting it is usually done when receiving a new TOW to invalidate the old data

    :param dict data: name of dict to store the modified data

    :return dict: Modified data dict
    """

    data['TIM_TOS.data_valid'] = False
    data['SiT.data_valid'] = False
    data['TIM-SMEAS.data_valid'] = False
    data['PUBX04.data_valid'] = False

    return data


def utcStdToStr(utcStandard: int) -> str:
    """
    Converts the numeric UTC Standard value to a string

    :param int utcStandard: numeric UTC Standard value

    :return str: string representing the UTC Standard
    """

    try:
        return UTCSTANDARD[utcStandard]
    except KeyError:
        return str(utcStandard)


def error_status(flag: int) -> str:
    """
    Converts the SiT5721 error status flag to a status string.

    :param int flag: SiT5721 error_status_flag register value

    :return str: "good" if the flag indicates no error, else "ERROR"
    """

    if flag == 7:
        return "good"
    return "ERROR"


def stability_status(flag: int) -> str:
    """
    Converts the SiT5721 stability flag to a status string.

    :param int flag: SiT5721 stability_flag register value

    :return str: "stabilized" if the flag indicates stability, else "unstabilized"
    """

    if flag == 1:
        return "stabilized"
    return "unstabilized"


def flag_valid(flag: bool) -> str:
    """
    Converts a bool flag to a "Valid"/"INVALID" status string.

    :param bool flag: flag to convert

    :return str: "Valid", "INVALID", or str(flag) if flag isn't a bool
    """

    FLAGVALUES = {
        False: "INVALID",
        True: "Valid",
    }
    try:
        return FLAGVALUES[flag]
    except KeyError:
        return str(flag)


def build_csv_line(data: dict) -> str:
    """
    Machine-parseable record embedded in SiT-calib_output.txt, versioned so
    parse_sit.py can evolve the field set without breaking entries already
    on disk (bump CSV_LINE_VERSION here and add a matching
    CSV_LINE_FIELDS_V<N> branch in parse_sit.py's parse_csv_line()).

    This is parse_sit.py's authoritative source going forward for every
    field it carries - the verbose block around it is for human reading
    only. See the deprecation note on the one-line summary below it.

    :param dict data: name of dict holding the captured data

    :return str: one CSV,<version>,... line (no trailing newline)
    """

    error_status_str = error_status(data['SiT.error_status_flag'])
    stability_status_str = stability_status(data['SiT.stability_flag'])
    sv_count = nav_pvt_display(data)

    fields = [
        str(data['TIM-TOS.utc.date']),
        # HH:MM:SS only, no UTC offset - matches UTC_RE's legacy extraction
        # (str() on a tz-aware time would append "+00:00", inconsistent
        # with historic rows in the same CSV column).
        data['TIM-TOS.utc.time'].strftime("%H:%M:%S"),
        data['TIM-TOS.week'],
        data['TIM-TOS.TOW'],
        repr(data['TIM-SMEAS.phaseOffset']),
        repr(data['SiT.total_offset_written'] / pow(10, -6)),
        repr(data['SiT.pull_value'] / pow(10, -6)),
        repr(data['SiT.aging_compensation']),
        flag_valid(data['TIM-SMEAS.freqValid']),
        flag_valid(data['TIM-SMEAS.phaseValid']),
        error_status_str,
        stability_status_str,
        repr(data['TIM-SMEAS.phaseUnc']),
        repr(data['TIM-SMEAS.freqUnc']),
        data['TIM-TOS.gnssUncertainty'],
        ("" if sv_count is None else sv_count),
    ]
    return f"CSV,{CSV_LINE_VERSION}," + ",".join(str(f) for f in fields)


def _kv(label: str, value) -> str:
    """One aligned 'label   value' line for the human-readable calib block."""

    return f"{label:<32}{value}"


def _format_calib_lines(data: dict) -> list:
    """
    Builds the human-readable per-cycle block (TIM-TOS through SiT totals)
    as a list of lines, shared by printToFile_calib_data() and
    printToScreen_calib_data() so file/screen output can't drift apart.
    Purely cosmetic - parse_sit.py no longer keys on any of this text (see
    build_csv_line() above), so this is free to be reformatted at will.

    :param dict data: name of dict holding the captured data

    :return list[str]: lines to print, in order (no trailing newlines)
    """

    error_status_str = error_status(data['SiT.error_status_flag'])
    stability_status_str = stability_status(data['SiT.stability_flag'])

    return [
        "-" * 60,
        _kv("TIM-TOS week/TOW/system",
            f"{data['TIM-TOS.week']}, {data['TIM-TOS.TOW']}, {data['TIM-TOS.gnssId.str']}"),
        _kv("TIM-TOS UTC",
            f"{data['TIM-TOS.utc.date']}, {data['TIM-TOS.utc.time']}, {data['TIM-TOS.utcStandard.str']}"),
        _kv("TIM-TOS GNSS time uncertainty", f"{data['TIM-TOS.gnssUncertainty']} ns"),
        "",
        _kv("TIM-SMEAS iTOW", f"{data['TIM-SMEAS.iTOW'] / 1000:.3f} s"),
        _kv("TIM-SMEAS source", data['TIM-SMEAS.source']),
        _kv("TIM-SMEAS flags",
            f"freq={flag_valid(data['TIM-SMEAS.freqValid'])} phase={flag_valid(data['TIM-SMEAS.phaseValid'])}"),
        _kv("TIM-SMEAS phase offset", f"{data['TIM-SMEAS.phaseOffset']:.3f} ns"),
        _kv("TIM-SMEAS phase uncertainty", f"{data['TIM-SMEAS.phaseUnc']:.3f} ns"),
        _kv("TIM-SMEAS freq offset", f"{data['TIM-SMEAS.freqOffset']:.3f} ps/s"),
        _kv("TIM-SMEAS freq uncertainty", f"{data['TIM-SMEAS.freqUnc']:.3f} ps/s"),
        "",
        _kv("PUBX04 UTC week/TOW", f"{data['PUBX04.utcWk']}, {data['PUBX04.utcTow']:.2f}"),
        _kv("PUBX04 leap sec", data['PUBX04.leapSec']),
        "",
        _kv("NAV-PVT SV count", fmt_or_na(nav_pvt_display(data))),
        "",
        _kv("SiT uptime", f"{data['SiT.uptime']} s ({timedelta(seconds=data['SiT.uptime'])})"),
        _kv("SiT status", f"{error_status_str}, {stability_status_str}"),
        _kv("SiT pull value", "{:+.8g} ppm".format(data['SiT.pull_value'] / pow(10, -6))),
        _kv("SiT pull range", "{:.8g} ppm".format(data['SiT.pull_range'] / pow(10, -6))),
        _kv("SiT aging compensation", "{:+.8g} part/s".format(data['SiT.aging_compensation'])),
        _kv("SiT max freq ramp rate", "{:.8g} ppm".format(data['SiT.max_freq_ramp_rate'] / pow(10, -6))),
        _kv("SiT total offset written", "{:+.8g} ppm".format(data['SiT.total_offset_written'] / pow(10, -6))),
        "-" * 60,
    ]


def printToFile_calib_data(data: dict, file: str, TOW_selected: int):
    """
    Print to file the results of the data collection

    :param dict data: name of dict to store the modified data
    :param str file: filename used for output
    :param int TOW_selected: TOW that was being waited for
    """

    error_status_str = error_status(
        data['SiT.error_status_flag'])

    stability_status_str = stability_status(
        data['SiT.stability_flag'])

    with open(file, "a") as f:
        print(
            f"...Waiting for TOW={TOW_selected:6d}, we're at {data['TIM-TOS.TOW']:6d}", file=f)
        for line in _format_calib_lines(data):
            print(line, file=f)
        print(file=f)

        print(build_csv_line(data), file=f)

        # DEPRECATED, frozen: kept only for block-boundary detection
        # (parse_sit.py's SUMMARY_RE) and backward compatibility with
        # entries that predate the CSV,<version>,... line above - no new
        # fields are ever added to this line again. Once there's enough
        # production history on the CSV-line format, plan is to drop this
        # print entirely and switch parse_sit.py's block-boundary
        # detection to the CSV line itself - see that file's module
        # docstring for the full removal plan.
        print(f"{data['TIM-TOS.week']:4d}, {data['TIM-TOS.TOW']:6d}, {data['TIM-SMEAS.phaseOffset']:=12.3f}, {data['SiT.total_offset_written'] / pow(10, -6):=+3.10g}, flags(freq: {flag_valid(data['TIM-SMEAS.freqValid'])}, phase: {flag_valid(data['TIM-SMEAS.phaseValid'])}), SiT status({error_status_str}, {stability_status_str})", file=f)
        print(file=f)


def printToScreen_calib_data(data: dict):
    """
    Print to screen the results of the data collection

    :param dict data: name of dict to store the modified data
    """

    for line in _format_calib_lines(data):
        print(line)


def calcNextTOW(tow: int, interval: int):
    """
    Calculates the next TOW (using TIM-TOS)

    :param int tow: current TOW
    :param int interval: interval in second between TOW

    :return tuple[int, int]: (next TOW, number of GPS weeks crossed to get
        there - 0 normally, 1 when tow + interval wraps past the end of the
        GPS week; the caller must add this to the current week number, or
        the saved (TOW, week) pair will silently mismatch)
    """

    total = tow + interval
    return total % GPSWEEK_SECONDS, total // GPSWEEK_SECONDS


def save_tow_state(statefile: str, tow_selected: int, interval, week: int) -> None:
    """
    Persist the current TOW_selected so a restart can resume without
    re-entering it. Written atomically so a power loss mid-write can't
    corrupt the state file.

    :param str statefile: path to the state file
    :param int tow_selected: TOW currently being waited for
    :param interval: capture interval in effect (int or False)
    :param int week: GPS week number the TOW belongs to
    """

    tmp = f"{statefile}.tmp"
    with open(tmp, "w") as f:
        json.dump({
            "TOW_selected": tow_selected,
            "week": week,
            "interval": interval,
            "saved_at": datetime.now(timezone.utc).isoformat(),
        }, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, statefile)


def load_tow_state(statefile: str):
    """
    Load a previously saved TOW_selected, ignoring it if it's older than
    the interval that was in effect when it was saved (i.e. at least one
    capture cycle has already been missed, so the -i cadence can no longer
    be trusted to resume correctly from).

    :param str statefile: path to the state file

    :return int | None: the saved TOW_selected, or None if unavailable/stale/invalid
    """

    try:
        with open(statefile) as f:
            state = json.load(f)
        tow = valid_tow(str(state["TOW_selected"]))
        interval = state["interval"]
        saved_at = datetime.fromisoformat(state["saved_at"])
        age = (datetime.now(timezone.utc) - saved_at).total_seconds()
        if age > interval:
            print(
                f"Ignoring stale state file {statefile}: saved {age:.0f}s ago "
                f"(week {state.get('week')}, TOW={tow}), exceeds interval={interval}s")
            return None
        return tow
    except (OSError, ValueError, KeyError, ArgumentTypeError, TypeError):
        return None


def str2bool(v: str) -> bool:
    """
    Converts a command-line string argument to a bool.
    (argparse's type=bool is broken: bool("False") is True)
    """

    if isinstance(v, bool):
        return v
    if v.lower() in ("yes", "true", "t", "y", "1"):
        return True
    if v.lower() in ("no", "false", "f", "n", "0"):
        return False
    raise ArgumentTypeError(f"Boolean value expected, got {v!r}")


def valid_tow(v: str) -> int:
    """
    Converts a command-line string argument to a valid GPS TOW.
    """

    tow = int(v)
    if not (0 <= tow < GPSWEEK_SECONDS):
        raise ArgumentTypeError(
            f"TOW must be in range [0, {GPSWEEK_SECONDS}), got {tow}")
    return tow


if __name__ == "__main__":
    arp = ArgumentParser(
        formatter_class=ArgumentDefaultsHelpFormatter,
    )
    arp.add_argument(
        "-P", "--port", required=False, help="Serial port", default="/dev/serial0"
    )
    arp.add_argument(
        "-B", "--baudrate", required=False, help="Baud rate", default=19200, type=int
    )
    arp.add_argument(
        "-T", "--timeout", required=False, help="Timeout in secs", default=3, type=float
    )
    arp.add_argument(
        "-ID",
        "--idonly",
        required=False,
        help="Print only the message entity (True|False)",
        default=False,
        type=str2bool,
    )
    arp.add_argument(
        "-F",
        "--filtered",
        required=False,
        help="Print only filtered messages (True|False)",
        default=True,
        type=str2bool,
    )
    arp.add_argument(
        "-W",
        "--WaitTOW",
        required=False,
        help="Wait for a specific TOW, grab and terminate (int)",
        default=False,
        type=valid_tow
    )
    arp.add_argument(
        "-i",
        "--interval",
        required=False,
        help="Interval between data captures (int)",
        default=False,
        type=int
    )
    arp.add_argument(
        "-O",
        "--output",
        required=False,
        help="File to write to (str)",
        default=False,
        type=str
    )
    arp.add_argument(
        "-S",
        "--statefile",
        required=False,
        help="File to persist/restore the current TOW_selected across restarts (str)",
        default=False,
        type=str
    )

    args = arp.parse_args()
    send_queue = Queue()
    stop_event = Event()

    bus = smbus.SMBus(0)
    address = 0x60
    siTime = SiT5721(bus, address)

    data = {
        'SiT.data_valid': False,
        'TIM_TOS.data_valid': False,
        'TIM-SMEAS.data_valid': False,
        'PUBX04.data_valid': False,
        # Best-effort, not part of the data_valid gate - see nav_pvt_display().
        'NAV-PVT.numSV': None,
        'NAV-PVT.received_at': None,
    }
    data_lock = Lock()
    TOW_old = int(0)
    TOW_selected = int(args.WaitTOW)
    process_start = monotonic()
    messages_checked = False

    if args.WaitTOW is False and args.statefile:
        resumed_tow = load_tow_state(args.statefile)
        if resumed_tow is not None:
            TOW_selected = resumed_tow
            args.WaitTOW = resumed_tow
            print(f"Resumed TOW_selected={TOW_selected} from {args.statefile}")

    TOW_next = TOW_selected
    weeks_crossed = 0

    try:
        print(f"Starting GNSS reader/writer... (get-data.py v{__version__})\n")
        print(f"args= {args}")
        with GNSSSkeletonApp(
            args.port,
            int(args.baudrate),
            float(args.timeout),
            stop_event,
            sendqueue=send_queue,
            idonly=args.idonly,
            filtered=bool(args.filtered),
            enableubx=False,
            showhacc=False,
        ) as gna:
            gna.run()
            while True:
                sleep(0.2)

                if stop_event.is_set():
                    print("Reader thread stopped unexpectedly, exiting.")
                    sys.exit(1)

                with data_lock:
                    snapshot = dict(data)

                # One-shot, not a retry loop: give the receiver a grace
                # period to prove each required message is already flowing
                # (the normal case today - see REQUIRED_MESSAGES/enable_ubx()
                # notes above), and only touch the receiver for whichever
                # ones genuinely never showed up. If that CFG-MSG send also
                # fails for some reason, we don't hammer the receiver again
                # this run - the next process start (or the operator, from
                # this warning) tries again.
                if not messages_checked and (monotonic() - process_start) > MESSAGE_GRACE_PERIOD_S:
                    messages_checked = True
                    for name, (msgClass, msgID) in REQUIRED_MESSAGES.items():
                        if f"_seen.{name}" not in snapshot:
                            print(
                                f"WARNING: {name} not observed within {MESSAGE_GRACE_PERIOD_S}s, "
                                f"requesting it via CFG-MSG")
                            gna.enable_message(msgClass, msgID)

                if snapshot['TIM_TOS.data_valid']:
                    if args.WaitTOW is not False:
                        if args.interval is not False:
                            TOW_next, weeks_crossed = calcNextTOW(
                                TOW_selected, args.interval)

                        if snapshot['TIM-TOS.TOW'] != TOW_old:
                            # Signed distance, wrapped across the GPS week
                            # boundary (centered on 0 so a small negative
                            # overshoot right at/after the target still
                            # shows correctly instead of jumping to +7 days).
                            waitTimeRemaining = int(
                                (TOW_selected - snapshot['TIM-TOS.TOW'] + GPSWEEK_SECONDS // 2)
                                % GPSWEEK_SECONDS - GPSWEEK_SECONDS // 2)

                            print(
                                f"...Waiting for TOW={TOW_selected:6d}, we're at {snapshot['TIM-TOS.TOW']:6d}", end='')

                            if args.interval is not False:
                                print(f", interval= {args.interval:6d}")
                            else:
                                print()

                            print(
                                f"...Time to go: {waitTimeRemaining} s or  {timedelta(seconds=waitTimeRemaining)}")

                            TOW_old = snapshot['TIM-TOS.TOW']

                if (snapshot['TIM_TOS.data_valid'] and snapshot['SiT.data_valid'] and snapshot['TIM-SMEAS.data_valid'] and snapshot['PUBX04.data_valid']):
                    if args.WaitTOW is not False:
                        if ((snapshot['TIM-TOS.TOW'] - TOW_selected) % GPSWEEK_SECONDS) < (2 * 60):
                            # If TOW_selected is now or in the past 2 minutes
                            # (wraps correctly across the GPS week boundary):
                            printToScreen_calib_data(snapshot)

                            if args.output:
                                printToFile_calib_data(
                                    snapshot, args.output, TOW_selected)
                                try:
                                    parse_sit.regenerate(
                                        args.output, verbose=False)
                                except Exception as e:
                                    out_dir = os.path.dirname(
                                        os.path.abspath(args.output)) or "."
                                    err_log = os.path.join(
                                        out_dir, "parsed_records_errors.log")
                                    print(
                                        f"WARNING: parse_sit regeneration failed: {e!r}", file=sys.stderr)
                                    try:
                                        with open(err_log, "a") as f:
                                            f.write(
                                                f"{datetime.now(timezone.utc).isoformat()} {e!r}\n")
                                    except OSError:
                                        pass

                            if args.interval is not False:
                                TOW_selected = TOW_next

                                if args.statefile:
                                    save_tow_state(
                                        args.statefile, TOW_selected, args.interval,
                                        snapshot['TIM-TOS.week'] + weeks_crossed)

                            else:
                                sys.exit()

                    else:
                        continue

    except KeyboardInterrupt:
        stop_event.set()
        print("Terminated by user")
