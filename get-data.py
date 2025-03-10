"""
Initially from pygnssutils - gnssapp.py
Heavily modified by Martin Boissonneault

*** FOR ILLUSTRATION ONLY - NOT FOR PRODUCTION USE ***

Skeleton GNSS application which continuously receives, parses and prints
NMEA, UBX or RTCM data from a receiver until the stop Event is set or
stop() method invoked. Assumes receiver is connected via serial USB or UART1 port.

The app also implements basic methods needed by certain pygnssutils classes.

Optional keyword arguments:

- sendqueue - any data placed on this Queue will be sent to the receiver
  (e.g. UBX commands/polls or NTRIP RTCM data). Data must be a tuple of
  (raw_data, parsed_data).
- idonly - determines whether the app prints out the entire parsed message,
  or just the message identity.
- enableubx - suppresses NMEA receiver output and substitutes a minimum set
  of UBX messages instead (NAV-PVT, NAV-SAT, NAV-DOP, RXM-RTCM).
- showhacc - show estimate of horizonal accuracy in metres (if available).
- filtered - Shows only messages related to SiT5721 calibration

Uses:
https://github.com/semuconsulting/pyubx2/tree/master
https://github.com/semuconsulting/pynmeagps
https://github.com/pyserial/pyserial
apt python3-smbus
mbt_SiT5721_lib

Created on 2024 Dec 07

:author: semuadmin / Martin Boissonneault VE2MRX
:copyright: SEMU Consulting © 2023 / Martin Boissonneault VE2MRX © 2024
:license: BSD 3-Clause
"""

# pylint: disable=invalid-name, too-many-instance-attributes

from argparse import ArgumentDefaultsHelpFormatter, ArgumentParser
from queue import Empty, Queue
from threading import Event, Thread
from time import sleep

from pynmeagps import NMEAMessageError, NMEAParseError
from pyrtcm import RTCMMessage, RTCMMessageError, RTCMParseError
from serial import Serial

from pyubx2 import (
    NMEA_PROTOCOL,
    #    RTCM3_PROTOCOL,
    UBX_PROTOCOL,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
    gnss2str,
    itow2utc,
    GNSSLIST,
    UTCSTANDARD
)

from mbt_SiT5721_lib import SiT5721
import smbus

from datetime import timedelta, time, date, timezone
import sys

CONNECTED = 1

GPSWEEK_SECONDS = (7 * 24 * 60 * 60)  # 7 days * 24h * 60m * 60s


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

        ubr = UBXReader(stream, protfilter=(NMEA_PROTOCOL | UBX_PROTOCOL))
        while not stopevent.is_set():
            try:
                if stream.in_waiting:
                    _, parsed_data = ubr.read()
                    if parsed_data:
                        # extract current navigation solution
                        self._extract_coordinates(parsed_data)

                        if self.filtered:
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
                            else:
                                continue

                            print(f"GNSS>> {parsed_data.identity}{nty}")

                        elif self.idonly:
                            print(f"GNSS>> {parsed_data.identity}{nty}")

                        else:
                            print(parsed_data)

                # send any queued output data to receiver
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

        # create event of specified eventtype


def invalidate_SiT_data(data: dict):
    """
    Get SiT5721 data from SiTdev

    :param object SiT5721 SiTdev: Device to extract data from
    :param dict dict: name of dict to store the extracted data

    :return dict dict: Modified data dict
    """

    data['SiT.data_valid'] = False

    # SiT_config
    data['SiT.pull_value'] = None
    data['SiT.pull_range'] = None
    data['SiT.aging_compensation'] = None
    data['SiT.max_freq_ramp_rate'] = None
    # SiT_dynamic
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

    # SiT_config
    data['SiT.pull_value'] = float(SiTdev.pull_value)
    data['SiT.pull_range'] = float(SiTdev.pull_range)
    data['SiT.aging_compensation'] = float(SiTdev.aging_compensation)
    data['SiT.max_freq_ramp_rate'] = float(SiTdev.max_freq_ramp_rate)
    # SiT_dynamic
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
    :param dict dict: name of dict to store the extracted data

    :return dict dict: Modified data dict
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

    data['TIM_TOS.data_valid'] = True

    return data


def get_ubx_TIM_SMEAS_data(parsed_data: object, data: dict):
    """
    Get uBlox TIM-SMEAS data from GNSS

    :param object UBXReader parsed_data: data to fetch values from
    :param dict dict: name of dict to store the extracted data

    :return dict dict: Modified data dict
    """

    if parsed_data.sourceId_03 == 2:
        # If the source for sourceId_03 is "EXTINT0", save
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
        # If the sourceId_03 is NOT "EXTINT0", clear
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
    :param dict dict: name of dict to store the extracted data

    :return dict dict: Modified data dict
    """

    data['PUBX04.utcWk'] = int(parsed_data.utcWk)
    data['PUBX04.utcTow'] = float(parsed_data.utcTow)
    # On GNSS start, it is "16D", thus NOT int
    data['PUBX04.leapSec'] = str(parsed_data.leapSec)

    data['PUBX04.data_valid'] = True

    return data


def reset_data_valid(data: dict):
    """
    Get Reset data_valid from dict
    data_valid indicates if the message data exists and is up-to-date for this TOW.
    Resetting it is usually done when receiving a new TOW to invalidate the old data

    :param dict dict: name of dict to store the modified data

    :return dict dict: Modified data dict
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
    :param dict dict: name of dict to store the modified data

    :return str: string representing the UTC Standard
    """

    try:
        return UTCSTANDARD[utcStandard]
    except KeyError:
        return str(utcStandard)


def printToFile_calib_data(data: dict, file: str):
    """
    Print to file the results of the data collection

    :param dict dict: name of dict to store the modified data
    :param str file: filename used for output
    """

    # Create status strings from flags
    error_status_str = "undefined"
    stability_status_str = "undefined"

    def error_status(flag):
        # print(flag)
        if (flag == 7):
            error_status = "good"
        else:
            error_status = "ERROR"
        return error_status

    def stability_status(flag):
        # print(flag)
        if (flag == 1):
            error_status = "stabilized"
        else:
            error_status = "unstabilized"
        return error_status

    def flag_valid(flag: bool) -> str:
        FLAGVALUES = {
            False: "INVALID",
            True: "Valid",
        }
        try:
            return FLAGVALUES[flag]
        except KeyError:
            return str(flag)

    error_status_str = error_status(
        data['SiT.error_status_flag'])

    stability_status_str = stability_status(
        data['SiT.stability_flag'])

    with open(file, "a") as f:
        print(
            f"...Waiting for TOW={TOW_selected:6d}, we're at {data['TIM-TOS.TOW']:6d}", file=f)
        print("----------------------------------------", file=f)
        print(
            f"TIM-TOS  week, TOW, system  {data['TIM-TOS.week']:4d}, {data['TIM-TOS.TOW']:6d}, {data['TIM-TOS.gnssId.str']}", file=f)
        print(
            f"TIM-TOS  UTC                {str(data['TIM-TOS.utc.date'])}, {str(data['TIM-TOS.utc.time'])}, {data['TIM-TOS.utcStandard.str']}", file=f)
        print(file=f)

        print(
            f"TIM-SMEAS  iTOW:                  {data['TIM-SMEAS.iTOW'] / 1000:=6.3f}, source {data['TIM-SMEAS.source']}, flags(freq: {flag_valid(data['TIM-SMEAS.freqValid'])}, phase: {flag_valid(data['TIM-SMEAS.phaseValid'])})", file=f)
        print(
            f"TIM-SMEAS  phase offset:      {data['TIM-SMEAS.phaseOffset']:=10.3f} ns, freq offset:      {data['TIM-SMEAS.freqOffset']:=10.3f} ns", file=f)
        print(
            f"TIM-SMEAS  phase uncertainty:      {data['TIM-SMEAS.phaseUnc']:=10.3f} ns, freq uncertainty: {data['TIM-SMEAS.freqUnc']:=10.3f} ns", file=f)
        print(file=f)

        print(
            f"PUBX04  UTC week, TOW,      {data['PUBX04.utcWk']:4d}, {data['PUBX04.utcTow']:6.2f}, leapsec: {data['PUBX04.leapSec']}", file=f)
        print(file=f)

        print("SiT Uptime                {:8d}s, {}".format(
            data['SiT.uptime'],
            timedelta(seconds=data['SiT.uptime'])
        ), file=f, end='\n')
        print(file=f)

        print(
            f"SiT Error, Stability status flag      {error_status_str}, {stability_status_str}", file=f, end='\n')
        print(
            "SiT Pull Value             {:=+.8g} ppm".format(data['SiT.pull_value'] / pow(10, -6)), file=f, end='\n')
        print("SiT Pull Range              {:=.8g} ppm".format(
            data['SiT.pull_range'] / pow(10, -6)), file=f, end='\n')
        print(
            "SiT Aging compensation     {:=+.8g} part/s".format(data['SiT.aging_compensation']), file=f, end='\n')
        print("SiT Max. Freq Ramp Rate     {:=.8g} ppm".format(
            data['SiT.max_freq_ramp_rate'] / pow(10, -6)), file=f, end='\n')
        print(file=f)
        print("SiT Total offset written   {:=+.8g} ppm".format(
            data['SiT.total_offset_written'] / pow(10, -6)), file=f, end='\n')
        print("----------------------------------------", file=f)
        print(f"{data['TIM-TOS.week']:4d}, {data['TIM-TOS.TOW']:6d}, {data['TIM-SMEAS.phaseOffset']:=12.3f}, {data['SiT.total_offset_written'] / pow(10, -6):=+3.10g}, flags(freq: {flag_valid(data['TIM-SMEAS.freqValid'])}, phase: {flag_valid(data['TIM-SMEAS.phaseValid'])}), SiT status({error_status_str}, {stability_status_str})", file=f)
        print(file=f)


def printToScreen_calib_data(data: dict):
    """
    Converts the numeric UTC Standard value to a string

    :param int utcStandard: numeric UTC Standard value
    :param dict dict: name of dict to store the modified data

    :return str: string representing the UTC Standard
    """
    # Create status strings from flags
    error_status_str = "undefined"
    stability_status_str = "undefined"

    def error_status(flag):
        # print(flag)
        if (flag == 7):
            error_status = "good"
        else:
            error_status = "ERROR"
        return error_status

    def stability_status(flag):
        # print(flag)
        if (flag == 1):
            error_status = "stabilized"
        else:
            error_status = "unstabilized"
        return error_status

    def flag_valid(flag: bool) -> str:
        FLAGVALUES = {
            False: "INVALID",
            True: "Valid",
        }
        try:
            return FLAGVALUES[flag]
        except KeyError:
            return str(flag)

    error_status_str = error_status(
        data['SiT.error_status_flag'])
    stability_status_str = stability_status(
        data['SiT.stability_flag'])

    print("----------------------------------------")
    print(
        f"TIM-TOS  week, TOW, system  {data['TIM-TOS.week']:4d}, {data['TIM-TOS.TOW']:6d}, {data['TIM-TOS.gnssId.str']}")
    print(
        f"TIM-TOS  UTC                {str(data['TIM-TOS.utc.date'])}, {str(data['TIM-TOS.utc.time'])}, {data['TIM-TOS.utcStandard.str']}")
    print()

    print(
        f"TIM-SMEAS  iTOW:                  {data['TIM-SMEAS.iTOW'] / 1000:=6.3f}, source {data['TIM-SMEAS.source']}, flags(freq: {flag_valid(data['TIM-SMEAS.freqValid'])}, phase: {flag_valid(data['TIM-SMEAS.phaseValid'])})")
    print(
        f"TIM-SMEAS  phase offset:      {data['TIM-SMEAS.phaseOffset']:=10.3f} ns, freq offset:      {data['TIM-SMEAS.freqOffset']:=10.3f} ns")
    print(
        f"TIM-SMEAS  phase uncertainty:     {data['TIM-SMEAS.phaseUnc']:=10.3f} ns, freq uncertainty: {data['TIM-SMEAS.freqUnc']:=10.3f} ns")
    print()

    print(
        f"PUBX04  UTC week, TOW,      {data['PUBX04.utcWk']:4d}, {data['PUBX04.utcTow']:6.2f}, leapsec: {data['PUBX04.leapSec']}")
    print()

    print("SiT Uptime                {:8d}s, {}".format(
        data['SiT.uptime'],
        timedelta(seconds=data['SiT.uptime'])
    ), end='\n')
    print()

    print(
        f"SiT Error, Stability status flag      {error_status_str}, {stability_status_str}", end='\n')
    print(
        "SiT Pull Value             {:=+.8g} ppm".format(data['SiT.pull_value'] / pow(10, -6)), end='\n')
    print("SiT Pull Range              {:=.8g} ppm".format(
        data['SiT.pull_range'] / pow(10, -6)), end='\n')
    print(
        "SiT Aging compensation     {:=+.8g} part/s".format(data['SiT.aging_compensation']), end='\n')
    print("SiT Max. Freq Ramp Rate     {:=.8g} ppm".format(
        data['SiT.max_freq_ramp_rate'] / pow(10, -6)), end='\n')
    print()
    print("SiT Total offset written   {:=+3.10g} ppm".format(
        data['SiT.total_offset_written'] / pow(10, -6)), end='\n')
    print("----------------------------------------")


def calcNextTOW(tow: int, interval: int):
    """
    Calculates the next TOW (using TIM-TOS)

    :param int tow: current TOW
    :param int interval: interval in second between TOW

    :return int: calculated next TOW
    """

    nextTow = tow + interval
    if nextTow > GPSWEEK_SECONDS:
        nextTow = nextTow - GPSWEEK_SECONDS

    return nextTow


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
    )
    arp.add_argument(
        "-F",
        "--filtered",
        required=False,
        help="Print only filtered messages (True|False)",
        default=True,
        type=bool
    )
    arp.add_argument(
        "-W",
        "--WaitTOW",
        required=False,
        help="Wait for a specific TOW, grab and terminate (int)",
        default=False,
        type=int
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
    }
    TOW_old = int(0)
    TOW_selected = int(args.WaitTOW)
    TOW_next = TOW_selected

    try:
        print("Starting GNSS reader/writer...\n")
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

                if (bool(data['TIM_TOS.data_valid']) == True):
                    if bool(args.WaitTOW) == True:
                        if bool(args.interval) == True:
                            TOW_next = calcNextTOW(
                                TOW_selected, args.interval)

                        if data['TIM-TOS.TOW'] != TOW_old:
                            waitTimeRemaining = int(
                                TOW_selected - data['TIM-TOS.TOW'])

                            print(
                                f"...Waiting for TOW={TOW_selected:6d}, we're at {data['TIM-TOS.TOW']:6d}", end='')

                            if bool(args.interval) == True:
                                print(f", interval= {args.interval:6d}")
                            else:
                                print()

                            print(
                                f"...Time to go: {waitTimeRemaining} s or  {timedelta(seconds=waitTimeRemaining)}")

                            TOW_old = data['TIM-TOS.TOW']

                if (bool(data['TIM_TOS.data_valid']) and bool(data['SiT.data_valid']) and bool(data['TIM-SMEAS.data_valid']) and bool(data['PUBX04.data_valid']) == True):
                    # Message data is valid!
                    if bool(args.WaitTOW) == True:
                        if (data['TIM-TOS.TOW'] >= TOW_selected) and (data['TIM-TOS.TOW'] < TOW_selected + (2 * 60)):
                            # If TOW_selected is now or in the past 2 minutes:
                            printToScreen_calib_data(data)

                            if bool(args.output) == True:
                                printToFile_calib_data(data, args.output)

                            if bool(args.interval) == True:
                                TOW_selected = TOW_next

                            else:
                                sys.exit()

                    else:
                        # print_calib_data(data)
                        continue

    except KeyboardInterrupt:
        stop_event.set()
        print("Terminated by user")
