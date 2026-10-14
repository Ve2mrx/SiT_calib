"""
The LEA-M8F's own view of its frequency sources, logged once a minute.

TODO.md B8 (2026-10-14). The SiT5721 is not steered by the LEA: it is a
CFG-ESRC *frequency source* on EXTINT0, and the LEA's sync manager learns
the SiT's offset against GNSS. What the receiver currently believes about
that arrangement is only visible in two periodic UART1 messages that
get-data.py already parses every second:

- TIM-TOS: DiscSrc (which source disciplines the LEA right now - 1 = GNSS),
  the in-limit / locked-pulse flags, and the internal / external oscillator
  frequency offsets and uncertainties (ppb).
- TIM-SMEAS: per-source frequency offset / uncertainty, incl. EXTINT0.

What the numbers are measured AGAINST (u-blox M8 protocol spec UBX-13003221
R28; copy + notes in GNSS_parameters/docs/u-blox/):
- TIM-TOS intOsc*/extOsc* describe the oscillators the LEA *disciplines*
  (sec. 32.20.7). The SiT is not one of them - it is only measured - so
  ext_osc_* read 0 and ext_osc_in_limit 0 permanently on timecard-mini.
  Kept as columns so a change in that would show.
- TIM-SMEAS sourceId 0 is the internal oscillator vs the receiver's current
  time/frequency estimate; sourceIds 1-3 are measured AGAINST THE INTERNAL
  OSCILLATOR: 1 = GNSS, 2 = EXTINT0 (the SiT) (sec. 32.20.4). So the SiT
  vs GNSS is ext0 relative to gnss, not ext0 alone - which is why v2 logs
  sourceId 1. (The daily CSV's freq_offset_ppb is the same EXTINT0 value.)
  The sign convention of that difference is not stated in the spec - check
  it against the daily phase-derived e before relying on it.

Neither can be polled over the LEA's I2C port (TIM-TOS gets no reply,
measured 2026-10-14), and UART1 belongs to the capture - so get-data.py is
the only place this can be logged without disturbing anything.

Purpose: (a) confirm GNSS stays the reference after the 2026-10-14 CFG-ESRC
change (timeToTemp 150 -> 3600 s); (b) record the post-power-loss retrace
curve, the evidence for tightening timeToTemp / withTemp; (c) holdover
evidence.

Hard rule: this must never affect the daily capture. Hence:
- tos_extra() / smeas_extra() run in get-data.py's reader THREAD, where any
  exception is "Fatal error in GNSS read loop" and stops the capture. They
  catch everything and return what they could read.
- LeaViewLogger.observe() runs in the main loop on the same snapshot the
  capture uses, reads it only, and catches everything (file errors
  included). It writes to its own file; the daily CSV / output file /
  parse_sit are untouched.
- It is opt-in: get-data.py only builds a logger when --lea-view is given.

Output: one LEAVIEW,<version>,<UTC ISO timestamp>,... line per ~60 s of
TIM-TOS seconds, appended forever to one file - the same convention as
SiT5721's ~/sit-health.csv (HEALTH,<version>,...) and parse_sit.py's
CSV,<version>,... records: no header, every line carries its own version,
lines of different versions may share the file, and readers dispatch per
line (parse_lea_view_line()). Field lists are append-only.

Like sit-health.csv - and unlike the calibration log - this file is
continuous telemetry: it is NOT rotated into ~/SiT-calib_archive/ at a
re-tune or a power loss. A power loss is exactly when its record has to be
unbroken (the retrace curve).
"""

import os
import sys
from datetime import datetime, timezone
from time import monotonic

LINE_TAG = "LEAVIEW"

# Bump when the LEAVIEW,<version>,... field list changes, and add a
# LEA_VIEW_FIELDS_V<N> - same convention as HEALTH_LINE_VERSION /
# HEALTH_CSV_FIELDS_V<N> in SiT5721's save-SiT5721.py.
LEA_VIEW_LINE_VERSION = 2

# TIM-TOS flag bits pyubx2 (1.3.4) exposes as attributes. DiscSrc: 0 internal
# oscillator, 1 GNSS, 2 EXTINT0, 3 EXTINT1, 4 internal osc measured by host,
# 5 external osc measured by host (u-blox M8 timing protocol spec, TIM-TOS).
_TOS_FLAGS = (
    "DiscSrc", "gnssTimeValid", "UTCTimeValid", "timeInLimit",
    "intOscInLimit", "extOscInLimit", "lockedPulse", "raim", "cohPulse",
)
# Already scaled to ppb by pyubx2 (2^-8 ppb raw units).
_TOS_OSC = ("intOscOffset", "intOscUncertainty", "extOscOffset", "extOscUncertainty")

# TIM-SMEAS sourceId: 0 internal oscillator (vs the receiver's estimate);
# 1 GNSS, 2 EXTINT0, 3 EXTINT1 (each vs the internal oscillator).
SMEAS_SOURCES = {0: "int", 1: "gnss", 2: "ext0"}

# Field order for a LEAVIEW,1,... line, timestamp excluded (it is the third
# field, handled separately in parse_lea_view_line()). v1 ran 2026-10-14
# 14:40Z-14:57Z: TIM-SMEAS sources 0 + 2 only.
LEA_VIEW_FIELDS_V1 = [
    "week", "tow",
    "n_sec",                 # TIM-TOS seconds seen in this line's window
    "discsrc_seen",          # distinct DiscSrc values in the window, e.g. "1" or "0|1"
    "n_not_gnss",            # seconds with DiscSrc != 1
    "n_ext_not_in_limit",    # seconds with extOscInLimit == 0 (all of them, here)
    "discsrc", "gnss_time_valid", "utc_time_valid", "time_in_limit",
    "int_osc_in_limit", "ext_osc_in_limit", "locked_pulse",
    "gnss_unc_ns",
    "int_osc_offset_ppb", "int_osc_unc_ppb",
    "ext_osc_offset_ppb", "ext_osc_unc_ppb",
    "ext_osc_offset_mean_ppb",
    "smeas_int_freq_valid", "smeas_int_freq_offset_ppb", "smeas_int_freq_unc_ppb",
    "smeas_ext0_freq_valid", "smeas_ext0_freq_offset_ppb", "smeas_ext0_freq_unc_ppb",
    "smeas_ext0_freq_offset_mean_ppb",
]

# v2 (2026-10-14 14:58Z) = v1 + TIM-SMEAS sourceId 1 (GNSS vs internal osc),
# without which the SiT vs GNSS cannot be reconstructed. Append-only.
LEA_VIEW_FIELDS_V2 = LEA_VIEW_FIELDS_V1 + [
    "smeas_gnss_freq_valid", "smeas_gnss_freq_offset_ppb", "smeas_gnss_freq_unc_ppb",
    "smeas_gnss_freq_offset_mean_ppb",
]

LEA_VIEW_FIELDS = {1: LEA_VIEW_FIELDS_V1, 2: LEA_VIEW_FIELDS_V2}


def parse_lea_view_line(line: str):
    """
    Parses one LEAVIEW,<version>,<iso>,... line. No field can contain a
    comma ("|" separates discsrc_seen), so a plain split is safe.

    :return tuple[datetime, int, dict] | None: (UTC timestamp, version,
        {field: str}) - values stay strings; empty means "not available".
        None for anything that is not a recognized, well-formed LEAVIEW line
        (unknown version, wrong field count, bad timestamp) - skipped, never
        raised, so one bad line cannot lose the rest of the file.
    """
    parts = line.rstrip("\r\n").split(",")
    if len(parts) < 3 or parts[0] != LINE_TAG:
        return None
    try:
        version = int(parts[1])
        ts = datetime.fromisoformat(parts[2])
    except ValueError:
        return None
    fields = LEA_VIEW_FIELDS.get(version)
    if fields is None or len(parts) != 3 + len(fields):
        return None
    return ts, version, dict(zip(fields, parts[3:]))


def tos_extra(parsed) -> dict:
    """
    LEA.tos.* keys from a parsed TIM-TOS. Never raises (reader thread).
    A field pyubx2 does not provide is simply absent.
    """
    out = {}
    try:
        for name in _TOS_FLAGS:
            v = getattr(parsed, name, None)
            if v is not None:
                out[f"LEA.tos.{name}"] = int(v)
        for name in _TOS_OSC:
            v = getattr(parsed, name, None)
            if v is not None:
                out[f"LEA.tos.{name}"] = float(v)
    except Exception:  # pylint: disable=broad-except
        pass
    return out


def smeas_extra(parsed) -> dict:
    """
    LEA.smeas.<src>.* keys for the sources in SMEAS_SOURCES, found by
    sourceId in whichever measurement block carries them (get-data.py's
    own TIM-SMEAS parser assumes EXTINT0 is block 03; this does not).
    A source missing from this message is returned as None, NOT left out:
    get-data.py merges this into a dict that persists across seconds, so an
    absent key would keep the last value - e.g. a SiT that lost power would
    go on being logged with its pre-outage offset. Never raises.
    """
    out = {f"LEA.smeas.{src}.{field}": None
           for src in SMEAS_SOURCES.values()
           for field in ("freqValid", "freqOffset", "freqUnc")}
    try:
        for i in range(1, int(getattr(parsed, "numMeas", 0)) + 1):
            sid = getattr(parsed, f"sourceId_{i:02d}", None)
            src = SMEAS_SOURCES.get(sid)
            if src is None:
                continue
            out[f"LEA.smeas.{src}.freqValid"] = int(getattr(parsed, f"freqValid_{i:02d}"))
            out[f"LEA.smeas.{src}.freqOffset"] = float(getattr(parsed, f"freqOffset_{i:02d}"))
            out[f"LEA.smeas.{src}.freqUnc"] = float(getattr(parsed, f"freqUnc_{i:02d}"))
    except Exception:  # pylint: disable=broad-except
        pass
    return out


def _fmt(v) -> str:
    if v is None:
        return ""
    if isinstance(v, float):
        return repr(v)
    return str(v)


def _mean(values) -> str:
    return f"{sum(values) / len(values):.5f}" if values else ""


class LeaViewLogger:
    """
    Feed it every main-loop snapshot; it counts each new TIM-TOS second once
    (keyed on TOW) and appends one row per `interval_s`. observe() never
    raises - errors are counted and reported, sparingly, on stderr (the
    screen session, which keeps no log) AND in `errlog` (default:
    <path stem>_errors.log), so a failure can be found after the fact.

    The error log only sees failures that raise. A logger that silently
    writes nothing (TIM-TOS never valid, option dropped from the launcher)
    shows up only as a stale last row in `path` - check its age, not just
    whether the error log is empty.
    """

    # Report errors 1..ERR_FIRST, then every ERR_EVERY-th: a fault that hits
    # every second would otherwise write ~86k lines a day.
    ERR_FIRST = 10
    ERR_EVERY = 100

    def __init__(self, path: str, interval_s: float = 60.0,
                 clock=monotonic, wallclock=None, errstream=None,
                 errlog=None):
        self.path = path
        self.errlog = errlog or os.path.splitext(path)[0] + "_errors.log"
        self.interval_s = float(interval_s)
        self._clock = clock
        self._wallclock = wallclock or (lambda: datetime.now(timezone.utc))
        self._err = errstream if errstream is not None else sys.stderr
        self.errors = 0
        self.rows_written = 0
        self._last_tow = None
        self._due = None
        self._reset_window()

    def _reset_window(self):
        self._n = 0
        self._disc_seen = set()
        self._n_not_gnss = 0
        self._n_ext_out = 0
        self._ext_offsets = []
        self._smeas_ext0 = []
        self._smeas_gnss = []
        self._last = {}

    def observe(self, snap: dict) -> None:
        try:
            self._observe(snap)
        except Exception as err:  # pylint: disable=broad-except
            self.errors += 1
            if self.errors <= self.ERR_FIRST or self.errors % self.ERR_EVERY == 0:
                self._report(err)

    def _report(self, err) -> None:
        # Must not raise either: the error may BE the disk (full, read-only),
        # in which case the error log fails too and stderr is all there is.
        msg = (f"lea-view error #{self.errors} ({self.path}): {err!r}"
               f" - capture unaffected")
        try:
            print(f"WARNING: {msg}", file=self._err)
        except Exception:  # pylint: disable=broad-except
            pass
        try:
            with open(self.errlog, "a") as f:
                f.write(f"{datetime.now(timezone.utc).isoformat()} {msg}\n")
        except Exception:  # pylint: disable=broad-except
            pass

    def _observe(self, snap: dict) -> None:
        if not snap.get("TIM_TOS.data_valid"):
            return
        tow = snap.get("TIM-TOS.TOW")
        if tow is None or tow == self._last_tow:
            return
        self._last_tow = tow

        now = self._clock()
        if self._due is None:
            self._due = now + self.interval_s

        self._n += 1
        disc = snap.get("LEA.tos.DiscSrc")
        self._disc_seen.add(disc)
        if disc != 1:
            self._n_not_gnss += 1
        if snap.get("LEA.tos.extOscInLimit") == 0:
            self._n_ext_out += 1
        ext = snap.get("LEA.tos.extOscOffset")
        if ext is not None:
            self._ext_offsets.append(ext)
        sm = snap.get("LEA.smeas.ext0.freqOffset")
        if sm is not None:
            self._smeas_ext0.append(sm)
        sg = snap.get("LEA.smeas.gnss.freqOffset")
        if sg is not None:
            self._smeas_gnss.append(sg)
        self._last = snap

        if now >= self._due:
            # Advance the schedule before writing: a write that fails must
            # not be retried every second for the rest of the minute.
            self._due = now + self.interval_s
            try:
                self._write_row()
            finally:
                self._reset_window()

    def row(self) -> list:
        """The LEAVIEW line, tag first; field order = LEA_VIEW_FIELDS_V<N>."""
        s = self._last
        seen = "|".join(str(d) if d is not None else "?" for d in
                        sorted(self._disc_seen, key=lambda d: (d is None, d)))
        return [
            LINE_TAG, LEA_VIEW_LINE_VERSION,
            # isoformat() with +00:00, like sit-health.csv's HEALTH lines
            self._wallclock().isoformat(),
            _fmt(s.get("TIM-TOS.week")), _fmt(s.get("TIM-TOS.TOW")),
            self._n, seen, self._n_not_gnss, self._n_ext_out,
            _fmt(s.get("LEA.tos.DiscSrc")),
            _fmt(s.get("LEA.tos.gnssTimeValid")),
            _fmt(s.get("LEA.tos.UTCTimeValid")),
            _fmt(s.get("LEA.tos.timeInLimit")),
            _fmt(s.get("LEA.tos.intOscInLimit")),
            _fmt(s.get("LEA.tos.extOscInLimit")),
            _fmt(s.get("LEA.tos.lockedPulse")),
            _fmt(s.get("TIM-TOS.gnssUncertainty")),
            _fmt(s.get("LEA.tos.intOscOffset")),
            _fmt(s.get("LEA.tos.intOscUncertainty")),
            _fmt(s.get("LEA.tos.extOscOffset")),
            _fmt(s.get("LEA.tos.extOscUncertainty")),
            _mean(self._ext_offsets),
            _fmt(s.get("LEA.smeas.int.freqValid")),
            _fmt(s.get("LEA.smeas.int.freqOffset")),
            _fmt(s.get("LEA.smeas.int.freqUnc")),
            _fmt(s.get("LEA.smeas.ext0.freqValid")),
            _fmt(s.get("LEA.smeas.ext0.freqOffset")),
            _fmt(s.get("LEA.smeas.ext0.freqUnc")),
            _mean(self._smeas_ext0),
            _fmt(s.get("LEA.smeas.gnss.freqValid")),
            _fmt(s.get("LEA.smeas.gnss.freqOffset")),
            _fmt(s.get("LEA.smeas.gnss.freqUnc")),
            _mean(self._smeas_gnss),
        ]

    def _write_row(self) -> None:
        line = ",".join(str(v) for v in self.row())
        with open(self.path, "a") as f:
            f.write(line + "\n")
        self.rows_written += 1
