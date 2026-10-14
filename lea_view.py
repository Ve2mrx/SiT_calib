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

Output: one CSV row per ~60 s of TIM-TOS seconds (header written when the
file is new or empty). Bump LEA_VIEW_VERSION when the column list changes.
"""

import os
import sys
from datetime import datetime, timezone
from time import monotonic

LEA_VIEW_VERSION = 1

# TIM-TOS flag bits pyubx2 (1.3.4) exposes as attributes. DiscSrc: 0 internal
# oscillator, 1 GNSS, 2 EXTINT0, 3 EXTINT1, 4 internal osc measured by host,
# 5 external osc measured by host (u-blox M8 timing protocol spec, TIM-TOS).
_TOS_FLAGS = (
    "DiscSrc", "gnssTimeValid", "UTCTimeValid", "timeInLimit",
    "intOscInLimit", "extOscInLimit", "lockedPulse", "raim", "cohPulse",
)
# Already scaled to ppb by pyubx2 (2^-8 ppb raw units).
_TOS_OSC = ("intOscOffset", "intOscUncertainty", "extOscOffset", "extOscUncertainty")

# TIM-SMEAS sourceId: 0 internal oscillator, 1 GNSS, 2 EXTINT0, 3 EXTINT1.
SMEAS_SOURCES = {0: "int", 2: "ext0"}

COLUMNS = (
    "version", "utc", "week", "tow",
    "n_sec",                 # TIM-TOS seconds seen in this row's window
    "discsrc_seen",          # distinct DiscSrc values in the window, e.g. "1" or "0|1"
    "n_not_gnss",            # seconds with DiscSrc != 1
    "n_ext_not_in_limit",    # seconds with extOscInLimit == 0
    "discsrc", "gnss_time_valid", "utc_time_valid", "time_in_limit",
    "int_osc_in_limit", "ext_osc_in_limit", "locked_pulse",
    "gnss_unc_ns",
    "int_osc_offset_ppb", "int_osc_unc_ppb",
    "ext_osc_offset_ppb", "ext_osc_unc_ppb",
    "ext_osc_offset_mean_ppb",
    "smeas_int_freq_valid", "smeas_int_freq_offset_ppb", "smeas_int_freq_unc_ppb",
    "smeas_ext0_freq_valid", "smeas_ext0_freq_offset_ppb", "smeas_ext0_freq_unc_ppb",
    "smeas_ext0_freq_offset_mean_ppb",
)


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
        s = self._last
        seen = "|".join(str(d) if d is not None else "?" for d in
                        sorted(self._disc_seen, key=lambda d: (d is None, d)))
        return [
            LEA_VIEW_VERSION,
            self._wallclock().strftime("%Y-%m-%dT%H:%M:%SZ"),
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
        ]

    def _write_row(self) -> None:
        line = ",".join(str(v) for v in self.row())
        new = not os.path.exists(self.path) or os.path.getsize(self.path) == 0
        with open(self.path, "a") as f:
            if new:
                f.write(",".join(COLUMNS) + "\n")
            f.write(line + "\n")
        self.rows_written += 1
