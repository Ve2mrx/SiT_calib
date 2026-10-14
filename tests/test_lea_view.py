"""
Tests for lea_view.py (TODO B8). Run from the repo root:
    python -m unittest discover -s tests -v

The TIM-TOS / TIM-SMEAS frames are built with pyubx2 and parsed back, so
the attribute names lea_view relies on are checked against the installed
pyubx2 (1.3.4 on the device), not against a hand-written fake.
"""

import os
import sys
import tempfile
import unittest
from datetime import datetime, timezone

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import lea_view  # noqa: E402
from pyubx2 import GET, UBXMessage, UBXReader  # noqa: E402


def tim_tos(**kw):
    fields = dict(version=0, gnssId=0, DiscSrc=1, gnssTimeValid=1,
                  UTCTimeValid=1, timeInLimit=1, intOscInLimit=1,
                  extOscInLimit=1, lockedPulse=1, year=2026, month=10, day=14,
                  hour=12, minute=0, second=0, utcStandard=3, week=2440,
                  TOW=388800, gnssUncertainty=4,
                  intOscOffset=1.5, intOscUncertainty=2.25,
                  extOscOffset=-3.75, extOscUncertainty=0.5)
    fields.update(kw)
    msg = UBXMessage("TIM", "TIM-TOS", GET, **fields)
    return UBXReader.parse(msg.serialize())


def tim_smeas(blocks):
    """blocks: list of (sourceId, freqValid, freqOffset, freqUnc)."""
    kw = dict(version=0, numMeas=len(blocks), iTOW=388800000)
    for i, (sid, fv, fo, fu) in enumerate(blocks, start=1):
        kw[f"sourceId_{i:02d}"] = sid
        kw[f"freqValid_{i:02d}"] = fv
        kw[f"phaseValid_{i:02d}"] = 0
        kw[f"freqOffset_{i:02d}"] = fo
        kw[f"freqUnc_{i:02d}"] = fu
    msg = UBXMessage("TIM", "TIM-SMEAS", GET, **kw)
    return UBXReader.parse(msg.serialize())


class Clock:
    def __init__(self):
        self.t = 1000.0

    def __call__(self):
        return self.t


def snapshot(tow, **extra):
    s = {"TIM_TOS.data_valid": True, "TIM-TOS.TOW": tow, "TIM-TOS.week": 2440,
         "TIM-TOS.gnssUncertainty": 4,
         "LEA.tos.DiscSrc": 1, "LEA.tos.extOscInLimit": 1,
         "LEA.tos.extOscOffset": 2.0,
         "LEA.smeas.ext0.freqOffset": 1.0, "LEA.smeas.ext0.freqUnc": 3.0,
         "LEA.smeas.ext0.freqValid": 1}
    s.update(extra)
    return s


class TestExtract(unittest.TestCase):
    def test_tos_extra_reads_flags_and_offsets(self):
        out = lea_view.tos_extra(tim_tos(DiscSrc=2, extOscInLimit=0))
        self.assertEqual(out["LEA.tos.DiscSrc"], 2)
        self.assertEqual(out["LEA.tos.extOscInLimit"], 0)
        self.assertEqual(out["LEA.tos.lockedPulse"], 1)
        # pyubx2 scales the 2^-8 ppb raw units; 2^-8 multiples survive exactly
        self.assertEqual(out["LEA.tos.extOscOffset"], -3.75)
        self.assertEqual(out["LEA.tos.intOscUncertainty"], 2.25)

    def test_tos_extra_never_raises(self):
        class Bad:
            def __getattr__(self, name):
                raise RuntimeError("boom")
        self.assertEqual(lea_view.tos_extra(Bad()), {})
        self.assertEqual(lea_view.tos_extra(object()), {})

    def test_smeas_extra_finds_ext0_in_any_block(self):
        # EXTINT0 deliberately NOT in block 03 (get-data.py's own parser's assumption)
        out = lea_view.smeas_extra(tim_smeas([(2, 1, 0.75, 4.0), (0, 1, -1.25, 9.0),
                                              (1, 0, 0.0, 0.0)]))
        self.assertEqual(out["LEA.smeas.ext0.freqOffset"], 0.75)
        self.assertEqual(out["LEA.smeas.ext0.freqUnc"], 4.0)
        self.assertEqual(out["LEA.smeas.int.freqOffset"], -1.25)
        self.assertNotIn("LEA.smeas.gnss.freqOffset", out)

    def test_smeas_extra_missing_source_overwrites_stale_value(self):
        data = lea_view.smeas_extra(tim_smeas([(2, 1, 0.75, 4.0)]))
        # next second EXTINT0 is gone (e.g. SiT power loss): must not stay 0.75
        data.update(lea_view.smeas_extra(tim_smeas([(1, 1, 0.0, 1.0)])))
        self.assertIsNone(data["LEA.smeas.ext0.freqOffset"])
        self.assertIsNone(data["LEA.smeas.int.freqUnc"])
        self.assertTrue(all(v is None for v in lea_view.smeas_extra(object()).values()))


class TestLogger(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.tmp.name, "lea-view.csv")
        self.clock = Clock()
        self.wall = lambda: datetime(2026, 10, 14, 12, 0, 0, tzinfo=timezone.utc)
        self.log = lea_view.LeaViewLogger(self.path, 60, clock=self.clock,
                                          wallclock=self.wall)

    def tearDown(self):
        self.tmp.cleanup()

    def feed(self, seconds, start_tow=100, **extra):
        for i in range(seconds):
            # 5 main-loop ticks per TIM-TOS second, as in get-data.py (0.2 s)
            for _ in range(5):
                self.log.observe(snapshot(start_tow + i, **extra))
            self.clock.t += 1.0

    def rows(self):
        with open(self.path) as f:
            return [line.rstrip("\n").split(",") for line in f]

    def test_one_row_per_minute_with_header_and_seconds_counted_once(self):
        self.feed(121)
        rows = self.rows()
        self.assertEqual(rows[0], list(lea_view.COLUMNS))
        self.assertEqual(len(rows), 3)
        r = dict(zip(rows[0], rows[1]))
        self.assertEqual(r["version"], "1")
        self.assertEqual(r["utc"], "2026-10-14T12:00:00Z")
        self.assertEqual(r["n_sec"], "61")       # t=0..60 inclusive, ticks deduped
        self.assertEqual(r["discsrc_seen"], "1")
        self.assertEqual(r["n_not_gnss"], "0")
        self.assertEqual(r["ext_osc_offset_mean_ppb"], "2.00000")
        self.assertEqual(r["smeas_ext0_freq_unc_ppb"], "3.0")
        self.assertEqual(len(rows[1]), len(lea_view.COLUMNS))

    def test_discsrc_excursion_inside_a_minute_is_counted(self):
        self.feed(30)
        self.feed(3, start_tow=130, **{"LEA.tos.DiscSrc": 0, "LEA.tos.extOscInLimit": 0})
        self.feed(30, start_tow=133)
        r = dict(zip(*self.rows()[:2]))
        self.assertEqual(r["discsrc_seen"], "0|1")
        self.assertEqual(r["n_not_gnss"], "3")
        self.assertEqual(r["n_ext_not_in_limit"], "3")
        self.assertEqual(r["discsrc"], "1")      # last sample, recovered

    def test_invalid_or_missing_tos_is_ignored(self):
        for i in range(70):
            self.log.observe({"TIM_TOS.data_valid": False, "TIM-TOS.TOW": i})
            self.clock.t += 1
        self.assertFalse(os.path.exists(self.path))

    def test_missing_lea_fields_give_empty_cells_not_errors(self):
        for i in range(61):
            self.log.observe({"TIM_TOS.data_valid": True, "TIM-TOS.TOW": i})
            self.clock.t += 1
        r = dict(zip(*self.rows()[:2]))
        self.assertEqual(r["discsrc_seen"], "?")
        self.assertEqual(r["ext_osc_offset_ppb"], "")
        self.assertEqual(self.log.errors, 0)

    def test_unwritable_file_never_raises_and_does_not_retry_every_second(self):
        err = open(os.devnull, "w")
        self.addCleanup(err.close)
        bad = lea_view.LeaViewLogger(os.path.join(self.tmp.name, "no", "dir.csv"), 60,
                                     clock=self.clock, wallclock=self.wall, errstream=err)
        for i in range(125):
            bad.observe(snapshot(i))
            self.clock.t += 1
        self.assertEqual(bad.errors, 2)          # one per due row, not one per second
        self.assertEqual(bad.rows_written, 0)

    def test_errors_go_to_error_log_throttled(self):
        err = open(os.devnull, "w")
        self.addCleanup(err.close)
        errlog = os.path.join(self.tmp.name, "lea-view_errors.log")
        log = lea_view.LeaViewLogger(self.path, 60, clock=self.clock, wallclock=self.wall,
                                     errstream=err, errlog=errlog)
        for _ in range(250):
            log.observe(None)                    # raises inside _observe every time
        with open(errlog) as f:
            lines = f.read().splitlines()
        # errors 1..10, then #100 and #200
        self.assertEqual(len(lines), 12)
        self.assertIn("lea-view error #1 ", lines[0])
        self.assertIn("lea-view error #200 ", lines[-1])

    def test_default_error_log_name(self):
        self.assertEqual(self.log.errlog, os.path.join(self.tmp.name, "lea-view_errors.log"))

    def test_unwritable_error_log_never_raises(self):
        err = open(os.devnull, "w")
        self.addCleanup(err.close)
        log = lea_view.LeaViewLogger(os.path.join(self.tmp.name, "no", "x.csv"), 60,
                                     clock=self.clock, wallclock=self.wall, errstream=err)
        log.observe(None)
        self.assertEqual(log.errors, 1)

    def test_appends_to_existing_file_without_second_header(self):
        self.feed(61)
        log2 = lea_view.LeaViewLogger(self.path, 60, clock=self.clock, wallclock=self.wall)
        for i in range(61):
            log2.observe(snapshot(1000 + i))
            self.clock.t += 1
        rows = self.rows()
        self.assertEqual(sum(1 for r in rows if r[0] == "version"), 1)
        self.assertEqual(len(rows), 3)


if __name__ == "__main__":
    unittest.main()
