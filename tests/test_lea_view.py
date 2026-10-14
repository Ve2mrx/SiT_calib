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
        self.assertEqual(out["LEA.smeas.gnss.freqValid"], 0)
        self.assertNotIn("LEA.smeas.ext1.freqOffset", out)

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


    def lines(self):
        with open(self.path) as f:
            return f.read().splitlines()

    def rows(self):
        """Every line, parsed - a line that does not parse fails the test."""
        out = []
        for line in self.lines():
            parsed = lea_view.parse_lea_view_line(line)
            self.assertIsNotNone(parsed, f"unparseable line: {line[:60]}")
            out.append(parsed)
        return out

    def test_one_line_per_minute_tagged_versioned_and_seconds_counted_once(self):
        self.feed(121)
        lines = self.lines()
        self.assertEqual(len(lines), 2)                       # no header
        self.assertTrue(lines[0].startswith("LEAVIEW,2,2026-10-14T12:00:00+00:00,"))
        ts, version, r = self.rows()[0]
        self.assertEqual(version, lea_view.LEA_VIEW_LINE_VERSION)
        self.assertEqual(ts, datetime(2026, 10, 14, 12, 0, 0, tzinfo=timezone.utc))
        self.assertEqual(r["n_sec"], "61")       # t=0..60 inclusive, ticks deduped
        self.assertEqual(r["discsrc_seen"], "1")
        self.assertEqual(r["n_not_gnss"], "0")
        self.assertEqual(r["ext_osc_offset_mean_ppb"], "2.00000")
        self.assertEqual(r["smeas_ext0_freq_unc_ppb"], "3.0")

    def test_discsrc_excursion_inside_a_minute_is_counted(self):
        self.feed(30)
        self.feed(3, start_tow=130, **{"LEA.tos.DiscSrc": 0, "LEA.tos.extOscInLimit": 0})
        self.feed(30, start_tow=133)
        r = self.rows()[0][2]
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
        r = self.rows()[0][2]
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

    def test_gnss_source_logged_with_mean(self):
        self.feed(61, **{"LEA.smeas.gnss.freqOffset": -0.5, "LEA.smeas.gnss.freqUnc": 2.0,
                         "LEA.smeas.gnss.freqValid": 1})
        r = self.rows()[0][2]
        self.assertEqual(r["smeas_gnss_freq_offset_ppb"], "-0.5")
        self.assertEqual(r["smeas_gnss_freq_unc_ppb"], "2.0")
        self.assertEqual(r["smeas_gnss_freq_offset_mean_ppb"], "-0.50000")

    def test_appends_across_restarts_and_versions_share_one_file(self):
        # A v1 line already in the file (as after a version bump): kept, and
        # new lines are appended after it - the sit-health.csv convention.
        v1 = "LEAVIEW,1,2026-10-14T11:00:00+00:00," + ",".join(
            "" for _ in lea_view.LEA_VIEW_FIELDS_V1)
        with open(self.path, "w") as f:
            f.write(v1 + "\n")
        self.feed(61)
        log2 = lea_view.LeaViewLogger(self.path, 60, clock=self.clock, wallclock=self.wall)
        for i in range(61):
            log2.observe(snapshot(1000 + i))
            self.clock.t += 1
        self.assertEqual([v for _, v, _ in self.rows()], [1, 2, 2])
        self.assertEqual(os.listdir(self.tmp.name), ["lea-view.csv"])   # nothing renamed


class TestParse(unittest.TestCase):
    def test_rejects_wrong_tag_version_count_or_timestamp(self):
        good = "LEAVIEW,2,2026-10-14T12:00:00+00:00," + ",".join(
            "x" for _ in lea_view.LEA_VIEW_FIELDS_V2)
        self.assertIsNotNone(lea_view.parse_lea_view_line(good))
        self.assertIsNotNone(lea_view.parse_lea_view_line(good + "\r\n"))
        for bad in (good.replace("LEAVIEW", "HEALTH", 1),
                    good.replace(",2,", ",9,", 1),
                    good + ",extra",
                    good.replace("2026-10-14T12:00:00+00:00", "yesterday"),
                    "", "LEAVIEW"):
            self.assertIsNone(lea_view.parse_lea_view_line(bad), bad[:40])

    def test_field_lists_are_append_only(self):
        v1, v2 = lea_view.LEA_VIEW_FIELDS_V1, lea_view.LEA_VIEW_FIELDS_V2
        self.assertEqual(v2[:len(v1)], v1)
        self.assertEqual(lea_view.LEA_VIEW_FIELDS[lea_view.LEA_VIEW_LINE_VERSION], v2)


def _read(path):
    with open(path) as f:
        return f.read()


def _load_migrate():
    import importlib.util
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "migrate-lea-view.py")
    spec = importlib.util.spec_from_file_location("migrate_lea_view", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class TestMigrate(unittest.TestCase):
    """The 2026-10-14 one-off conversion of the headed files to LEAVIEW lines."""

    def setUp(self):
        self.m = _load_migrate()
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.m.ARCHIVE_DIR = os.path.join(self.tmp.name, "SiT-calib_archive")
        self.main = os.path.join(self.tmp.name, "lea-view.csv")
        self.aside = os.path.join(self.tmp.name, "lea-view.v1-20261014T145858Z.csv")
        n1, n2 = len(lea_view.LEA_VIEW_FIELDS_V1), len(lea_view.LEA_VIEW_FIELDS_V2)
        self.v1_rows = [f"1,2026-10-14T14:{m}:15Z," + ",".join(str(i) for i in range(n1))
                        for m in (40, 41)]
        self.v2_rows = [f"2,2026-10-14T14:{m}:58Z," + ",".join(str(i) for i in range(n2))
                        for m in (58, 59)]
        self.tagged = ("LEAVIEW,2,2026-10-14T15:30:00.123456+00:00,"
                       + ",".join(str(i) for i in range(n2)))
        with open(self.aside, "w") as f:
            f.write(self.m.LEGACY_HEADERS[1] + "\n" + "\n".join(self.v1_rows) + "\n")
        with open(self.main, "w") as f:
            f.write(self.m.LEGACY_HEADERS[2] + "\n" + "\n".join(self.v2_rows) + "\n"
                    + self.tagged + "\n")

    def run_main(self, *args):
        out = open(os.devnull, "w")
        self.addCleanup(out.close)
        stdout, sys.stdout = sys.stdout, out
        try:
            return self.m.main(["--file", self.main, "--no-wait", *args])
        finally:
            sys.stdout = stdout

    def test_dry_run_changes_nothing(self):
        before = _read(self.main)
        self.assertEqual(self.run_main(), 0)
        self.assertEqual(_read(self.main), before)
        self.assertTrue(os.path.exists(self.aside))

    def test_commit_merges_in_time_order_and_archives_originals(self):
        main_before, aside_before = _read(self.main), _read(self.aside)
        self.assertEqual(self.run_main("--commit"), 0)
        lines = _read(self.main).splitlines()
        self.assertEqual(len(lines), 5)
        parsed = [lea_view.parse_lea_view_line(l) for l in lines]
        self.assertTrue(all(parsed))
        self.assertEqual([p[1] for p in parsed], [1, 1, 2, 2, 2])
        self.assertEqual([p[0] for p in parsed], sorted(p[0] for p in parsed))
        self.assertEqual(lines[0].split(",")[2], "2026-10-14T14:40:15+00:00")
        self.assertEqual(lines[-1], self.tagged)               # kept byte-for-byte
        # originals archived unchanged, under <stem>_<stamp>.csv; nothing left beside
        arch = sorted(os.listdir(self.m.ARCHIVE_DIR))
        self.assertEqual(len(arch), 2)
        self.assertTrue(arch[0].startswith("lea-view-pre-tag_"))
        self.assertTrue(arch[1].startswith("lea-view-v1-pre-tag_"))
        self.assertEqual(_read(os.path.join(self.m.ARCHIVE_DIR, arch[0])), main_before)
        self.assertEqual(_read(os.path.join(self.m.ARCHIVE_DIR, arch[1])), aside_before)
        self.assertFalse(os.path.exists(self.aside))

    def test_second_run_is_a_no_op_on_content(self):
        self.run_main("--commit")
        after_first = _read(self.main)
        import time as _t
        _t.sleep(1.1)                                          # new archive stamp
        self.assertEqual(self.run_main("--commit"), 0)
        self.assertEqual(_read(self.main), after_first)

    def test_unknown_line_refuses_and_writes_nothing(self):
        with open(self.main, "a") as f:
            f.write("garbage,line\n")
        before = _read(self.main)
        with self.assertRaises(self.m.MigrationError):
            self.run_main("--commit")
        self.assertEqual(_read(self.main), before)
        self.assertFalse(os.path.exists(self.m.ARCHIVE_DIR))

    def test_wrong_field_count_refuses(self):
        with open(self.main, "a") as f:
            f.write("2,2026-10-14T15:00:00Z,1,2,3\n")
        with self.assertRaises(self.m.MigrationError):
            self.run_main("--commit")


if __name__ == "__main__":
    unittest.main()
