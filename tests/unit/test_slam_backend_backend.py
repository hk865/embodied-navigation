"""SLAM-BE end-to-end core (R-BE-*) through the frozen SlamBackend.ingest API.

Only frozen surface is used: BackendConfig() defaults, SlamBackend(config,
prior_map=...), ingest() -> dict, T_world_odom, save_map(), MapStore.load().
Frozen event-record keys used here: "error", and "loop" (bool) per the
supplementary ruling (answer b).

Ground truth: the mapping world frame is the body frame of the first mapping
keyframe (= the odom frame of the mapping run), as the rulings define.
"""
import tempfile
import unittest
from pathlib import Path

import numpy as np

import slam_backend_fixtures as F
from embodied_navigation.localization_mapping.backend.backend import BackendConfig, SlamBackend
from embodied_navigation.localization_mapping.backend.keyframes import Keyframe
from embodied_navigation.localization_mapping.backend.map_store import MapStore

SEC = 1_000_000_000


def _truthy(value):
    if isinstance(value, np.ndarray):
        return value.size > 0
    try:
        return bool(value)
    except Exception:  # pragma: no cover - exotic values count as present
        return True


def _mentions_loop(record):
    # Frozen key (supplementary ruling, answer b): "loop" is a bool.
    return record.get("loop") is True


def _has_error(record):
    return "error" in record and _truthy(record["error"])


def _body_error(C, T_odom_body, T_world_body_true):
    return float(np.linalg.norm((C @ T_odom_body)[:3, 3] - T_world_body_true[:3, 3]))


def _run(backend, rows):
    records, corrections = [], []
    for row in rows:
        record = backend.ingest(Keyframe(**row))
        records.append(record)
        C = backend.T_world_odom
        corrections.append(None if C is None else np.array(C, dtype=float, copy=True))
    return records, corrections


class _Mapping:
    """The R-BE-1 mapping run, executed once and shared (R-BE-2/3 reuse its saved map)."""
    _result = None

    @classmethod
    def get(cls):
        if cls._result is None:
            loop = F.closed_loop_with_drift(drift_m=1.5)
            rows = F.keyframe_stream(loop["scene"], loop["odom"], epoch=1)
            tmp = tempfile.TemporaryDirectory()
            map_dir = Path(tmp.name) / "map"
            try:
                backend = SlamBackend(BackendConfig())
                records, corrections = _run(backend, rows)
                backend.save_map(map_dir)
                cls._result = dict(loop=loop, rows=rows, records=records, corrections=corrections,
                                   map_dir=map_dir, tmp=tmp, error=None)
            except Exception as error:  # surface the same failure in every dependent test
                cls._result = dict(error=error, tmp=tmp)
        if cls._result["error"] is not None:
            raise cls._result["error"]
        return cls._result

    @classmethod
    def cleanup(cls):
        if cls._result is not None:
            cls._result["tmp"].cleanup()
            cls._result = None


def tearDownModule():
    _Mapping.cleanup()


class SlamBackendMappingTests(unittest.TestCase):
    def test_r_be_1_loop_closes_1p5m_drift(self):
        # Catches: a loop-closure basin limited to ~0.5 m (B-2) -- 1.5 m of drift is then never corrected.
        run = _Mapping.get()
        loop, records, corrections = run["loop"], run["records"], run["corrections"]
        n = len(loop["truth"])
        self.assertTrue(40 <= n <= 60, n)
        ret = loop["return_index"]
        raw = float(np.linalg.norm(loop["odom"][ret][:3, 3] - loop["truth"][ret][:3, 3]))
        self.assertAlmostEqual(raw, 1.5, places=6)  # fixture sanity
        self.assertTrue(all(isinstance(r, dict) for r in records))
        self.assertFalse([i for i, r in enumerate(records) if _has_error(r)], "clean frames reported errors")
        # first keyframe: world == odom, C = I
        self.assertIsNotNone(corrections[0])
        np.testing.assert_allclose(corrections[0], np.eye(4), atol=1e-9)
        self.assertTrue(any(_mentions_loop(r) for r in records), "no loop closure reported in any record")
        C = corrections[-1]
        self.assertIsNotNone(C)
        raw_end = _body_error(np.eye(4), loop["odom"][-1], loop["truth"][-1])
        self.assertGreater(raw_end, 0.8)  # fixture sanity: the end is still ~1.07 m off in raw odom
        self.assertLess(_body_error(C, loop["odom"][-1], loop["truth"][-1]), 0.3)

    def test_r_be_4_empty_cloud_reports_error_and_continues(self):
        # Catches: an exception inside one frame stopping (or poisoning) the backend.
        loop = F.closed_loop_with_drift(drift_m=0.0)
        rows = F.keyframe_stream(loop["scene"][:6], loop["odom"][:6], epoch=4)
        rows[3]["points_lidar"] = np.zeros((0, 3))
        backend = SlamBackend(BackendConfig())
        records = []
        for row in rows:
            records.append(backend.ingest(Keyframe(**row)))
        self.assertTrue(_has_error(records[3]), records[3])
        for i in (4, 5):
            self.assertIsInstance(records[i], dict)
            self.assertFalse(_has_error(records[i]), records[i])
        self.assertIsNotNone(backend.T_world_odom)
        self.assertLess(_body_error(backend.T_world_odom, loop["odom"][5], loop["truth"][5]), 0.3)


class SlamBackendRelocalizationTests(unittest.TestCase):
    YAW_OFFSET = 1.2
    START_ARC_M = 5.0

    def _relocalize(self, frames, jump_from=None, jump=None):
        mapping = _Mapping.get()
        prior = MapStore.load(mapping["map_dir"])
        reloc = F.relocalization_run(self.START_ARC_M, self.YAW_OFFSET, frames=frames)
        odom = [(jump @ T) if jump_from is not None and k >= jump_from else T
                for k, T in enumerate(reloc["odom"])]
        rows = F.keyframe_stream(reloc["scene"], odom, seed=F.SEED + 1000, epoch=2,
                                 stamp0_ns=500 * SEC)
        backend = SlamBackend(BackendConfig(), prior_map=prior)
        records, corrections = _run(backend, rows)
        errors = [None if C is None else _body_error(C, T_ob, T_wb)
                  for C, T_ob, T_wb in zip(corrections, odom, reloc["truth"])]
        return reloc, records, corrections, errors

    def test_r_be_2_global_relocalization_5m_1p2rad(self):
        # Catches: prior-map mode with only local tracking around an initial guess and no global search (B-3).
        reloc, records, corrections, errors = self._relocalize(10)
        C_true = reloc["T_world_odom_true"]
        self.assertAlmostEqual(float(np.linalg.norm(C_true[:3, 3])), 5.0, places=6)  # fixture sanity
        self.assertAlmostEqual(F.yaw_of(C_true), 1.2, places=6)
        self.assertFalse([i for i, r in enumerate(records) if _has_error(r)])
        self.assertIsNone(corrections[0], "adopted on a single candidate; two confirmations required")
        adopted = [k for k, C in enumerate(corrections) if C is not None]
        self.assertTrue(adopted, f"no correction adopted within 10 keyframes: {records}")
        first = adopted[0]
        self.assertLessEqual(first, 9)
        for k in range(first, 10):
            self.assertIsNotNone(corrections[k], f"correction lost at frame {k}")
            self.assertLess(errors[k], 0.2, f"frame {k}: body error {errors[k]:.3f} m")

    def test_r_be_3_tracking_loss_returns_to_global_search(self):
        # Catches: a relocalized backend that freezes forever after tracking is lost.
        jump_from, frames = 12, 40
        jump = F.pose(2.0, 0.0, 0.0)                       # odom frame jumps 2 m
        reloc, records, corrections, errors = self._relocalize(frames, jump_from, jump)
        before = [k for k in range(jump_from) if corrections[k] is not None]
        self.assertTrue(before, "precondition: relocalized before the odom jump")
        self.assertLess(errors[jump_from - 1], 0.2, "precondition: correct C before the jump")
        # the stale C is 2 m off right after the jump; at the end the correct C must be adopted again
        for k in (frames - 2, frames - 1):
            self.assertIsNotNone(corrections[k])
            self.assertLess(errors[k], 0.2, f"frame {k}: body error {errors[k]:.3f} m after the jump")
        recovered = [k for k in range(jump_from, frames) if errors[k] is not None and errors[k] < 0.2]
        self.assertTrue(recovered)


if __name__ == "__main__":
    unittest.main()
