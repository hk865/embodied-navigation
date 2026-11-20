"""开机主动热身迈步、停稳趋势门禁与物理级 IMU 噪声注入的 CPU 单元测试。"""

from __future__ import annotations

import contextlib
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from embodied_navigation.project import project_root
from embodied_navigation.simulation.dynanav import experiment
from embodied_navigation.simulation.dynanav.capture.imu_noise import (
    ImuNoiseConfig,
    ImuNoiseInjector,
)
from embodied_navigation.simulation.dynanav.capture.startup_gate import (
    PHASE_LAND_SETTLE,
    PHASE_READY_FOR_SLAM,
    PHASE_STOP_CONVERGE,
    PHASE_WARMUP_WALK,
    StartupWarmupConfig,
    StartupWarmupController,
    TrendStabilityConfig,
    evaluate_trend_stability,
)


def _euler_zyx_to_quat_wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    """将 ZYX 欧拉角转换为单位四元数 [w, x, y, z]。"""
    cr, sr = math.cos(0.5 * roll), math.sin(0.5 * roll)
    cp, sp = math.cos(0.5 * pitch), math.sin(0.5 * pitch)
    cy, sy = math.cos(0.5 * yaw), math.sin(0.5 * yaw)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return [w, x, y, z]


def _make_rows(
    count: int = 200,
    *,
    dt_s: float = 0.005,
    t0_s: float = 2.0,
    pos_fn=None,
    rpy_fn=None,
    contacts_fn=None,
    q_fn=None,
    qd_fn=None,
) -> list[dict]:
    """构造与 capture_go2_rgbd.py 完全兼容的合成状态行序列。"""
    rows: list[dict] = []
    for k in range(count):
        t = t0_s + k * dt_s
        pos = [0.0, 0.0, 0.30] if pos_fn is None else list(pos_fn(k, t))
        roll, pitch, yaw = (0.0, 0.0, 0.0) if rpy_fn is None else rpy_fn(k, t)
        contacts = [0, 1, 1, 1, 1] if contacts_fn is None else list(contacts_fn(k, t))
        q = [0.0] * 12 if q_fn is None else list(q_fn(k, t))
        qd = [0.0] * 12 if qd_fn is None else list(qd_fn(k, t))
        rows.append(
            {
                "time_s": t,
                "root_pose_wxyz": [pos, _euler_zyx_to_quat_wxyz(roll, pitch, yaw)],
                "contacts": contacts,
                "q": q,
                "qd": qd,
            }
        )
    return rows


class TrendStabilityTests(unittest.TestCase):
    """验证停稳趋势+白噪声门禁在五类典型合成信号下的行为。"""

    def test_stationary_with_zero_mean_white_noise_passes(self):
        rng = np.random.default_rng(1001)
        pos_noise = rng.normal(0.0, 0.0005, size=(200, 3))
        att_noise = rng.normal(0.0, 0.0004, size=(200, 3))
        q_noise = rng.normal(0.0, 0.001, size=(200, 12))
        qd_noise = rng.normal(0.0, 0.02, size=(200, 12))

        rows = _make_rows(
            200,
            pos_fn=lambda k, _t: [pos_noise[k, 0], pos_noise[k, 1], 0.30 + pos_noise[k, 2]],
            rpy_fn=lambda k, _t: (att_noise[k, 0], att_noise[k, 1], att_noise[k, 2]),
            q_fn=lambda k, _t: q_noise[k].tolist(),
            qd_fn=lambda k, _t: qd_noise[k].tolist(),
        )
        result = evaluate_trend_stability(rows)
        self.assertTrue(result.settled, msg=f"不应被拦截: {result.failed_checks}")
        self.assertEqual(result.failed_checks, ())
        # 所有导出指标均可直接 JSON 序列化
        encoded = json.dumps(result.to_dict())
        self.assertIn('"settled": true', encoded)

    def test_slow_monotonic_yaw_drift_is_blocked(self):
        # 0.9 deg/s ≈ 0.015708 rad/s，显著高于 max_att_slope_radps=0.003
        drift_rate = math.radians(0.9)
        rng = np.random.default_rng(2002)
        att_noise = rng.normal(0.0, 0.0001, size=(200, 3))
        rows = _make_rows(
            200,
            rpy_fn=lambda k, t: (
                att_noise[k, 0],
                att_noise[k, 1],
                drift_rate * (t - 2.0) + att_noise[k, 2],
            ),
        )
        result = evaluate_trend_stability(rows)
        self.assertFalse(result.settled)
        self.assertIn("att_slope", result.failed_checks)
        self.assertIn("half_window_att_diff", result.failed_checks)
        self.assertIn("att_direction_consistency", result.failed_checks)
        self.assertAlmostEqual(result.metrics["att_slope_max_radps"], drift_rate, delta=0.001)

    def test_zero_mean_high_frequency_oscillation_is_blocked(self):
        # 零均值高频剧烈晃动（±2 cm，15 Hz），线性斜率接近 0，但残差标准差必须拦死
        rows = _make_rows(
            200,
            pos_fn=lambda _k, t: [
                0.02 * math.sin(2.0 * math.pi * 15.0 * t),
                0.0,
                0.30,
            ],
            rpy_fn=lambda _k, t: (
                0.015 * math.cos(2.0 * math.pi * 15.0 * t),
                0.0,
                0.0,
            ),
        )
        result = evaluate_trend_stability(rows)
        self.assertFalse(result.settled)
        self.assertIn("pos_resid_std", result.failed_checks)
        self.assertIn("att_resid_std", result.failed_checks)

    def test_joint_motion_and_insufficient_foot_support_are_blocked(self):
        # 情况 A：关节持续运动（速度和斜率超限）
        rows_joint = _make_rows(
            200,
            q_fn=lambda _k, t: [0.10 * (t - 2.0)] + [0.0] * 11,
            qd_fn=lambda _k, _t: [0.35] + [0.0] * 11,
        )
        res_joint = evaluate_trend_stability(rows_joint)
        self.assertFalse(res_joint.settled)
        self.assertIn("joint_speed_p95", res_joint.failed_checks)
        self.assertIn("joint_slope", res_joint.failed_checks)

        # 情况 B：两脚悬空（只有 2 足着地）
        rows_support = _make_rows(
            200,
            contacts_fn=lambda k, _t: [0, 1, 1, 0, 0] if k < 20 else [0, 1, 1, 1, 1],
        )
        res_support = evaluate_trend_stability(rows_support)
        self.assertFalse(res_support.settled)
        self.assertIn("support_fraction", res_support.failed_checks)

        # 情况 C：躯干触地
        rows_body = _make_rows(
            200,
            contacts_fn=lambda k, _t: [1, 1, 1, 1, 1] if k == 50 else [0, 1, 1, 1, 1],
        )
        res_body = evaluate_trend_stability(rows_body)
        self.assertFalse(res_body.settled)
        self.assertIn("body_contact", res_body.failed_checks)

    def test_stationary_yaw_wrapping_across_pi_passes(self):
        # yaw 在 ±pi 边界附近做 ±0.0005 rad 的零均值微扰，必须经 np.unwrap 后通过
        rng = np.random.default_rng(3003)
        yaw_offsets = rng.normal(0.0, 0.0004, size=200)
        rows = _make_rows(
            200,
            rpy_fn=lambda k, _t: (
                0.0,
                0.0,
                math.pi - 0.0002 + float(yaw_offsets[k]),
            ),
        )
        result = evaluate_trend_stability(rows)
        self.assertTrue(result.settled, msg=f"±pi 环绕静止不应误报: {result.failed_checks}")
        self.assertLess(result.metrics["att_resid_std_max_rad"], 0.003)

    def test_insufficient_samples_and_deadband_protection(self):
        short_rows = _make_rows(50)
        res_short = evaluate_trend_stability(short_rows)
        self.assertFalse(res_short.settled)
        self.assertEqual(res_short.failed_checks, ("insufficient_samples",))

        # 单调极微弱量化漂移（总变化 0.0005 m < deadband 0.002 m），方向一致性不应误报
        rows_tiny = _make_rows(
            200,
            pos_fn=lambda k, _t: [0.0005 * (k / 199.0), 0.0, 0.30],
        )
        res_tiny = evaluate_trend_stability(rows_tiny)
        self.assertTrue(res_tiny.settled, msg=f"死区内极微变化不应误报: {res_tiny.failed_checks}")
        self.assertEqual(res_tiny.metrics["pos_direction_consistency"], 0.0)


class StartupWarmupControllerTests(unittest.TestCase):
    """验证开机热身四阶段状态机与超时诊断逻辑。"""

    def test_disabled_controller_is_immediately_ready(self):
        ctrl = StartupWarmupController(StartupWarmupConfig(enabled=False))
        self.assertEqual(ctrl.phase, PHASE_READY_FOR_SLAM)
        self.assertTrue(ctrl.ready_for_slam)
        self.assertTrue(ctrl.slam_sensors_enabled)
        cmd, sensors_on, event = ctrl.step(0.0, [])
        np.testing.assert_allclose(cmd, [0.0, 0.0, 0.0])
        self.assertTrue(sensors_on)
        self.assertIsNone(event)

    def test_four_phase_state_machine_and_timestamps(self):
        cfg = StartupWarmupConfig(
            enabled=True,
            settle_s=0.5,
            walk_vx_mps=0.35,
            walk_duration_s=1.5,
            max_stop_wait_s=5.0,
            gate_slam_sensors_until_settled=True,
            stability=TrendStabilityConfig(window_samples=20),
        )
        ctrl = StartupWarmupController(cfg)
        self.assertEqual(ctrl.phase, PHASE_LAND_SETTLE)
        self.assertFalse(ctrl.slam_sensors_enabled)

        rows: list[dict] = []
        seen_phases: list[str] = []
        events: list[dict] = []
        pos_x = 0.0
        for step_idx in range(300):
            t_s = step_idx * 0.02
            # 模拟行数据：迈步阶段位置前进、停步前 0.3 s 逐步停稳
            if 0.5 <= t_s < 2.0:
                pos_x += 0.35 * 0.02
                qd_val = 1.0
            elif 2.0 <= t_s < 2.3:
                pos_x += 0.05 * 0.02
                qd_val = 0.3
            else:
                qd_val = 0.0
            rows.append(
                {
                    "time_s": t_s,
                    "root_pose_wxyz": [[pos_x, 0.0, 0.3], [1.0, 0.0, 0.0, 0.0]],
                    "contacts": [0, 1, 1, 1, 1],
                    "q": [0.0] * 12,
                    "qd": [qd_val] + [0.0] * 11,
                }
            )
            cmd, sensors_on, ev = ctrl.step(t_s, rows)
            seen_phases.append(ctrl.phase)
            if ev is not None:
                events.append(ev)
            if ctrl.phase == PHASE_WARMUP_WALK:
                np.testing.assert_allclose(cmd, [0.35, 0.0, 0.0])
                self.assertFalse(sensors_on)
            elif ctrl.phase == PHASE_STOP_CONVERGE:
                np.testing.assert_allclose(cmd, [0.0, 0.0, 0.0])
                self.assertFalse(sensors_on)
            elif ctrl.phase == PHASE_READY_FOR_SLAM:
                np.testing.assert_allclose(cmd, [0.0, 0.0, 0.0])
                self.assertTrue(sensors_on)
                break

        self.assertEqual(
            [(e["from_phase"], e["to_phase"]) for e in events],
            [
                (PHASE_LAND_SETTLE, PHASE_WARMUP_WALK),
                (PHASE_WARMUP_WALK, PHASE_STOP_CONVERGE),
                (PHASE_STOP_CONVERGE, PHASE_READY_FOR_SLAM),
            ],
        )
        summary = ctrl.summary()
        self.assertAlmostEqual(summary["t_walk_start_s"], 0.5)
        self.assertAlmostEqual(summary["t_stop_start_s"], 2.0)
        self.assertGreaterEqual(summary["t_slam_start_s"], 2.6)
        self.assertGreater(summary["stop_converge_duration_s"], 0.5)
        self.assertFalse(summary["timed_out"])
        self.assertFalse(summary["settled_by_timeout"])
        self.assertTrue(summary["final_stability"]["settled"])
        # 确保 summary 可直接 JSON 序列化
        json.dumps(summary)

    def test_timeout_records_explicit_failure_diagnostics_without_faking_pass(self):
        cfg = StartupWarmupConfig(
            enabled=True,
            settle_s=0.1,
            walk_vx_mps=0.35,
            walk_duration_s=0.2,
            max_stop_wait_s=0.6,
            stability=TrendStabilityConfig(window_samples=10),
        )
        ctrl = StartupWarmupController(cfg)
        rows: list[dict] = []
        last_event = None
        for idx in range(60):
            t_s = idx * 0.02
            # 持续存在 0.9 deg/s 慢漂，永远无法通过停稳判据
            rows.append(
                {
                    "time_s": t_s,
                    "root_pose_wxyz": [[0.0, 0.0, 0.3], _euler_zyx_to_quat_wxyz(0.0, 0.0, math.radians(0.9) * t_s)],
                    "contacts": [0, 1, 1, 1, 1],
                    "q": [0.0] * 12,
                    "qd": [0.0] * 12,
                }
            )
            _cmd, _sensors_on, ev = ctrl.step(t_s, rows)
            if ev is not None:
                last_event = ev
            if ctrl.ready_for_slam:
                break

        self.assertTrue(ctrl.ready_for_slam)
        self.assertTrue(ctrl.timed_out)
        self.assertTrue(ctrl.settled_by_timeout)
        self.assertIsNotNone(last_event)
        self.assertTrue(last_event["timed_out"])
        self.assertFalse(last_event["stability"]["settled"])
        self.assertIn("att_slope", last_event["stability"]["failed_checks"])
        summary = ctrl.summary()
        self.assertEqual(summary["timeout_reason"], "max_stop_wait_exceeded")
        self.assertFalse(summary["final_stability"]["settled"])

    def test_strict_config_validation_rejects_bad_types_and_unknown_keys(self):
        with self.assertRaises(ValueError):
            StartupWarmupConfig.from_dict({"unknown_field": 1})
        with self.assertRaises(ValueError):
            StartupWarmupConfig.from_dict({"enabled": 1})
        with self.assertRaises(ValueError):
            StartupWarmupConfig.from_dict({"walk_vx_mps": True})
        with self.assertRaises(ValueError):
            StartupWarmupConfig.from_dict({"stability": {"window_samples": 3}})
        with self.assertRaises(ValueError):
            TrendStabilityConfig(max_direction_consistency=1.5)


class ImuNoiseInjectorTests(unittest.TestCase):
    """验证物理级 IMU 偏置、随机游走、白噪声与安装倾角注入。"""

    def test_bias_white_noise_statistics_determinism_and_immutability(self):
        cfg = ImuNoiseConfig(
            enabled=True,
            seed=20261121,
            gyro_bias_init_radps=(0.015, -0.010, 0.020),
            accel_bias_init_mps2=(0.10, -0.08, 0.10),
            gyro_noise_density_radps_sqrthz=0.0005,
            accel_noise_density_mps2_sqrthz=0.005,
            gyro_bias_rw_radps_sqrts=0.0,
            accel_bias_rw_mps2_sqrts=0.0,
        )
        inj_a = ImuNoiseInjector(cfg)
        inj_b = ImuNoiseInjector(cfg)
        inj_diff = ImuNoiseInjector(ImuNoiseConfig(enabled=True, seed=999))

        raw_accel = np.array([0.0, 0.0, 9.81], dtype=float)
        raw_gyro = np.array([0.0, 0.0, 0.0], dtype=float)
        raw_reading = {
            "time": 1.0,
            "linear_acceleration": raw_accel.copy(),
            "angular_velocity": raw_gyro.copy(),
            "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
        }

        gyros = []
        accels = []
        dt_s = 0.005
        for step in range(4000):
            reading_copy = {
                "time": 1.0 + step * dt_s,
                "linear_acceleration": raw_accel.copy(),
                "angular_velocity": raw_gyro.copy(),
                "orientation": np.array([1.0, 0.0, 0.0, 0.0]),
            }
            out_a, row_a = inj_a.apply(reading_copy, dt_s=dt_s)
            out_b, _ = inj_b.apply(reading_copy, dt_s=dt_s)
            out_diff, _ = inj_diff.apply(reading_copy, dt_s=dt_s)

            # 同 seed 完全一致，不同 seed 不一致
            np.testing.assert_allclose(out_a["angular_velocity"], out_b["angular_velocity"])
            np.testing.assert_allclose(out_a["linear_acceleration"], out_b["linear_acceleration"])
            self.assertFalse(np.allclose(out_a["angular_velocity"], out_diff["angular_velocity"]))
            # 绝不原地修改输入字典
            np.testing.assert_allclose(reading_copy["linear_acceleration"], raw_accel)
            np.testing.assert_allclose(reading_copy["angular_velocity"], raw_gyro)
            self.assertEqual(row_a["sample_index"], step)

            gyros.append(out_a["angular_velocity"])
            accels.append(out_a["linear_acceleration"] - raw_accel)

        gyros_arr = np.asarray(gyros)
        accels_arr = np.asarray(accels)
        expected_gyro_std = 0.0005 / math.sqrt(dt_s)
        expected_accel_std = 0.005 / math.sqrt(dt_s)
        np.testing.assert_allclose(gyros_arr.mean(axis=0), [0.015, -0.010, 0.020], atol=0.001)
        np.testing.assert_allclose(accels_arr.mean(axis=0), [0.10, -0.08, 0.10], atol=0.01)
        np.testing.assert_allclose(gyros_arr.std(axis=0), [expected_gyro_std] * 3, rtol=0.10)
        np.testing.assert_allclose(accels_arr.std(axis=0), [expected_accel_std] * 3, rtol=0.10)

        # 验证原始样本 dict 未受破坏
        self.assertEqual(raw_reading["time"], 1.0)
        np.testing.assert_allclose(raw_reading["linear_acceleration"], raw_accel)

    def test_bias_random_walk_variance_grows_with_time(self):
        # 仅开启 bias random walk，检查终末偏置相对初值的扩散统计
        dt_s = 0.01
        steps = 200  # T = 2.0 s
        rw_gyro = 0.002
        final_deltas = []
        for trial in range(120):
            inj = ImuNoiseInjector(
                ImuNoiseConfig(
                    enabled=True,
                    seed=10_000 + trial,
                    gyro_bias_init_radps=(0.0, 0.0, 0.0),
                    accel_bias_init_mps2=(0.0, 0.0, 0.0),
                    gyro_noise_density_radps_sqrthz=0.0,
                    accel_noise_density_mps2_sqrthz=0.0,
                    gyro_bias_rw_radps_sqrts=rw_gyro,
                    accel_bias_rw_mps2_sqrts=0.0,
                )
            )
            reading = {"time": 0.0, "linear_acceleration": [0.0, 0.0, 9.81], "angular_velocity": [0.0, 0.0, 0.0]}
            for _ in range(steps):
                _, row = inj.apply(reading, dt_s=dt_s)
            final_deltas.append(row["gyro_bias_radps"])
        empirical_std = np.std(np.asarray(final_deltas), axis=0)
        theoretical_std = rw_gyro * math.sqrt(steps * dt_s)
        np.testing.assert_allclose(empirical_std, [theoretical_std] * 3, rtol=0.25)

    def test_mount_tilt_rotation_and_config_validation(self):
        tilt = (0.02, -0.03, 0.05)
        inj = ImuNoiseInjector(
            ImuNoiseConfig(
                enabled=True,
                seed=1,
                gyro_bias_init_radps=(0.0, 0.0, 0.0),
                accel_bias_init_mps2=(0.0, 0.0, 0.0),
                gyro_noise_density_radps_sqrthz=0.0,
                accel_noise_density_mps2_sqrthz=0.0,
                gyro_bias_rw_radps_sqrts=0.0,
                accel_bias_rw_mps2_sqrts=0.0,
                mount_tilt_rpy_rad=tilt,
            )
        )
        reading = {
            "time": 0.5,
            "linear_acceleration": [0.0, 0.0, 9.81],
            "angular_velocity": [1.0, 0.0, 0.0],
        }
        corrupted, _ = inj.apply(reading, dt_s=0.005)
        expected_accel = inj.R_mount @ np.array([0.0, 0.0, 9.81])
        expected_gyro = inj.R_mount @ np.array([1.0, 0.0, 0.0])
        np.testing.assert_allclose(corrupted["linear_acceleration"], expected_accel, atol=1e-12)
        np.testing.assert_allclose(corrupted["angular_velocity"], expected_gyro, atol=1e-12)

        with self.assertRaises(ValueError):
            ImuNoiseConfig.from_dict({"bad_key": True})
        with self.assertRaises(ValueError):
            ImuNoiseConfig.from_dict({"seed": True})
        with self.assertRaises(ValueError):
            ImuNoiseConfig.from_dict({"gyro_bias_init_radps": [0.0, 0.0]})
        with self.assertRaises(ValueError):
            ImuNoiseConfig(gyro_noise_density_radps_sqrthz=-1.0)


class ConfigAndWiringTests(unittest.TestCase):
    """验证新增配置文件格式、experiment --check 透传及 ros_interface 门控开关。"""

    def test_repo_configs_are_valid_and_loadable(self):
        root = project_root()
        exp_json = root / "configs/simulation/dynanav/startup_walk_imu_bias_experiment.json"
        livo_yaml = root / "configs/simulation/dynanav/fastlivo_livo_noisy_imu.yaml"

        exp_data = json.loads(exp_json.read_text(encoding="utf-8"))
        warmup_cfg = StartupWarmupConfig.from_dict(exp_data["startup_warmup"])
        noise_cfg = ImuNoiseConfig.from_dict(exp_data["imu_noise"])
        self.assertTrue(warmup_cfg.enabled)
        self.assertTrue(warmup_cfg.gate_slam_sensors_until_settled)
        self.assertAlmostEqual(warmup_cfg.walk_vx_mps, 0.35)
        self.assertTrue(noise_cfg.enabled)

        yaml_data = yaml.safe_load(livo_yaml.read_text(encoding="utf-8"))["/**"]["ros__parameters"]
        self.assertIs(yaml_data["imu"]["ba_bg_est_en"], True)
        self.assertEqual(yaml_data["imu"]["init_max_gyro_norm"], 0.08)
        self.assertGreater(yaml_data["imu"]["gyr_cov"], 4.0e-8)

    def test_experiment_cli_merges_and_validates_experiment_json(self):
        root = project_root()
        exp_json = root / "configs/simulation/dynanav/startup_walk_imu_bias_experiment.json"
        with tempfile.TemporaryDirectory() as tmp:
            tmp_root = Path(tmp)
            scene = tmp_root / "scene.usd"
            scene.touch()
            robot = tmp_root / "runtime_assets/robot/go2_description/go2_description.usda"
            robot.parent.mkdir(parents=True)
            robot.touch()
            output = tmp_root / "capture"
            stream = io.StringIO()
            with patch.object(experiment, "data_root", return_value=tmp_root), patch(
                "embodied_navigation.robot.unitree.robogauge.runtime.verify_model"
            ), patch(
                "embodied_navigation.robot.unitree.robogauge.runtime.default_model_path",
                return_value=tmp_root / "model.pt",
            ), contextlib.redirect_stdout(stream):
                code = experiment.main(
                    [
                        "--output",
                        str(output),
                        "--scene",
                        str(scene),
                        "--check",
                        "--experiment",
                        str(exp_json),
                    ]
                )
            self.assertEqual(code, 0)
            profile = json.loads(stream.getvalue())["profile"]
            self.assertTrue(profile["startup_warmup"]["enabled"])
            self.assertTrue(profile["imu_noise"]["enabled"])
            self.assertIn("experiment_config_sha256", profile)

    def test_publish_imu_and_clock_respects_publish_imu_flag(self):
        from embodied_navigation.simulation.dynanav.capture.ros_interface import create_ros_interface

        class _Message:
            def __init__(self, **kwargs):
                self.header = types.SimpleNamespace(
                    stamp=types.SimpleNamespace(sec=0, nanosec=0), frame_id=""
                )
                self.linear_acceleration = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.angular_velocity = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
                self.orientation = types.SimpleNamespace(w=1.0, x=0.0, y=0.0, z=0.0)
                self.orientation_covariance = [0.0] * 9
                self.clock = types.SimpleNamespace(sec=0, nanosec=0)
                self.__dict__.update(kwargs)

        publishers: dict[str, list] = {}

        class _Publisher:
            def __init__(self, topic: str):
                self.topic_name = topic
                self.sent = []
                publishers[topic] = self.sent

            def publish(self, msg):
                self.sent.append(msg)

        node = types.SimpleNamespace(
            create_publisher=lambda _kind, topic, _qos: _Publisher(topic),
            create_subscription=lambda *args, **kwargs: None,
        )
        rclpy = types.ModuleType("rclpy")
        rclpy.init = lambda **kwargs: None
        rclpy.create_node = lambda _name: node
        rclpy.spin_once = lambda *args, **kwargs: None
        rclpy.shutdown = lambda: None
        modules = {
            "rclpy": rclpy,
            "rclpy.qos": types.SimpleNamespace(qos_profile_sensor_data=object()),
            "rclpy.signals": types.SimpleNamespace(SignalHandlerOptions=types.SimpleNamespace(NO=0)),
            "geometry_msgs.msg": types.SimpleNamespace(TwistStamped=_Message),
            "nav_msgs.msg": types.SimpleNamespace(Odometry=_Message),
            "rosgraph_msgs.msg": types.SimpleNamespace(Clock=_Message),
            "sensor_msgs.msg": types.SimpleNamespace(
                CameraInfo=_Message,
                Image=_Message,
                Imu=_Message,
                PointCloud2=_Message,
                PointField=_Message,
            ),
        }
        reading = {
            "time": 1.0,
            "linear_acceleration": [0.0, 0.0, 9.81],
            "angular_velocity": [0.0, 0.0, 0.0],
            "orientation": [1.0, 0.0, 0.0, 0.0],
        }
        with patch.dict(sys.modules, modules):
            ros = create_ros_interface({}, None, None)
            ros.publish_imu_and_clock(0.5, reading, False, publish_imu=False)
            self.assertEqual(len(publishers["/clock"]), 1)
            self.assertEqual(len(publishers["/dynanav/imu"]), 0)
            self.assertEqual(ros.published["imu"], 0)

            ros.publish_imu_and_clock(1.0, reading, False)
            self.assertEqual(len(publishers["/clock"]), 2)
            self.assertEqual(len(publishers["/dynanav/imu"]), 1)
            self.assertEqual(ros.published["imu"], 1)

    def test_control_loop_warmup_drives_request_and_gates_tracker_ready(self):
        from embodied_navigation.simulation.dynanav.capture.control import ControlLoop

        class _FakeTensor:
            def __init__(self, arr):
                self._arr = np.asarray(arr, dtype=float)

            def numpy(self):
                return self._arr

        class _FakeArt:
            def get_world_poses(self):
                return _FakeTensor([[0.0, 0.0, 0.3]]), _FakeTensor([[1.0, 0.0, 0.0, 0.0]])

            def get_velocities(self):
                return _FakeTensor([[0.0, 0.0, 0.0]]), _FakeTensor([[0.0, 0.0, 0.0]])

        class _FakeTracker:
            def __init__(self):
                self.ready_history = []
                self.phase = "idle"
                self.stage = "idle"
                self.arrived = False
                self.failure = None
                self.events = []
                self.terminal_reason = None

            def observe_native_guards(self, **_kwargs):
                return None

            def tick(self, *_args, **kwargs):
                self.ready_history.append(kwargs["ready"])
                return [0.25, 0.0, 0.0], {"status": "ok" if kwargs["ready"] else "hold"}

        class _FakePolicy:
            def __init__(self):
                self.qdefault = np.zeros(12)
                self.requests = []
                self.cmd = np.zeros(3)

            def step(self, _i, _state, request):
                self.requests.append(request.copy())
                self.cmd = request.copy()
                return np.zeros(12), np.zeros(45), 0.0, np.zeros(12)

        class _FakeReceipts:
            def __init__(self):
                self.result = {"freshness_hold_events": []}
                self.journals = []

            def journal(self, name, payload):
                self.journals.append((name, payload))

            def save(self, force=False):
                return None

        sim_t = [0.0]
        sim_mod = types.ModuleType("isaacsim.core.simulation_manager")
        sim_mod.SimulationManager = types.SimpleNamespace(
            get_simulation_time=lambda: sim_t[0]
        )
        obs_mod = types.ModuleType(
            "embodied_navigation.robot.unitree.policy.native.observation"
        )
        obs_mod.quaternion_rotation = lambda _q: np.eye(3)

        warmup_cfg = StartupWarmupConfig(
            enabled=True,
            settle_s=0.1,
            walk_vx_mps=0.35,
            walk_duration_s=0.1,
            max_stop_wait_s=2.0,
            stability=TrendStabilityConfig(window_samples=4),
        )
        ctrl_warmup = StartupWarmupController(warmup_cfg)
        tracker = _FakeTracker()
        policy = _FakePolicy()
        receipts = _FakeReceipts()
        loop = ControlLoop(
            experiment={"navigation_pose_source": "measured_livo_post_vio"},
            gt_diagnostic=False,
            interactive_control=True,
            robogauge=True,
            target_speed=1.0,
            indices=list(range(12)),
            target=np.zeros(12),
            tracker=tracker,
            policy=policy,
            receipts=receipts,
            latest_nav=[None],
            latest_odom=[None],
            startup_warmup=ctrl_warmup,
        )
        ros = types.SimpleNamespace(spin_once=lambda _t=0.0: None)
        rows = _make_rows(10, t0_s=0.0, dt_s=0.005)
        fake_modules = {
            "isaacsim": types.ModuleType("isaacsim"),
            "isaacsim.core": types.ModuleType("isaacsim.core"),
            "isaacsim.core.simulation_manager": sim_mod,
            "embodied_navigation.robot.unitree.policy.native.observation": obs_mod,
        }
        with patch.dict(sys.modules, fake_modules):
            # LAND_SETTLE -> WARMUP_WALK -> STOP_CONVERGE -> READY_FOR_SLAM
            for step_idx, t_val in enumerate((0.0, 0.1, 0.2, 0.22)):
                sim_t[0] = t_val
                loop.tick(
                    iteration=step_idx * 4,
                    q=np.zeros(12),
                    qd=np.zeros(12),
                    art=_FakeArt(),
                    rows=rows,
                    contact_state=[0, 1, 1, 1, 1],
                    previous_completed_tau=np.zeros(12),
                    ros=ros,
                    policy_frames=[],
                )

        # 未停稳前 tracker.ready 必须为 False，进入 READY_FOR_SLAM 后放行为 True
        self.assertEqual(tracker.ready_history, [False, False, False, True])
        # WARMUP_WALK 期间直接驱动 0.35 m/s，STOP_CONVERGE 归零，READY 后交还 tracker
        np.testing.assert_allclose(policy.requests[0], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(policy.requests[1], [0.35, 0.0, 0.0])
        np.testing.assert_allclose(policy.requests[2], [0.0, 0.0, 0.0])
        np.testing.assert_allclose(policy.requests[3], [0.25, 0.0, 0.0])
        lifecycle_events = [p for name, p in receipts.journals if name == "startup_lifecycle.jsonl"]
        self.assertEqual(
            [(e["from_phase"], e["to_phase"]) for e in lifecycle_events],
            [
                (PHASE_LAND_SETTLE, PHASE_WARMUP_WALK),
                (PHASE_WARMUP_WALK, PHASE_STOP_CONVERGE),
                (PHASE_STOP_CONVERGE, PHASE_READY_FOR_SLAM),
            ],
        )
        self.assertTrue(receipts.result["startup_warmup"]["final_stability"]["settled"])


if __name__ == "__main__":
    unittest.main()
