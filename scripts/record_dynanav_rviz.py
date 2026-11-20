#!/usr/bin/env python3
"""实时录制 DynaNav RViz2 3D 点云生长视频、ROS 2 bag 及三画面合成的薄 CLI 工具。"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_RVIZ_CONFIG = REPO_ROOT / "configs" / "visualization" / "dynanav_live_pointcloud.rviz"
DEFAULT_BAG_TOPICS = (
    "/dynanav/rtx_livo/cloud_registered_full",
    "/dynanav/rtx_livo/post_vio_odometry",
    "/dynanav/rtx_livo/aft_mapped_to_init",
    "/dynanav/lidar_rtx",
    "/dynanav/imu",
    "/dynanav/camera",
    "/tf",
    "/tf_static",
)


def _ros_env(ros_domain_id: int, display: str) -> dict[str, str]:
    """构造与 run_dynanav_native_exploration.sh 对齐的 ROS 2 与 X11 环境。"""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ros_domain_id)
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    env["DISPLAY"] = display
    if not env.get("XAUTHORITY"):
        gdm_auth = Path(f"/run/user/{os.getuid()}/gdm/Xauthority")
        if gdm_auth.is_file():
            env["XAUTHORITY"] = str(gdm_auth)
    return env


def _spawn_ros_shell(command: str, env: dict[str, str], log_path: Path) -> subprocess.Popen:
    """在已加载 ROS 2 环境的子进程组中启动命令，便于统一优雅退出。"""
    ros_setup = os.environ.get("ROS_SETUP_SCRIPT", "/opt/ros/jazzy/setup.bash")
    wrapped = f"set -eo pipefail; source '{ros_setup}'; exec {command}"
    log_fh = log_path.open("w", encoding="utf-8")
    proc = subprocess.Popen(
        ["bash", "-lc", wrapped],
        env=env,
        stdout=log_fh,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    proc._log_fh = log_fh  # type: ignore[attr-defined]
    return proc


def _graceful_stop(proc: subprocess.Popen | None, name: str, *, sig: int = signal.SIGINT, timeout_s: float = 12.0) -> None:
    """向子进程组发送温和信号，超时再升级 SIGTERM/SIGKILL，确保 MP4 与 rosbag 正常落盘。"""
    if proc is None:
        return
    try:
        if proc.poll() is None:
            os.killpg(proc.pid, sig)
            try:
                proc.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=3.0)
    except ProcessLookupError:
        pass
    finally:
        fh = getattr(proc, "_log_fh", None)
        if fh is not None:
            fh.close()
        print(f"[record_rviz] stopped {name} rc={proc.returncode}", flush=True)


def _cmd_start(args: argparse.Namespace) -> int:
    run_dir = Path(args.run_dir).resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    rviz_cfg = Path(args.rviz_config).resolve()
    if not rviz_cfg.is_file():
        raise SystemExit(f"RViz 配置文件不存在: {rviz_cfg}")
    if shutil.which("ffmpeg") is None:
        raise SystemExit("未找到 ffmpeg，无法录制视频")

    env = _ros_env(args.ros_domain_id, args.display)
    rviz_mp4 = run_dir / "rviz_pointcloud.mp4"
    bag_dir = run_dir / "rosbag_explore"

    # 1. 启动 RViz2 加载实时点云生长配置
    rviz_proc = _spawn_ros_shell(f"rviz2 -d '{rviz_cfg}'", env, run_dir / "rviz.log")
    bag_proc: subprocess.Popen | None = None
    ffmpeg_proc: subprocess.Popen | None = None

    stop_requested = False

    def _handle_signal(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        # 2. 若启用 rosbag，同时录制核心传感器与前端输出话题
        if args.record_bag:
            topics = " ".join(f"'{t}'" for t in (args.topics or DEFAULT_BAG_TOPICS))
            bag_proc = _spawn_ros_shell(
                f"ros2 bag record -o '{bag_dir}' {topics}",
                env,
                run_dir / "rosbag.log",
            )

        # 等待 RViz2 窗口完成初始化再开始抓屏
        deadline = time.monotonic() + float(args.rviz_warmup_s)
        while time.monotonic() < deadline and not stop_requested:
            if rviz_proc.poll() is not None:
                raise SystemExit(f"rviz2 提前退出 (rc={rviz_proc.returncode})，请检查 {run_dir / 'rviz.log'}")
            time.sleep(0.2)

        if not stop_requested:
            # 3. 用 ffmpeg x11grab 录制 RViz2 窗口区域（ultrafast 降低对仿真 CPU 的影响）
            ffmpeg_log = (run_dir / "rviz_ffmpeg.log").open("w", encoding="utf-8")
            ffmpeg_cmd = [
                "ffmpeg",
                "-hide_banner",
                "-loglevel",
                "warning",
                "-y",
                "-f",
                "x11grab",
                "-framerate",
                str(args.fps),
                "-video_size",
                f"{args.width}x{args.height}",
                "-i",
                f"{args.display}+{args.offset_x},{args.offset_y}",
                "-an",
                "-c:v",
                "libx264",
                "-threads",
                "2",
                "-preset",
                "ultrafast",
                "-crf",
                "22",
                "-pix_fmt",
                "yuv420p",
                "-movflags",
                "+faststart",
                str(rviz_mp4),
            ]
            ffmpeg_proc = subprocess.Popen(
                ffmpeg_cmd,
                env=env,
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=ffmpeg_log,
                start_new_session=True,
            )
            ffmpeg_proc._log_fh = ffmpeg_log  # type: ignore[attr-defined]
            print(
                f"[record_rviz] recording -> {rviz_mp4} (domain={args.ros_domain_id}, display={args.display}, bag={args.record_bag})",
                flush=True,
            )

        start_wall = time.monotonic()
        while not stop_requested:
            if args.max_duration_s > 0 and (time.monotonic() - start_wall) >= args.max_duration_s:
                break
            if rviz_proc.poll() is not None or (ffmpeg_proc is not None and ffmpeg_proc.poll() is not None):
                break
            time.sleep(0.25)
    finally:
        # 先优雅停 ffmpeg（写完 MP4 moov atom），再停 rosbag 与 rviz2
        if ffmpeg_proc is not None and ffmpeg_proc.poll() is None and ffmpeg_proc.stdin is not None:
            try:
                ffmpeg_proc.stdin.write(b"q\n")
                ffmpeg_proc.stdin.flush()
            except OSError:
                pass
        _graceful_stop(ffmpeg_proc, "ffmpeg", sig=signal.SIGINT, timeout_s=15.0)
        _graceful_stop(bag_proc, "rosbag", sig=signal.SIGINT, timeout_s=15.0)
        _graceful_stop(rviz_proc, "rviz2", sig=signal.SIGINT, timeout_s=10.0)

    return 0


def _cmd_compose(args: argparse.Namespace) -> int:
    """将仿真 overview+RGB 视频与 RViz 3D 点云视频上下合成为三画面同屏复盘视频。"""
    overview = Path(args.overview_mp4).resolve()
    rviz = Path(args.rviz_mp4).resolve()
    output = Path(args.output).resolve()
    if not overview.is_file():
        raise SystemExit(f"找不到 overview 视频: {overview}")
    if not rviz.is_file():
        raise SystemExit(f"找不到 RViz 视频: {rviz}")
    output.parent.mkdir(parents=True, exist_ok=True)
    # 上方将 1024x512 等比缩放到 1280x640，下方保持 1280x720，总分辨率 1280x1360（均为偶数）
    filter_complex = "[0:v]scale=1280:640:flags=lanczos,fps=15[top];[1:v]scale=1280:720:flags=lanczos,fps=15[bot];[top][bot]vstack=inputs=2[v]"
    cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-y",
        "-i",
        str(overview),
        "-i",
        str(rviz),
        "-filter_complex",
        filter_complex,
        "-map",
        "[v]",
        "-c:v",
        "libx264",
        "-preset",
        "veryfast",
        "-crf",
        "21",
        "-pix_fmt",
        "yuv420p",
        "-shortest",
        "-movflags",
        "+faststart",
        str(output),
    ]
    subprocess.run(cmd, check=True)
    print(f"[record_rviz] composed 3-view video -> {output}", flush=True)
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="DynaNav RViz2 实时点云录屏与三画面合成工具")
    sub = parser.add_subparsers(dest="command", required=True)

    p_start = sub.add_parser("start", help="启动 RViz2、ffmpeg 抓屏与可选 rosbag 录制")
    p_start.add_argument("--run-dir", required=True, help="本次运行数据卷目录")
    p_start.add_argument("--ros-domain-id", type=int, required=True, help="与仿真一致的 ROS_DOMAIN_ID")
    p_start.add_argument("--rviz-config", default=str(DEFAULT_RVIZ_CONFIG), help="RViz2 配置文件路径")
    p_start.add_argument("--display", default=os.environ.get("DISPLAY", ":0"), help="X11 DISPLAY（默认 :0）")
    p_start.add_argument("--width", type=int, default=1280, help="抓屏宽度")
    p_start.add_argument("--height", type=int, default=720, help="抓屏高度")
    p_start.add_argument("--offset-x", type=int, default=0, help="抓屏左上角 X 偏移")
    p_start.add_argument("--offset-y", type=int, default=0, help="抓屏左上角 Y 偏移")
    p_start.add_argument("--fps", type=int, default=15, help="抓屏帧率")
    p_start.add_argument("--rviz-warmup-s", type=float, default=4.0, help="启动 RViz2 后等待抓屏的秒数")
    p_start.add_argument("--max-duration-s", type=float, default=0.0, help="最长录制墙钟秒数（0 表示直到收到停止信号）")
    p_start.add_argument("--record-bag", action="store_true", help="同时录制核心 ROS 2 点云/位姿/图像/IMU bag")
    p_start.add_argument("--topics", nargs="+", default=None, help="自定义 rosbag 录制话题列表")

    p_comp = sub.add_parser("compose", help="将 overview+RGB 视频与 RViz 视频上下合成为三画面同屏 MP4")
    p_comp.add_argument("--overview-mp4", required=True, help="仿真自带的 multiwaypoint05_overview_and_rgb.mp4")
    p_comp.add_argument("--rviz-mp4", required=True, help="RViz 录制的 rviz_pointcloud.mp4")
    p_comp.add_argument("--output", required=True, help="输出三画面合成视频路径")

    args = parser.parse_args(argv)
    if args.command == "start":
        return _cmd_start(args)
    return _cmd_compose(args)


if __name__ == "__main__":
    raise SystemExit(main())
