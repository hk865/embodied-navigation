"""DynaNav RViz2 录制、多机位防穿模回放与多画面合成核心逻辑。"""

from __future__ import annotations

import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import time

REPO_ROOT = Path(__file__).resolve().parents[3]
VIZ_CONFIG_DIR = REPO_ROOT / "configs" / "visualization"

CAMERA_MODE_CONFIGS: dict[str, Path] = {
    "legacy": VIZ_CONFIG_DIR / "dynanav_live_pointcloud.rviz",
    "spring_follow": VIZ_CONFIG_DIR / "dynanav_spring_follow.rviz",
    "overhead_cutaway": VIZ_CONFIG_DIR / "dynanav_overhead_cutaway.rviz",
    "isometric_follow": VIZ_CONFIG_DIR / "dynanav_isometric_follow.rviz",
}

DEFAULT_BAG_TOPICS = (
    "/dynanav/backend/global_map",
    "/dynanav/backend/corrected_odometry",
    "/dynanav/rtx_livo/post_vio_odometry",
    "/dynanav/rtx_livo/aft_mapped_to_init",
    "/dynanav/lidar_rtx",
    "/dynanav/imu",
    "/dynanav/camera",
    "/tf",
    "/tf_static",
)

DEFAULT_REPLAY_TOPICS = (
    "/clock",
    "/tf",
    "/dynanav/backend/global_map",
    "/dynanav/backend/corrected_odometry",
    "/dynanav/rtx_livo/post_vio_odometry",
    "/dynanav/rtx_livo/path",
)


def ros_env(ros_domain_id: int, display: str) -> dict[str, str]:
    """构造与 run_dynanav_native_exploration.sh 对齐的 ROS 2 与 X11 环境。"""
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(ros_domain_id)
    env["RMW_IMPLEMENTATION"] = "rmw_fastrtps_cpp"
    env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    env["DISPLAY"] = display
    src_path = str(REPO_ROOT / "src")
    prev_py = env.get("PYTHONPATH", "")
    if src_path not in prev_py.split(":"):
        env["PYTHONPATH"] = f"{src_path}:{prev_py}" if prev_py else src_path
    if not env.get("XAUTHORITY"):
        gdm_auth = Path(f"/run/user/{os.getuid()}/gdm/Xauthority")
        if gdm_auth.is_file():
            env["XAUTHORITY"] = str(gdm_auth)
    return env


def spawn_ros_shell(command: str, env: dict[str, str], log_path: Path) -> subprocess.Popen:
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


def graceful_stop(
    proc: subprocess.Popen | None,
    name: str,
    *,
    sig: int = signal.SIGINT,
    timeout_s: float = 12.0,
) -> None:
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


def detect_rviz_window_geometry(env: dict[str, str]) -> tuple[int, int, int, int] | None:
    """用 xwininfo 自动定位 RViz2 窗口在 X11 屏幕上的精确 (x, y, width, height)。"""
    if shutil.which("xwininfo") is None:
        return None
    try:
        out = subprocess.check_output(
            ["xwininfo", "-root", "-tree"],
            env=env,
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=3.0,
        )
    except Exception:
        return None
    # 匹配形如: 0x3c00007 "dynanav_spring_follow.rviz* - RViz": ("rviz2" "rviz2")  1280x720+0+32  +0+32
    pattern = re.compile(r'RViz".*?\s+(\d+)x(\d+)\+(-?\d+)\+(-?\d+)\s+\+(-?\d+)\+(-?\d+)')
    best: tuple[int, int, int, int] | None = None
    best_area = 0
    for line in out.splitlines():
        if "RViz" not in line:
            continue
        m = pattern.search(line)
        if not m:
            continue
        w, h, abs_x, abs_y = int(m.group(1)), int(m.group(2)), int(m.group(5)), int(m.group(6))
        if w >= 320 and h >= 240 and w * h > best_area:
            best_area = w * h
            # 裁掉窗口标题栏、菜单/工具栏与底部状态栏，仅保留纯 3D 视口区域
            crop_top = 104 if h > 420 else 0
            crop_bot = 32 if h > 420 else 0
            crop_lr = 14 if w > 420 else 0
            vw = max(2, (w - 2 * crop_lr) - ((w - 2 * crop_lr) % 2))
            vh = max(2, (h - crop_top - crop_bot) - ((h - crop_top - crop_bot) % 2))
            best = (max(0, abs_x + crop_lr), max(0, abs_y + crop_top), vw, vh)
    return best


def _start_ffmpeg_x11grab(
    output_mp4: Path,
    *,
    display: str,
    width: int,
    height: int,
    offset_x: int,
    offset_y: int,
    fps: int,
    env: dict[str, str],
    log_path: Path,
) -> subprocess.Popen:
    ffmpeg_log = log_path.open("w", encoding="utf-8")
    w = max(2, width - (width % 2))
    h = max(2, height - (height % 2))
    ffmpeg_cmd = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-y",
        "-f",
        "x11grab",
        "-framerate",
        str(fps),
        "-video_size",
        f"{w}x{h}",
        "-i",
        f"{display}+{offset_x},{offset_y}",
        "-an",
        "-c:v",
        "libx264",
        "-threads",
        "2",
        "-preset",
        "ultrafast",
        "-crf",
        "21",
        "-pix_fmt",
        "yuv420p",
        "-movflags",
        "+faststart",
        str(output_mp4),
    ]
    proc = subprocess.Popen(
        ffmpeg_cmd,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.DEVNULL,
        stderr=ffmpeg_log,
        start_new_session=True,
    )
    proc._log_fh = ffmpeg_log  # type: ignore[attr-defined]
    return proc


def _stop_ffmpeg(proc: subprocess.Popen | None) -> None:
    if proc is not None and proc.poll() is None and proc.stdin is not None:
        try:
            proc.stdin.write(b"q\n")
            proc.stdin.flush()
        except OSError:
            pass
    graceful_stop(proc, "ffmpeg", sig=signal.SIGINT, timeout_s=15.0)


def resolve_rviz_config(camera_mode: str, explicit_config: str | None) -> Path:
    if explicit_config:
        return Path(explicit_config).resolve()
    if camera_mode not in CAMERA_MODE_CONFIGS:
        raise SystemExit(f"未知 camera_mode={camera_mode!r}，可选: {sorted(CAMERA_MODE_CONFIGS)}")
    return CAMERA_MODE_CONFIGS[camera_mode].resolve()


def _spring_arm_cmd(
    *,
    odom_topic: str,
    cloud_topic: str,
    fixed_frame: str,
    arm_length_m: float,
    overhead_height_m: float,
    cutaway_above_m: float,
) -> str:
    return (
        "python3 -B -m embodied_navigation.visualization.spring_arm_node "
        f"--odom-topic '{odom_topic}' --cloud-topic '{cloud_topic}' "
        f"--fixed-frame '{fixed_frame}' --arm-length-m {arm_length_m:.3f} "
        f"--overhead-height-m {overhead_height_m:.3f} --cutaway-above-m {cutaway_above_m:.3f}"
    )


def run_live_record(
    *,
    run_dir: Path,
    ros_domain_id: int,
    camera_mode: str,
    rviz_config: str | None,
    display: str,
    width: int,
    height: int,
    offset_x: int,
    offset_y: int,
    fps: int,
    rviz_warmup_s: float,
    max_duration_s: float,
    record_bag: bool,
    topics: list[str] | None,
    odom_topic: str,
    cloud_topic: str,
    fixed_frame: str,
    arm_length_m: float,
    overhead_height_m: float,
    cutaway_above_m: float,
    auto_window: bool,
) -> int:
    run_dir = run_dir.resolve()
    run_dir.mkdir(parents=True, exist_ok=True)
    rviz_cfg = resolve_rviz_config(camera_mode, rviz_config)
    if not rviz_cfg.is_file():
        raise SystemExit(f"RViz 配置文件不存在: {rviz_cfg}")
    if shutil.which("ffmpeg") is None:
        raise SystemExit("未找到 ffmpeg，无法录制视频")

    env = ros_env(ros_domain_id, display)
    rviz_mp4 = run_dir / "rviz_pointcloud.mp4"
    bag_dir = run_dir / "rosbag_explore"

    cam_proc: subprocess.Popen | None = None
    if camera_mode != "legacy":
        cam_proc = spawn_ros_shell(
            _spring_arm_cmd(
                odom_topic=odom_topic,
                cloud_topic=cloud_topic,
                fixed_frame=fixed_frame,
                arm_length_m=arm_length_m,
                overhead_height_m=overhead_height_m,
                cutaway_above_m=cutaway_above_m,
            ),
            env,
            run_dir / "spring_arm_camera.log",
        )

    rviz_proc = spawn_ros_shell(f"rviz2 -d '{rviz_cfg}'", env, run_dir / "rviz.log")
    bag_proc: subprocess.Popen | None = None
    ffmpeg_proc: subprocess.Popen | None = None
    stop_requested = False

    def _handle_signal(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    try:
        if record_bag:
            topic_str = " ".join(f"'{t}'" for t in (topics or DEFAULT_BAG_TOPICS))
            bag_proc = spawn_ros_shell(f"ros2 bag record -o '{bag_dir}' {topic_str}", env, run_dir / "rosbag.log")

        deadline = time.monotonic() + float(rviz_warmup_s)
        while time.monotonic() < deadline and not stop_requested:
            if rviz_proc.poll() is not None:
                raise SystemExit(f"rviz2 提前退出 (rc={rviz_proc.returncode})，请检查 {run_dir / 'rviz.log'}")
            time.sleep(0.2)

        if not stop_requested:
            gx, gy, gw, gh = offset_x, offset_y, width, height
            if auto_window:
                detected = detect_rviz_window_geometry(env)
                if detected is not None:
                    gx, gy, gw, gh = detected
            ffmpeg_proc = _start_ffmpeg_x11grab(
                rviz_mp4,
                display=display,
                width=gw,
                height=gh,
                offset_x=gx,
                offset_y=gy,
                fps=fps,
                env=env,
                log_path=run_dir / "rviz_ffmpeg.log",
            )
            print(
                f"[record_rviz] recording ({camera_mode}) -> {rviz_mp4} "
                f"(geom={gw}x{gh}+{gx}+{gy}, domain={ros_domain_id})",
                flush=True,
            )

        start_wall = time.monotonic()
        while not stop_requested:
            if max_duration_s > 0 and (time.monotonic() - start_wall) >= max_duration_s:
                break
            if rviz_proc.poll() is not None or (ffmpeg_proc is not None and ffmpeg_proc.poll() is not None):
                break
            time.sleep(0.25)
    finally:
        _stop_ffmpeg(ffmpeg_proc)
        graceful_stop(bag_proc, "rosbag", sig=signal.SIGINT, timeout_s=15.0)
        graceful_stop(rviz_proc, "rviz2", sig=signal.SIGINT, timeout_s=10.0)
        graceful_stop(cam_proc, "spring_arm_camera", sig=signal.SIGINT, timeout_s=6.0)
    return 0


def compose_three_view(
    *,
    overview_mp4: Path,
    rviz_mp4: Path,
    output: Path,
    overview_offset_s: float = 0.0,
    rviz_offset_s: float = 0.0,
) -> int:
    """将仿真 overview+RGB 视频与 RViz 3D 点云视频上下合成为三画面同屏复盘视频。"""
    overview = overview_mp4.resolve()
    rviz = rviz_mp4.resolve()
    out = output.resolve()
    if not overview.is_file():
        raise SystemExit(f"找不到 overview 视频: {overview}")
    if not rviz.is_file():
        raise SystemExit(f"找不到 RViz 视频: {rviz}")
    out.parent.mkdir(parents=True, exist_ok=True)
    filter_complex = (
        "[0:v]scale=1280:640:flags=lanczos,fps=15[top];"
        "[1:v]scale=1280:720:flags=lanczos,fps=15[bot];"
        "[top][bot]vstack=inputs=2[v]"
    )
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-y"]
    if overview_offset_s > 0:
        cmd.extend(["-ss", f"{overview_offset_s:.3f}"])
    cmd.extend(["-i", str(overview)])
    if rviz_offset_s > 0:
        cmd.extend(["-ss", f"{rviz_offset_s:.3f}"])
    cmd.extend(
        [
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
            str(out),
        ]
    )
    subprocess.run(cmd, check=True)
    print(f"[record_rviz] composed 3-view video -> {out}", flush=True)
    return 0


def compose_multiview_grid(view_videos: dict[str, Path], output: Path, *, fps: int = 20) -> Path:
    """将多机位回放 MP4 合成为广播级分屏视频（主画面 1280×720 + 右侧辅助机位）。"""
    ordered = [(name, p.resolve()) for name, p in view_videos.items() if p.is_file()]
    if len(ordered) < 2:
        raise SystemExit(f"多机位合成至少需要 2 路视频，实际仅找到 {len(ordered)} 路")
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "warning", "-y"]
    for _, path in ordered:
        cmd.extend(["-i", str(path)])

    if len(ordered) == 2:
        # 左主视图 1280x720 + 右辅助视图 640x720（总 1920x720）
        fc = (
            f"[0:v]scale=1280:720:flags=lanczos,fps={fps}[v0];"
            f"[1:v]scale=640:720:force_original_aspect_ratio=decrease,"
            f"pad=640:720:(ow-iw)/2:(oh-ih)/2:color=0x0f131a,fps={fps}[v1];"
            "[v0][v1]hstack=inputs=2[v]"
        )
    else:
        # 左主视图 1280x720 + 右上 640x360 + 右下 640x360（总 1920x720）
        fc = (
            f"[0:v]scale=1280:720:flags=lanczos,fps={fps}[v0];"
            f"[1:v]scale=640:360:force_original_aspect_ratio=decrease,"
            f"pad=640:360:(ow-iw)/2:(oh-ih)/2:color=0x0f131a,fps={fps}[v1];"
            f"[2:v]scale=640:360:force_original_aspect_ratio=decrease,"
            f"pad=640:360:(ow-iw)/2:(oh-ih)/2:color=0x0f131a,fps={fps}[v2];"
            "[v1][v2]vstack=inputs=2[right];"
            "[v0][right]hstack=inputs=2[v]"
        )

    cmd.extend(
        [
            "-filter_complex",
            fc,
            "-map",
            "[v]",
            "-c:v",
            "libx264",
            "-preset",
            "veryfast",
            "-crf",
            "20",
            "-pix_fmt",
            "yuv420p",
            "-shortest",
            "-movflags",
            "+faststart",
            str(output),
        ]
    )
    subprocess.run(cmd, check=True)
    print(f"[record_rviz] composed multiview ({[n for n, _ in ordered]}) -> {output}", flush=True)
    return output


def run_bag_replay_record(
    *,
    bag_dir: Path,
    output_dir: Path,
    views: list[str],
    ros_domain_id: int,
    display: str,
    rate: float,
    start_offset_s: float,
    max_duration_s: float,
    fps: int,
    rviz_warmup_s: float,
    odom_topic: str,
    cloud_topic: str,
    fixed_frame: str,
    arm_length_m: float,
    overhead_height_m: float,
    cutaway_above_m: float,
    replay_topics: list[str] | None,
    compose_multiview: bool,
) -> dict[str, Path]:
    """回放已录制 ROS 2 bag，依次以多套防穿墙机位清晰渲染 MP4，并可选合成多机位同屏视频。"""
    bag_path = bag_dir.resolve()
    if not bag_path.exists():
        raise SystemExit(f"找不到 rosbag 目录: {bag_path}")
    out_dir = output_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    if shutil.which("ffmpeg") is None:
        raise SystemExit("未找到 ffmpeg，无法录制视频")

    env = ros_env(ros_domain_id, display)
    topic_list = replay_topics if replay_topics else list(DEFAULT_REPLAY_TOPICS)
    topic_args = " ".join(f"'{t}'" for t in topic_list)
    recorded: dict[str, Path] = {}

    for idx, view in enumerate(views):
        rviz_cfg = resolve_rviz_config(view, None)
        if not rviz_cfg.is_file():
            raise SystemExit(f"RViz 配置文件不存在: {rviz_cfg}")
        mp4_path = out_dir / f"rviz_{view}.mp4"
        cam_proc: subprocess.Popen | None = None
        rviz_proc: subprocess.Popen | None = None
        bag_proc: subprocess.Popen | None = None
        ffmpeg_proc: subprocess.Popen | None = None

        try:
            if view != "legacy":
                cam_proc = spawn_ros_shell(
                    _spring_arm_cmd(
                        odom_topic=odom_topic,
                        cloud_topic=cloud_topic,
                        fixed_frame=fixed_frame,
                        arm_length_m=arm_length_m,
                        overhead_height_m=overhead_height_m,
                        cutaway_above_m=cutaway_above_m,
                    )
                    + " --ros-args -p use_sim_time:=true",
                    env,
                    out_dir / f"spring_arm_{view}.log",
                )
            rviz_proc = spawn_ros_shell(
                f"rviz2 -d '{rviz_cfg}' --ros-args -p use_sim_time:=true",
                env,
                out_dir / f"rviz_{view}.log",
            )

            deadline = time.monotonic() + float(rviz_warmup_s)
            while time.monotonic() < deadline:
                if rviz_proc.poll() is not None:
                    raise SystemExit(f"rviz2 ({view}) 提前退出 rc={rviz_proc.returncode}")
                time.sleep(0.2)

            geom = detect_rviz_window_geometry(env)
            gx, gy, gw, gh = geom if geom is not None else (0, 32, 1280, 720)
            ffmpeg_proc = _start_ffmpeg_x11grab(
                mp4_path,
                display=display,
                width=gw,
                height=gh,
                offset_x=gx,
                offset_y=gy,
                fps=fps,
                env=env,
                log_path=out_dir / f"ffmpeg_{view}.log",
            )

            offset_flag = f"--start-offset {start_offset_s:.3f} " if start_offset_s > 0.0 else ""
            bag_cmd = (
                f"ros2 bag play '{bag_path}' --rate {rate:.3f} {offset_flag}"
                f"--topics {topic_args}"
            )
            bag_proc = spawn_ros_shell(bag_cmd, env, out_dir / f"bag_play_{view}.log")
            print(
                f"[record_rviz] replay [{idx + 1}/{len(views)}] view={view} rate={rate}x "
                f"geom={gw}x{gh}+{gx}+{gy} -> {mp4_path}",
                flush=True,
            )

            t0 = time.monotonic()
            while bag_proc.poll() is None:
                if max_duration_s > 0.0 and (time.monotonic() - t0) >= max_duration_s:
                    break
                if rviz_proc.poll() is not None or ffmpeg_proc.poll() is not None:
                    break
                time.sleep(0.25)
            # 尾部留 0.8s 确保最后几帧渲染完毕
            time.sleep(0.8)
            recorded[view] = mp4_path
        finally:
            _stop_ffmpeg(ffmpeg_proc)
            graceful_stop(bag_proc, f"bag_play_{view}", sig=signal.SIGINT, timeout_s=8.0)
            graceful_stop(rviz_proc, f"rviz2_{view}", sig=signal.SIGINT, timeout_s=10.0)
            graceful_stop(cam_proc, f"spring_arm_{view}", sig=signal.SIGINT, timeout_s=6.0)
            time.sleep(0.6)

    if compose_multiview and len(recorded) >= 2:
        composed = compose_multiview_grid(recorded, out_dir / "rviz_multiview.mp4", fps=fps)
        recorded["multiview"] = composed
    return recorded
