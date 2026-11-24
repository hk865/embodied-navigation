#!/usr/bin/env python3
"""实时录制、防穿墙多机位 bag 回放渲染及三画面合成的薄 CLI 入口。"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

from embodied_navigation.visualization.rviz_recorder import (  # noqa: E402
    CAMERA_MODE_CONFIGS,
    compose_three_view,
    run_bag_replay_record,
    run_live_record,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="DynaNav RViz2 防穿墙多机位录屏、bag 回放与合成工具")
    sub = parser.add_subparsers(dest="command", required=True)

    modes = sorted(CAMERA_MODE_CONFIGS)

    p_start = sub.add_parser("start", help="启动 RViz2、防穿墙相机节点、ffmpeg 抓屏与可选 rosbag 录制")
    p_start.add_argument("--run-dir", required=True, help="本次运行数据卷目录")
    p_start.add_argument("--ros-domain-id", type=int, required=True, help="与仿真一致的 ROS_DOMAIN_ID")
    p_start.add_argument("--camera-mode", choices=modes, default="spring_follow", help="机位模式（默认防穿墙第三人称）")
    p_start.add_argument("--rviz-config", default=None, help="自定义 RViz2 配置文件路径（覆盖 --camera-mode 默认配置）")
    p_start.add_argument("--display", default=os.environ.get("DISPLAY", ":0"), help="X11 DISPLAY（默认 :0）")
    p_start.add_argument("--width", type=int, default=1280, help="抓屏宽度（未自动定位窗口时的回退值）")
    p_start.add_argument("--height", type=int, default=720, help="抓屏高度")
    p_start.add_argument("--offset-x", type=int, default=0, help="抓屏左上角 X 偏移")
    p_start.add_argument("--offset-y", type=int, default=0, help="抓屏左上角 Y 偏移")
    p_start.add_argument("--no-auto-window", action="store_true", help="禁用 xwininfo 自动定位 RViz2 窗口")
    p_start.add_argument("--fps", type=int, default=20, help="抓屏帧率")
    p_start.add_argument("--rviz-warmup-s", type=float, default=4.0, help="启动 RViz2 后等待抓屏的秒数")
    p_start.add_argument("--max-duration-s", type=float, default=0.0, help="最长录制墙钟秒数（0 表示直到收到停止信号）")
    p_start.add_argument("--record-bag", action="store_true", help="同时录制核心 ROS 2 点云/位姿/图像/IMU bag")
    p_start.add_argument("--topics", nargs="+", default=None, help="自定义 rosbag 录制话题列表")
    p_start.add_argument("--odom-topic", default="/dynanav/backend/corrected_odometry")
    p_start.add_argument("--cloud-topic", default="/dynanav/backend/global_map")
    p_start.add_argument("--fixed-frame", default="map")
    p_start.add_argument("--arm-length-m", type=float, default=4.2, help="Spring-Arm 理想跟随臂长（米）")
    p_start.add_argument("--overhead-height-m", type=float, default=13.5, help="俯视切片相机高度（米）")
    p_start.add_argument("--cutaway-above-m", type=float, default=1.85, help="俯视模式切除机器人上方天花板阈值（米）")

    p_rep = sub.add_parser("replay", help="回放已录制 ROS 2 bag，以游戏防穿墙多机位渲染演示视频")
    p_rep.add_argument("--bag", required=True, help="ROS 2 bag 目录路径")
    p_rep.add_argument("--output-dir", required=True, help="输出 MP4 与日志目录（建议在数据卷）")
    p_rep.add_argument(
        "--views",
        nargs="+",
        choices=modes,
        default=["spring_follow", "overhead_cutaway", "isometric_follow"],
        help="待渲染的机位列表",
    )
    p_rep.add_argument("--ros-domain-id", type=int, default=83, help="离线回放使用的独立 ROS_DOMAIN_ID")
    p_rep.add_argument("--display", default=os.environ.get("DISPLAY", ":0"), help="X11 DISPLAY（默认 :0）")
    p_rep.add_argument("--rate", type=float, default=8.0, help="ros2 bag play 倍速（默认 8x）")
    p_rep.add_argument("--start-offset-s", type=float, default=0.0, help="bag 起点偏移秒数")
    p_rep.add_argument("--max-duration-s", type=float, default=0.0, help="每路机位最长录制墙钟秒数（0=直到回放结束）")
    p_rep.add_argument("--fps", type=int, default=20, help="录制帧率")
    p_rep.add_argument("--rviz-warmup-s", type=float, default=4.5, help="等待 RViz2 窗口就绪秒数")
    p_rep.add_argument("--odom-topic", default="/dynanav/backend/corrected_odometry")
    p_rep.add_argument("--cloud-topic", default="/dynanav/backend/global_map")
    p_rep.add_argument("--fixed-frame", default="map")
    p_rep.add_argument("--arm-length-m", type=float, default=4.2)
    p_rep.add_argument("--overhead-height-m", type=float, default=13.5)
    p_rep.add_argument("--cutaway-above-m", type=float, default=1.85)
    p_rep.add_argument("--replay-topics", nargs="+", default=None, help="自定义回放过滤话题")
    p_rep.add_argument("--compose-multiview", action="store_true", help="将多路机位合成为同屏多视角 MP4")

    p_comp = sub.add_parser("compose", help="将 overview+RGB 视频与 RViz 视频上下合成为三画面同屏 MP4")
    p_comp.add_argument("--overview-mp4", required=True, help="仿真自带的 overview+RGB 视频")
    p_comp.add_argument("--rviz-mp4", required=True, help="RViz 录制的点云视频")
    p_comp.add_argument("--output", required=True, help="输出三画面合成视频路径")
    p_comp.add_argument("--overview-offset-s", type=float, default=0.0)
    p_comp.add_argument("--rviz-offset-s", type=float, default=0.0)

    args = parser.parse_args(argv)
    if args.command == "start":
        return run_live_record(
            run_dir=Path(args.run_dir),
            ros_domain_id=args.ros_domain_id,
            camera_mode=args.camera_mode,
            rviz_config=args.rviz_config,
            display=args.display,
            width=args.width,
            height=args.height,
            offset_x=args.offset_x,
            offset_y=args.offset_y,
            fps=args.fps,
            rviz_warmup_s=args.rviz_warmup_s,
            max_duration_s=args.max_duration_s,
            record_bag=args.record_bag,
            topics=args.topics,
            odom_topic=args.odom_topic,
            cloud_topic=args.cloud_topic,
            fixed_frame=args.fixed_frame,
            arm_length_m=args.arm_length_m,
            overhead_height_m=args.overhead_height_m,
            cutaway_above_m=args.cutaway_above_m,
            auto_window=not args.no_auto_window,
        )
    if args.command == "replay":
        run_bag_replay_record(
            bag_dir=Path(args.bag),
            output_dir=Path(args.output_dir),
            views=list(args.views),
            ros_domain_id=args.ros_domain_id,
            display=args.display,
            rate=args.rate,
            start_offset_s=args.start_offset_s,
            max_duration_s=args.max_duration_s,
            fps=args.fps,
            rviz_warmup_s=args.rviz_warmup_s,
            odom_topic=args.odom_topic,
            cloud_topic=args.cloud_topic,
            fixed_frame=args.fixed_frame,
            arm_length_m=args.arm_length_m,
            overhead_height_m=args.overhead_height_m,
            cutaway_above_m=args.cutaway_above_m,
            replay_topics=args.replay_topics,
            compose_multiview=args.compose_multiview,
        )
        return 0
    return compose_three_view(
        overview_mp4=Path(args.overview_mp4),
        rviz_mp4=Path(args.rviz_mp4),
        output=Path(args.output),
        overview_offset_s=args.overview_offset_s,
        rviz_offset_s=args.rviz_offset_s,
    )


if __name__ == "__main__":
    raise SystemExit(main())
