"""由操作者标记实际观测阶段，源时间来自当前运行的SLAM观测。"""
import argparse
import json
from pathlib import Path
import time


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--event", choices=["start", "end", "note"], required=True)
    parser.add_argument("--note", default="")
    args = parser.parse_args(argv)
    observation = json.loads((args.run_dir / "capture/interactive/observation.json").read_text())
    if observation["source_ns"] is None:
        raise ValueError("当前观测没有SLAM源时间；结束后的阶段应使用已保存时间作离线标记")
    row = {"stage": args.stage, "event": args.event, "note": args.note,
           "stamp_ns": observation["source_ns"], "sim_s": observation["current_sim_s"],
           "active_seq": observation["waypoints"]["active_seq"],
           "pose": observation["pose"], "recorded_unix_ns": time.time_ns(),
           "time_source": "current observation source_ns; never simulator truth"}
    directory = args.run_dir / "stages"
    directory.mkdir(exist_ok=True)
    with (directory / "events.jsonl").open("a") as stream:
        stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
    print(json.dumps(row, ensure_ascii=False))


if __name__ == "__main__":
    main()
