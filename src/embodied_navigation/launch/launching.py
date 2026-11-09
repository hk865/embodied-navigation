"""共享的 ROS2 launch 机制；各模块文件自己维护组件清单。

本模块只提供两件事：把 `python -m` 形式的命令拼成 `ExecuteProcess` 能用的参数
列表（:func:`python`），以及为受管子进程登记退出事件、把退出码追加到
`<run_dir>/component_exit.jsonl` 并触发整体 Shutdown（:func:`owned`）。它不
决定进程清单、不构造环境、也不写 run 目录之外的文件。
"""

from pathlib import Path
import json
import os
from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

# 传给每个 ROS 节点的公共参数：仿真时间由 Gazebo 提供，节点不以系统墙钟为准。
ROS = ["--ros-args", "-p", "use_sim_time:=true"]


def python(module, *arguments, ros=True, executable="/usr/bin/python3"):
    """拼出以模块方式启动一个 Python 进程的命令行列表。

    `module` 是 `-m` 后面的模块名（如 `embodied_navigation.xxx`）；`arguments`
    逐项 `str()` 后追加，因此数值参数可直接传入。`-B` 禁止写 `.pyc`，避免在
    源码树里留下缓存。`ros=True` 时追加模块级 `ROS` 参数；`executable` 默认
    系统 `/usr/bin/python3`，需要别的解释器（例如带 CUDA 的 venv）时由调用方
    显式指定。返回的是参数列表，不启动进程。
    """
    return [str(executable), "-B", "-m", module, *map(str, arguments), *(ROS if ros else [])]


def owned(role, command, run, *, environment=None):
    """创建受管子进程，并在其退出时记录并触发整体关停。

    `role` 同时是进程名与退出记录里的角色名；`command` 是完整参数列表；
    `run` 是本轮 run 目录，用于追加 `component_exit.jsonl`；`environment` 叠加
    在父进程环境之上（`None` 表示不加）。`output="screen"` 表示子进程输出
    进入 launch 日志而不是被吞掉。

    返回 `[RegisterEventHandler(...), process]` 两个动作，交给调用方的
    `LaunchDescription` 依序展开：先登记 `OnProcessExit` 处理器，再启动进程。
    `sigterm_timeout="8"` 与 `sigkill_timeout="5"` 是 launch 的关停预算（秒，
    字符串形式）：先 SIGTERM 等 8 s，再 SIGKILL 等 5 s。任一组件退出都会追加
    一行 JSON（含 `role`/`pid`/`returncode`）并发出 `Shutdown`，让整棵 launch
    树一起停下；父进程 `launch/main.py` 的 cleanup 随后读取该文件汇总真实失败。
    """
    process = ExecuteProcess(
        cmd=command,
        name=role,
        output="screen",
        additional_env=environment or {},
        sigterm_timeout="8",
        sigkill_timeout="5",
    )

    def stopped(event, context):
        """组件退出回调：追加一行退出记录，并以"角色名 + 退出码"作为关停原因。"""
        # 追加模式：多个组件可能先后退出，行序即退出顺序。
        with (Path(run) / "component_exit.jsonl").open("a") as out:
            out.write(
                json.dumps(dict(role=role, pid=event.pid, returncode=event.returncode)) + "\n"
            )
        return [EmitEvent(event=Shutdown(reason=role + " exited: " + str(event.returncode)))]

    return [RegisterEventHandler(OnProcessExit(target_action=process, on_exit=stopped)), process]
