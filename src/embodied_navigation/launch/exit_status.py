"""汇总受管进程的真实失败，包括 ROS launch 的子组件。

纯函数模块：不启动进程、不读文件、不写日志，只把调用方已收集到的退出码整理
成失败清单。父进程 `ros2 launch` 返回 0 并不代表整棵进程树成功，因此必须把
`component_exit.jsonl` 里的子组件退出码一并纳入判断。
"""


def failed_exits(process_exit_codes, component_exits):
    """返回所有非零退出的 `role`/`returncode` 记录。

    `process_exit_codes` 是调用方直接管理的子进程 `{role: code}`，`code` 来自
    `Popen.poll()`：仍在运行时为 `None`，被信号终止时为负值（例如 SIGINT 对应
    `-2`）。`component_exits` 是 ROS launch 树写出的记录列表，每项至少含
    `role` 与 `returncode`。

    只有 `code == 0` 算成功；`None`（未结束）与负值（信号退出）都算失败，因为
    关停阶段结束后它们同样表示没有干净收尾。两个来源按各自顺序拼接且不去重：
    同一 `role` 若同时出现在两处会各报告一次。返回新列表，元素形如
    `{"role": ..., "returncode": ...}`，不修改入参。
    """
    failures = [
        dict(role=role, returncode=code) for role, code in process_exit_codes.items() if code != 0
    ]
    failures.extend(
        dict(role=row["role"], returncode=row["returncode"])
        for row in component_exits
        if row["returncode"] != 0
    )
    return failures
