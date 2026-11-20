"""The raw-cloud sampling policy in the artifact sampler is bounded and covers the run."""

import ast
import math
from pathlib import Path

SAMPLER = Path(
    "/home/hyh001/projects/1.Project/embodied-navigation/.dsh-subagents/"
    "dsh-full-building-exploration-20261110/livo_yaw_sampler.py"
)


def load_policy():
    tree = ast.parse(SAMPLER.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == "should_save_raw":
            module = ast.Module(body=[node], type_ignores=[])
            namespace = {"math": math}
            exec(compile(module, str(SAMPLER), "exec"), namespace)
            return namespace["should_save_raw"]
    raise AssertionError("should_save_raw not found in the sampler artifact")


def simulate(stamps, *, interval_s=2.0, max_saves=64):
    should_save_raw = load_policy()
    saved, last, reasons = 0, None, []
    for stamp in stamps:
        save, reason = should_save_raw(
            stamp, last, saved, interval_s=interval_s, max_saves=max_saves
        )
        reasons.append(reason)
        if save:
            saved += 1
            last = stamp
    return saved, last, reasons


def test_the_policy_covers_the_action_window_and_the_cap():
    stamps = [round(0.1 * index, 6) for index in range(1201)]
    saved, last, reasons = simulate(stamps)
    assert last is not None and last >= 118.0
    assert saved <= 64
    windows = {int(last * 0) for _ in (0,)}
    saved_stamps = []
    should_save_raw = load_policy()
    count, last = 0, None
    for stamp in stamps:
        save, _ = should_save_raw(stamp, last, count, interval_s=2.0, max_saves=64)
        if save:
            count += 1
            last = stamp
            saved_stamps.append(stamp)
    assert any(39.0 <= s <= 40.0 for s in saved_stamps)
    assert any(59.0 <= s <= 60.5 for s in saved_stamps)
    assert any(79.0 <= s <= 80.5 for s in saved_stamps)
    capped, _, cap_reasons = simulate(stamps, max_saves=5)
    assert capped == 5
    assert cap_reasons[-1] == "cap"


def test_invalid_and_backward_stamps_never_save():
    should_save_raw = load_policy()
    assert should_save_raw(float("nan"), None, 0, interval_s=2.0, max_saves=8) == (False, "invalid_stamp")
    assert should_save_raw(-1.0, None, 0, interval_s=2.0, max_saves=8) == (False, "negative_stamp")
    assert should_save_raw(5.0, 6.0, 1, interval_s=2.0, max_saves=8) == (False, "rollback")
    assert should_save_raw(5.0, 4.5, 1, interval_s=2.0, max_saves=8) == (False, "too_soon")
