"""后端观测写盘；记录失败单独报告，不改变匹配、优化或纠正决策。"""
from __future__ import annotations

from dataclasses import asdict
import json
import math
from pathlib import Path

import numpy as np

from embodied_navigation.common.runtime_io import EvidenceWriter
from .pose_graph import invert


def json_values(value):
    """保存数组的当时数值；ICP的无穷分数用显式字符串表示。"""
    if isinstance(value, np.ndarray):
        return json_values(value.tolist())
    if isinstance(value, np.generic):
        return json_values(value.item())
    if isinstance(value, dict):
        return {str(k): json_values(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_values(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    return value


class BackendTrace:
    def __init__(self, directory: Path, config):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.writer = EvidenceWriter()
        self.sequence = 0
        self.error = None
        self.emit("configuration", config=asdict(config), edge_order="rho_phi")

    def emit(self, event_name, **fields):
        self.sequence += 1
        try:
            self.writer.append(self.directory / "events.jsonl", json_values({
                "schema": "slam_backend_trace/v1", "seq": self.sequence,
                "event": event_name, **fields,
            }))
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"

    def close(self):
        try:
            self.writer.close()
        except Exception as error:
            self.error = f"{type(error).__name__}: {error}"
        summary = {"submitted": self.sequence, "written": self.writer.completed,
                   "error": self.error or self.writer.error}
        (self.directory / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))
        return summary


def graph_snapshot(graph):
    """边内部存Z的逆；导出实际a→b测量，保留信息矩阵和实际鲁棒权重。"""
    edges = []
    for edge in graph.edges:
        a, b, inverse_measurement, information, robust = edge
        residual = graph._residual(graph.poses, edge)
        weight = graph._weight(residual, information, robust)
        edges.append({"a": graph.keys[a], "b": graph.keys[b],
                      "T_a_b": invert(inverse_measurement), "information": information,
                      "tangent_order": "rho_phi", "robust": robust,
                      "effective_weight": weight, "residual": residual})
    return {"keys": graph.keys.copy(), "poses": [p.copy() for p in graph.poses], "edges": edges}
