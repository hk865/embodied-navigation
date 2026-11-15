# planning/pct

本目录是当前一代实测支撑网格到 vendored native PCT A* 的适配器；真正的 PCT 算法在 `../../../../third_party/pct`（vendored 源码），这里只是包装，不是自有实现。

## 边界

- 做：`measured_tomogram.py` 用调用方给出的实测 world/map 点构造有界多层支撑网格，并调用 `native_pct_binary()` 构建/定位 source-hashed 的 `pct_astar`。
- 不做：射线开凿支撑、补洞、场景资产加载、最近目标替换或直线 fallback；调用方负责精确 stamp 的点云/LIO 配对与 generation 重置。
- 不把产物当许可：它是 point-footprint 全局规划器，不是 clearance 或执行证书，不能当作 goto 门禁。

## 目录结构

```text
src/embodied_navigation/planning/pct/
├── README.md            # 本文件
└── measured_tomogram.py # 320 行：MeasuredPctPlanner 与 native_pct_binary
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码 | `measured_tomogram.py` | `MeasuredPctPlanner` 只填充实测 cell；`MeasuredPctError` 表示原生规划器建立、序列化或执行失败 |
| 代码 | `native_pct_binary()` | 按 `third_party/pct` 源码哈希在外部 cache（`PROJECT_DATA_ROOT` 下）构建 `pct_astar`，不写 checkout |
| 配置 | 无配置读取点 | 不读 `configs/`；路径与 build cache 由代码/环境决定 |

## 耦合、测试与限制

`measured_tomogram` 被 `simulation/dynanav/pct_scan_worker.py` 与 `planning/exploration/waypoint_candidates.py` 使用；`tests/unit/test_measured_pct_tomogram.py` 覆盖 `MeasuredPctPlanner` 与 `native_pct_binary`。原生构建与规划由 `python3 docker/verify_native_sources.py --compile-pct` 间接覆盖；本目录无专用端到端运行测试。
