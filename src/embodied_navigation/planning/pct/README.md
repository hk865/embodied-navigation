# planning/pct

本目录是 PCT 参考路线生成器：用既有建筑 tomogram 与仓库根 `install/pct` 中的 `pct_astar` 生成 A* 路线，再转成 body/ground 3D 折线；真正的 PCT 算法在 `../../../../third_party/pct`（vendored 源码），这里只是包装与生成，不是自有实现。

## 边界

- 做：离线生成路线、把网格 A* 结果转成物理支撑高度与 body 折线、简化与重采样。
- 不做：在线重规划、SCAN 执行、运动许可。产物里 `execution_success` 恒为 `false`，生成成功不等于路线可执行。
- 不做算法：A* 网格搜索在 `third_party/pct/native/a_star/`，本目录只调用其 CLI（`pct_cli.cpp` 提供 `main`）。

## 目录结构

```text
src/embodied_navigation/planning/pct/
├── README.md  # 本文件
└── generate_routes.py  # 158 行，模块级脚本：tomogram→pct_astar→route JSON
```

## 代码、日志与配置

| 类别 | 位置 | 含义 |
| --- | --- | --- |
| 代码（脚本，import 即执行） | `generate_routes.py` | **没有 `main()`**：模块顶部的 `P.mkdir`、两次 `np.load`、`cases` 循环与 `subprocess.run` 都在 import 时执行。不要在代码里 import 它，只能 `python3 -m`/直接跑文件 |
| 代码（辅助函数） | 同文件 `support()` / `node()` / `reduce()` | 分别做三角形求支撑高度、端点映射到 PCT 网格 `[layer,row,col]`、3D RDP 简化；三个函数都很短（≤11 行），但无 docstring |
| 日志/输出 | `build/pct_routes/pct_map.bin` | 二进制地图：`<iii (L,H,W)>` + `<d resolution>` + cost/ground/gateway 三个 `<f8` 数组；`cost` 已把"无支撑 / 净空 <0.7 m / 非有限"的 cell 置 `1e6` |
| 日志/输出 | `build/pct_routes/<case>_query.txt` | 单个 query 行：`layer row col` × start/end 六个整数，作为 `pct_astar` 的位置参数输入 |
| 日志/输出 | `build/pct_routes/<case>_native_path.json` | `pct_astar` 的原始输出，读回 `found` 与 `cells_layer_row_col` |
| 日志/输出 | `build/pct_routes/<case>_native_search.log` | `pct_astar` 的 stdout+stderr 原样落盘（诊断用） |
| 日志/输出 | `build/pct_routes/<case>_route.json` | 最终冻结路线，schema `native_PCT_Astar_physical_reference/v1`，含 `world_ground_points`/`world_body_points`（body = ground + 0.32 m）、`length_m`、`layers_used`、`execution_success=false` |
| 日志/输出 | `build/pct_routes/PCT_route_summary.json` | 每个 case 的成功摘要或 `{"found": false, "error": ...}`；同时 `print()` 到 stdout |
| 日志/输出 | 本目录没有 run 目录产物 | 不写 `runs/`，不写日志文件；`build/` 是仓库根构建产物目录（不入 Git） |
| 配置 | **无配置读取点** | 不读 `configs/` 下任何文件；输入路径由 `embodied_navigation.project.project_root()` + 固定相对路径拼出，数据在 `assets/building/`（软链到外部数据卷） |

`cases` 表写死在文件里（6 个）：`stairs_short`、`single_floor_turn`、`multi_flight_floor1`、`combined_floor1_inside`、`multi_floor2`、`roof_connectivity_probe`；后处理是 45 mm RDP 简化加 ≤0.6 m 间距，保留端点。

## 模块内文件耦合

**方向结论**：只有一个文件，模块内无耦合。对外依赖两条：`embodied_navigation.project.project_root()`（仓库根定位）与外部可执行 `install/pct/pct_astar`。被依赖方向为空——全仓 `grep` 没有 `import generate_routes` 的命中（只有文档与本文件自己提到它）。

| 文件 | 行数 | 依赖谁 | 被谁依赖 | 判断 |
| --- | ---: | --- | --- | --- |
| `generate_routes.py` | 158 | `embodied_navigation.project.project_root`、numpy、subprocess、`assets/building/*.npz`、`install/pct/pct_astar` | 无（无 import 方） | 清楚但"脚本本质"（import 副作用） |

四类问题：

- **上帝文件**：无（158 行）。
- **重复实现**：无。
- **跨层直达**：单文件直接调外部二进制并读外部数据卷，这是生成脚本的本职，不算越层；但它绕过任何 `common/paths` 常量，路径是本地拼的。
- **隐式契约**：①`pct_astar` 的参数是**位置参数** `(map.bin, query.txt, out.json)`，顺序写死在 `subprocess.run` 里；②输出 JSON 键 `found`、`cells_layer_row_col` 是外部契约；③写死的 `cases` 表与 `node()` 的高度容差（0.22 m / 0.16 m）、`cost<=20` 门槛是隐式语义，改数据就要同步改；④同一 schema 还被 `configs/robogauge/routes/` 与 `assets/building/routes/` 的冻结副本使用（见下）。

结论：耦合**清楚**——单文件、无环、无内部依赖；风险不在耦合而在"import 即执行 + 写死路径/表"。

## 可读性评估

分级：**需改进**（文档与入口问题，不是结构混乱）。

- 证据：158 行、3 个顶层函数（`support`/`node`/`reduce`）、最长函数 11 行、函数级 docstring 比 0.0、注释 1 行；主流程是模块级 `for name, (start, end) in cases.items()`（第 95–157 行）加 `try/except` 汇总。
- 可读的部分：函数粒度小、命名直白（`support`/`node`/`reduce`），模块 docstring 写清了"只写 `build/pct_routes`、`execution_success` 恒 False"。
- 难读的部分：读文件时无法判断"哪部分是 import 副作用、哪部分是函数"；魔法数（0.7 m 净空、0.16 m 支撑匹配带、0.22 m 高度匹配、45 mm RDP、0.6 m 间距、0.32 m body 偏移）散在流程里；`node()` 的 `eligible` 过滤条件需要对照 PCT 的 `[layer,row,col]` 语义才能懂。

## 重构建议

没有 P0（生成物自带 `execution_success=false`，不会被误当许可）；P1 只有入口形态一项。

| 优先级 | 动作 | 理由 | 风险 | 验证方式 |
| --- | --- | --- | --- | --- |
| P1 | 把模块级执行流包进 `def main()`（`if __name__ == "__main__": main()`），只保留 import 与常量在模块级 | 消除 import 副作用：现在任何 `import` 都会 mkdir、读 npz、跑 6 个外部 CLI，测试/工具静态扫描都会被牵连 | 中：`subprocess.run` 与输出字节必须保持不变 | 先在能构建 `pct_astar` 的环境各跑一次，逐字节对比 `pct_map.bin`、6 个 `_route.json` 与 `PCT_route_summary.json`；本检出无 `install/pct`，只能做语法/静态检查 |
| P2 | 给 `support()`/`node()`/`reduce()` 补 docstring，并把 0.7/0.16/0.22/0.045/0.6/0.32 提成命名常量 | 这些阈值是几何语义，散落时无法核对来源 | 低 | 常量改名后跑一次生成，产物应逐字节相同 |
| P2 | 把输出目录 `build/pct_routes`、输入 `assets/building/*.npz` 改成显式 CLI 参数（默认值保持现状） | 现在只能在仓库根跑，且路径不可注入 | 低 | 默认参数不变时产物一致；新增参数只影响路径 |
| P2 | 把 `cases` 表与 `pct_astar` 的位置参数契约写进模块 docstring | 这是与外部 CLI 的隐式契约，新人只能读 `subprocess.run` 猜 | 低 | 与 `third_party/pct/pct_cli.cpp` 的 argv 解析对照 |

本轮只写建议、不改代码；P1 的验证依赖 `install/pct/pct_astar`，本检出未构建，故未执行。

## 依赖与产物用途

需要仓库根数据入口 `assets/building` 的数据与已构建的 `pct_astar`（构建见 [`docs/dependencies/BUILD.md`](../../../../docs/dependencies/BUILD.md)，产物路径 `install/pct/pct_astar`）。产物格式与 `configs/robogauge/routes/` 下的路线 JSON 一致（同 schema `native_PCT_Astar_physical_reference/v1`、同字段集），供 Building/RoboGauge 准备阶段作为冻结路线消费：

- `navigation/building_navigation.py` 读 run 目录的 `PCT_route.json`；
- `launch/building.py`（读 `assets/building/routes/<case>.json`）与 `simulation/gazebo/robogauge/world.py`（读 `configs/robogauge/routes/<case>.json`）各自写 run 目录的 `PCT_route.json`。两处 route 文件与 `generate_routes.py` 产物同 schema `native_PCT_Astar_physical_reference/v1`、同字段集。

因此本脚本的产物**不是运行时直接读取的文件**，而是生成/更新这些冻结副本的离线来源。

## 测试

本目录无专用测试。PCT 的 CPU 编译与简单网格规划由已有入口 `python3 docker/verify_native_sources.py --compile-pct` 覆盖（在仓库根执行，只引用，本轮未执行）。

## 限制

仓库根的 `install/` 仅有 `backend`，`pct_astar` 未构建（`ls install` 实测只有 `install/backend`），因此生成流程未在本轮运行；建筑数据在 Git 之外（`assets` 是软链），`../../../../third_party/pct` 只包含实际调用的 A* CLI 闭包（`pct_cli.cpp` + `native/a_star` + `native/common`），不含未使用的 Python 绑定与 OSQP 平滑模块。`node()` 的端点在数据不匹配时会直接抛 `ValueError` 并被 `try/except` 收进 summary，不会静默产出半条路线。
