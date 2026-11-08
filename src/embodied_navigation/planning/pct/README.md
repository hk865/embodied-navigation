# planning/pct

本目录是 PCT 参考路线生成器：用既有建筑 tomogram 与仓库根 install/pct 中的 `pct_astar` 生成 A* 路线，再转成 body/ground 3D 折线；真正的 PCT 算法在 `../../../../third_party/pct`，这里只是包装与生成。

## 边界

- 做：离线生成路线、把网格 A* 结果转成物理支撑高度与 body 折线、简化与重采样。
- 不做：在线重规划、SCAN 执行、运动许可。产物里 `execution_success` 恒为 `false`，生成成功不等于路线可执行。

## 关键文件

`generate_routes.py` 是模块级脚本（import 即执行，没有 `main()`）：

1. 读仓库根数据入口 assets/building 的 `building_tomogram_safe.npz` 与 `building_triangles.npz`，把 cost/ground/gateway 写成仓库根 build/pct_routes 的 `pct_map.bin`；ceiling 仅用于搜索前的净空屏蔽；
2. 对每个 case 生成 `<case>_query.txt`，调用仓库根 install/pct 中的 `pct_astar`，读回 `cells_layer_row_col`；
3. 用三角形求实际支撑面高度，输出仓库根 build/pct_routes 的 `<case>_route.json`（schema 为 native_PCT_Astar_physical_reference/v1）与 `PCT_route_summary.json`。

case 表写死在文件里：`stairs_short`、`single_floor_turn`、`multi_flight_floor1`、`combined_floor1_inside`、`multi_floor2`、`roof_connectivity_probe`。后处理是 45 mm RDP 简化加 ≤0.6 m 间距，保留端点。

## 依赖与产物用途

需要仓库根数据入口 assets/building 的数据与已构建的 `pct_astar`（构建见 `../../../../docs/dependencies/BUILD.md`）。产物格式与 `../../../../configs/robogauge/routes/` 下的路线 JSON 一致，供 Building/RoboGauge 准备阶段作为冻结路线消费。

## 测试

本目录无专用测试。PCT 的 CPU 编译与简单网格规划由已有入口 `python3 docker/verify_native_sources.py --compile-pct` 覆盖（在仓库根执行，只引用，本轮未执行）。

## 限制

仓库根的 install 仅有 backend，`pct_astar` 未构建，因此生成流程未在本轮运行；建筑数据在 Git 之外，`../../../../third_party/pct` 只包含实际调用的 A* CLI 闭包。
