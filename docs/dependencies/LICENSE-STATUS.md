# 许可、署名与来源边界

本次只保留已有声明、复制已存在的许可文本和记录缺口，不替权利人新增或改变许可。下表不能代替法律审查。

| 组件 | 已保留证据 | 尚需注意 |
| --- | --- | --- |
| FAST-LIVO2 | 两个 package.xml 声明 GPLv2；原源码作者/商业许可联系头保留；源仓库 `LICENSES/FAST_LIVO2_GPL_2.0.txt` 复制到组件及两个包的 LICENSE | 原包内 LICENSE 缺失，本次补的是源仓库已有文本，不是新的权利授予；具体版本/与 Vikit 链接时的许可兼容性未解决 |
| SCAN 五包及消息 | package.xml 声明 Apache-2.0；原 `SCAN_PLANNER_APACHE_2.0.txt`、`SCAN_PLANNER_NOTICE.txt` 原样复制为 `third_party/scan/LICENSE`、`NOTICE` | 保留 ROS 2 移植和原研究作者署名；未独立查证更早上游 commit |
| PCT CLI/A* | 原 GPL-2.0 LICENSE 和 NOTICE 保留；NOTICE 明确“version 2 or any later version” | 只提供 CLI 实际闭包；NOTICE 中提及完整上游的 GTSAM/OSQP/pybind11，不代表这些组件被复制 |
| Vikit common/ROS | package.xml 声明 GPLv3；源文件作者信息保留 | 源仓库没有随附 GPLv3 LICENSE/COPYING；现按其package.xml声明附上系统标准GPLv3全文（third_party/vikit/COPYING）。这不新增作者授权；公开再分发前仍需核实准确上游来源及与FAST-LIVO2组合的许可 |
| Livox ROS Driver2 / RapidJSON | 完整 LICENSE.txt 和 RapidJSON 的 license.txt 保留，package.xml 声明 MIT | 保留完整文件中的多组件许可说明，不以简短 MIT 标签覆盖它们 |
| FastDDS / Asio / bundled notices | 原 Apache-2.0、Boost-1.0、Asio、filewatch MIT 文本、来源固定值和本地补丁保留 | 构建下载的完整源码也必须保留各自附带声明；本次未下载重核 |
| Livox SDK2 | 固定公开 commit/URL/SHA256 在构建清单中 | 完整源码及其许可在构建下载后保留；本次没有下载，也未把驱动包许可证套用于 SDK |

来源统一固定到本次复制的源仓库提交 `5688f2868e241ca4a7cc80ba9f166f8afa84c727`。更早 FAST-LIVO2/Vikit/SCAN/PCT 的独立 commit 在现有清单里不完整，不能伪称已恢复精确上游历史。`MIGRATION.json` 的 `6401e5fe` 是更早发布基线，不应覆盖当前工作树版本。

自有 Python/集成代码、Student 权重目前没有统一明确的独立对外许可证。本次不能替用户选择 MIT/Apache。模型的已有来源说明是自主训练/导出的发布依据，不代表对所有第三方使用者自动授予任意再分发权利。

Go2 部署 USD 的转换结果没有在相邻目录发现许可文件，应补充其原网格、转换来源和相应授权说明。原 Building 场景还引用 `L1_4D_Lidar.dae`，旧工程已明确它的授权缺失、未随公开分支提供；这不能通过新增下载脚本就变成许可明确的素材。

因此，本次完成的是实体源码闭包和构建配方，不是宣告全部源码/资产的公开再分发许可已齐全。不会把整个工程以单一许可证覆盖第三方组件。
