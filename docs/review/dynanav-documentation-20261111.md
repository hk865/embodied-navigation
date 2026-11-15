# DynaNav 文档与可复用入口整理

日期：2026-11-11。范围限于说明、索引和已有skill的项目内归档；没有实现新规划器，没有改变传感/控制/物理代码，也没有启动或停止仿真。

## 基线与保留内容

- 项目及祖先目录未发现适用的`AGENTS.md`，项目原来没有`.agents/skills/`。
- 开始时HEAD `15d97fb6647c3349a5a02382a9bf96a8494ce7db`，分支`improve/module-docs-readability`；已有30个tracked修改、85个untracked条目。
- 全部现有业务代码、配置、第三方补丁、实验文档与原始数据保持原样，不stage、commit、push或回滚。
- 只新增当前状态文档，并在原索引前补入口/历史快照声明，不重写旧实验结论。

## 修改清单

1. `docs/dynanav-exploration.md`：按功能说明当前技术架构、模块选型、frontier实际职责、节点话题、QoS、线程/cache、源/墙钟时基、地图/物理边界及操作入口。
2. `docs/experiments/dynanav-native-exploration-32-waypoints-20261111.md`：补齐最新32目标run的执行回执、stop/parking/exit、26个body-contact样本、地图限制、视频、速度/决策来源验证的缺口。
3. README.md、README_zh.md、docs/README.md、docs/architecture.md、simulation/dynanav/README.md：增加新入口并区分旧快照与当前状态。
4. `.agents/skills/dynanav-explore/`：归档原skill的SKILL、参考SOP、agent元数据和三个脚本；`archive-manifest.json`记来源和六文件SHA。个人安装副本不变，不会自动双向同步。
5. 本记录：说明改动和验证范围。

源代码和可复用文档留项目；云、grid、PNG、视频、日志、模型/场景和构建产物继续留`PROJECT_DATA_ROOT`，未搬进仓库。架构不是把本机运行资产包装成单克隆即复现的承诺。

## 最小必要验证

- 对修改/新增文档和skill文档的本地相对链接核对目标存在。
- 六份归档源文件逐一SHA比对，排除pycache和运行产物。
- 对归档三个Python脚本作AST语法检查，不import ROS/Isaac，不发命令。
- 对归档SOP和3D renderer执行`--help`；collector在已装ROS环境中执行`--help`。这些help均在启动节点/写数据之前退出。
- 核对52份最新run source_manifest与业务源码/配置一致，检查32/32/stop33回执、物理接触区间、视频ffprobe和既有测试日志。
- `git diff --check`检查本轮tracked文档；新文件另查行尾空白/冲突标记。核对本轮写入目标限于文档/skill；52份业务源码与最新run快照仍一致，但这不是整理前全工作树的逐文件哈希审计。

这是文档/归档整理，不重跑1053/108全量测试或Isaac；既有测试结果明确注明是历史运行保存的日志。没有声称对每次模型选点来源、全视频逐帧、地图倾斜根因、完整覆盖或端到端时延作本轮独立验证。

## 实际检查结果

- 六文件源/归档/manifest SHA：6/6通过；三个Python脚本AST解析通过。
- SOP、3D renderer、ROS collector的`--help`均退出0；没有启动节点或仿真，没有写目标请求。
- 10份Markdown中的129个本地相对链接目标均存在；新文件无行尾空白/冲突标记，五份tracked文档的`git diff --check`通过。
- 最新run的52个source_manifest条目与当前业务源码/配置SHA一致；32/32/stop33、26个contact样本及视频元数据已复核。
- 无commit/push，无全量测试/新仿真。业务工作树仍脏，不能把文档完成当作所有代码已经版本化。
