# 完整扫描 producer：明确运行属性覆盖契约

## 首次 A 的真实结果

`rtx_async_ab_A_20261110T233405Z` 在 RTX sensor 构造期间被新配置认证保守拒绝：`elementsCoordsType` 与冻结资产比较不同。该 generation 没有开始 physics，没有首个 cloud/输入，没有动作；不是有效 A/B 性能样本。保留 capture.log 和只读取证，不把首次失败称为成功或覆盖旧证据。异常当时没有记录 actual repr/type，stage 已关闭，因此不能声称取得了该次运行时值。

根因是实现把既有、明确的生产运行覆盖值与资产默认直接比较，而不是配置被意外修改。原 `Lidar.create` 一直显式指定：

- outputFrameOfReference = SENSOR。
- outputMotionCompensationState = NONCOMPENSATED。
- elementsCoordsType = CARTESIAN。

冻结 USD 未 authored 这三项。无 sensor schema 的 CPU 读取显示 invalid/None；纯 CPU 注册安装版真实 schema 后，resolved fallback 依次为 SENSOR、NONCOMPENSATED、SPHERICAL，类型都是 USD token / Python str。安装版 `_sensor_base.py:288–304` 对传入属性直接 `GetAttribute(...).Set(value)`，因此 CARTESIAN 是既有合法输出格式覆盖，不能与 SPHERICAL fallback 等同。

## 修正边界

只改 `capture/lidar.py` 的 opt-in 构造认证和对应 unit test。默认关闭行为、原三个 `Lidar.create` setter、点坐标、时间、扫描证书和 native 代码均未改。

- 原资产 SHA256 仍固定 `ec5b732ef18521efe33ccdfc4e1242e9c3ce066e9482a91d29b3cea91fc7000b`，不改 USD，不用新hash掩盖差异。
- 三项运行覆盖无论 frozen_names 是否包含，都必须逐项为精确 Python str、上列固定文本和 resolved USD type `token`。缺项、其他文本、类型不符、timeSamples 均拒绝，不是跳过校验。
- 其他全部冻结 resolved sensor 属性与 emitter 集合/发射时间表继续精确比对；10Hz、32通道、accumulateOutputs 和完整扫描时间相位约束原样保留。
- 原 GMO SENSOR / NONCOMPENSATED / CARTESIAN 语义检查原样保留。没有把球坐标误当 xyz 的接受路径。
- 错误新增 actual/expected repr、Python type 和 USD type，后续失败可以审计具体差异。

## CPU 证据与限制

修订目录：`build_cache/dynanav_complete_scan_producer_contract_20261110T233710Z`。

最终 v2 focused tests 58 通过、py_compile 通过。host Python3.12.3 与真实 Isaac `python.sh` 的 Kit Python3.12.13＋ROS 环境均完成：真实 pxr、注册真实 schema、anonymous 内存 stage 克隆、三个固定 token 正例、逐项错误文本/blocked/timeSample、fireTimeNs+1 拒绝。原336127字节USD核对未变；没有启动 SimulationApp、Kit app、ROS节点或GPU。

同文本而 resolved USD type 为 string 的拒绝由 mock 用例覆盖。实际 pxr schema 强制该属性 resolved 为 token：attribute spec.typeName 无setter，SetInfo改变author spec后resolved仍为token。保留两次尝试日志，不把mock说成实际成功制造了错误类型。

71个 native/message/fixture文件与原冻结源逐字节相等，原 native core `f7c7cc0df9e3c69184f18c017a3681bf14831555a57afe39f73c96d63cab55f1` 未改、未重编。原6/6 native测试及22目标相同新初态/空白图片段证据继续引用旧manifest，不能说本次新编译通过。原prefix与首次A证据不动，新producer快照/combined manifest明确区分二者。

仍需唯一GPUowner在最终CPU聚合和独立review后开新的异步零动作generation，检验实际运行prim、认证流与source age/occupancy。此次修复本身不证明实时性改善，也不把无回波当free space。
