# robot/unitree/robogauge：RoboGauge 45D 部署

本目录提供 RoboGauge MoE-CTS 的纯数值契约、45D 直接观测、CPU TorchScript 推理与可选真实权重 replay，不启动 ROS、Gazebo、Isaac 或硬件。

## 关键文件与接口

- `contract.py`：`observation(state64, command3, last_action12)` 返回 `(float32[45], provenance_dict)` 二元组（body 角速度 ×0.25、投影重力、command ×[0.5, 1, 1]、关节位置相对默认、关节速度 ×0.05、上一原始动作；策略侧 official FL/FR/RL/RR 顺序）；`go2hv_torque()` 复刻归档 Go2HV PD 与速度包络（20.2 / 23.4 Nm，13.5 / 30 rad/s）；`MODEL_SHA256` 与默认关节顺序冻结。
- `runtime.py`：`verify_model()` 校验模型 SHA256；`Runtime` 从 `data_root()/runtime_assets/models/robogauge_policy.pt` 加载 CPU TorchScript，单线程，预热 10 个内部历史帧后 `reset()`；`infer(obs)` 返回 12 维动作，`metadata()` 记录 45D、proprioception-only、无 terrain/COM 速度输入、无 COM 速度 PID。
- `cpu_fixture.py`：可选真实权重 CPU replay，对 30 帧与独立 JIT 对比并验证 reset 可复现；需要 torch，不启动仿真。

## 调用与依赖

实际 45D native 循环在 [simulation/gazebo/robogauge/native_loop.py](../../../simulation/gazebo/robogauge/native_loop.py)，基准编排在 [navigation/robogauge/benchmark.py](../../../navigation/robogauge/benchmark.py)；本目录只提供契约与运行时。权重不在 Git，由 `PROJECT_DATA_ROOT` 指向的数据目录提供；更完整的部署与权重说明见 [docs/robogauge/README.md](../../../../../docs/robogauge/README.md)。

## 与 247D 的区别

45D 是 RoboGauge 自己的直接观测接口（独立 scale 与关节顺序），不是 [policy/native](../policy/native/README.md) 247D 中间观测链的裁剪，也不含 Student 的 COM 速度 PID 或 terrain/camera 输入；两者不能当作同一策略输入或 actuator 契约。

## 测试与限制

[tests/regression/test_robogauge_contract.py](../../../../../tests/regression/test_robogauge_contract.py)（NumPy 契约）、[test_robogauge_native_loop.py](../../../../../tests/regression/test_robogauge_native_loop.py)（可选 torch + 假 socket）、[test_robogauge_benchmark.py](../../../../../tests/regression/test_robogauge_benchmark.py)、[test_native_safety_integration.py](../../../../../tests/regression/test_native_safety_integration.py) 覆盖。CPU fixture 不是物理或导航测试；仿真部署与训练环境不宣称等价。
