"""统一的 RoboGauge 仿真安全决策；不导入任何仿真器。

职责：对调用方给出的单帧 `PhysicalSafetyState` 作出 fault / stop / reason 决策。
锁存顺序固定为：策略非有限值、执行器故障，随后才查询地形；倾倒/低离地判定保留
归档阈值与宽限时间（见 `evaluate` 内字面量）。本体接触只作诊断，不新增停车条件；
其他位置上已有的接触持续时长保护继续有效。

边界：本模块不做地形查询、不发布命令、不写盘。地形查询由调用方
（`../../robot/unitree/policy/native/worker.py` 与
`../../simulation/gazebo/robogauge/native_loop.py`）在 `latch_runtime_faults`
之后执行，因此查询抛异常时已锁存的故障仍然保留。这不是实机安全控制器，不接受
`REAL_SENSOR` 等实机来源；该类型在 `../../common/state_contract.py` 中仍为
保留接口。
"""

from dataclasses import dataclass
from embodied_navigation.common.state_contract import (
    GAZEBO_NATIVE64_SOURCE,
    GAZEBO_SIMULATION_CLOCK,
    PhysicalSafetyState,
    StateContractError,
    StatePurpose,
    validate_state_provenance,
)


@dataclass(frozen=True)
class SimulationSafetyResult:
    """单帧安全决策结果；调用方据 `stop` 与 `fault` 决定阻尼/停车。

    `fault`/`reason` 为稳定的故障码（None 表示无故障）；`stop` 为最终停车请求。
    四个 `observed_*`/`persistent_*` 字段是诊断位；`fall_started_s` 是持续倾倒
    起点，单位 s。`state_contract_*` 只在来源/时钟/测量契约被拒绝时置位。
    """

    fault: str | None
    stop: bool
    reason: str | None
    observed_fall_or_low: bool
    observed_body_contact: bool
    persistent_fall: bool
    persistent_fall_observation_end: bool
    fall_started_s: float | None
    state_contract_valid: bool = True
    state_contract_error: str | None = None

    @property
    def request_fault_damping(self):
        """任一 fault 即请求故障阻尼；不另设判据，由调用方消费。"""
        return self.fault is not None


class SimulationSafetyGate:
    """有状态的逐帧仿真安全门；每个 run 一个实例。

    `fault` 一旦锁存不会自行清除（跨帧保留），`fall_started` 记录持续倾倒起点。
    调用方每帧先 `latch_runtime_faults`，再查询地形，最后 `evaluate`；重复调用同一
    帧是幂等的。本类不发布命令、不修改调用方状态。
    """

    def __init__(self, *, run_id):
        """只接受非空字符串 run_id；该值用于拒绝跨 run 的安全状态。"""
        if not isinstance(run_id, str) or not run_id.strip():
            raise StateContractError("missing_run_id")
        self.run_id = run_id
        self.fault = None
        self.fall_started = None

    def latch_runtime_faults(self, *, policy_finite, actuator_fault_code, prior_fault=None):
        """查询地形之前的第一阶段锁存；保留原优先级。

        参数：`policy_finite` 表示动作与观测是否全为有限值；`actuator_fault_code`
        来自 native64 帧的索引 56（`> 0` 即故障，代码会转成整数写入故障码）；
        `prior_fault` 是调用方更早阶段已产生的故障。

        在 `evaluate` 中重复调用是幂等的。这条两阶段钩子的目的是：即使随后的
        射线/地形查询抛异常，已经锁存的策略/执行器故障也不会丢失。
        """
        # `or` 表示先锁存者优先，后续判定不会覆盖已锁存的故障码。
        self.fault = self.fault or prior_fault
        if not policy_finite:
            self.fault = self.fault or "nonfinite_policy"
        if actuator_fault_code > 0:
            self.fault = self.fault or f"actuator_fault_{int(actuator_fault_code)}"
        return self.fault

    def reject_state(self, error, *, prior_fault=None):
        """对新出现的来源/时钟/测量契约违例 fail closed。

        返回单帧结果：`stop=True`，`state_contract_valid=False` 且保留原始异常文本；
        故障码取 `prior_fault` 或默认的
        `simulation_safety_state_contract_rejected`。不更新倾倒计时，也不做地形查询。
        """
        self.fault = self.fault or prior_fault or "simulation_safety_state_contract_rejected"
        return SimulationSafetyResult(
            fault=self.fault,
            stop=True,
            reason=self.fault,
            observed_fall_or_low=False,
            observed_body_contact=False,
            persistent_fall=False,
            persistent_fall_observation_end=False,
            fall_started_s=self.fall_started,
            state_contract_valid=False,
            state_contract_error=str(error),
        )

    def evaluate(self, state, *, policy_finite=True, prior_fault=None):
        """对一帧安全状态作完整判定并返回 `SimulationSafetyResult`。

        先校验 typed 来源、时钟域、run_id 与测量时间戳，契约不满足立即
        `reject_state`（fail closed，不再读阈值）；通过后才锁存运行时故障并判定
        倾倒/离地。本体接触只置诊断位。所有阈值均为归档原值、硬编码，单位随字段
        （`roll_rad`/`pitch_rad` 为 rad，`body_clearance_m` 为 m，`elapsed_s` 为 s）。
        """
        if not isinstance(state, PhysicalSafetyState):
            return self.reject_state("missing_typed_physical_safety_state", prior_fault=prior_fault)
        # 来源/时钟/run 三重契约：只接受仿真真值来源与仿真时钟，跨 run 立即拒绝。
        try:
            validate_state_provenance(
                state.provenance,
                purpose=StatePurpose.SIMULATION_SAFETY,
                expected_source=GAZEBO_NATIVE64_SOURCE,
                expected_clock_domain=GAZEBO_SIMULATION_CLOCK,
                expected_run_id=self.run_id,
            )
            state.validate_measurements()
            # 帧时间与采样时间必须与 provenance 中的整数时间戳严格一致；原生
            # preupdate 采样固定在帧前 0.005 s（5 ms，硬编码）。
            if state.provenance.frame_stamp_ns != round(state.frame_time_s * 1e9):
                raise StateContractError("frame_time_stamp_mismatch")
            if state.provenance.sample_stamp_ns != round((state.frame_time_s - 0.005) * 1e9):
                raise StateContractError("native_preupdate_sample_time_mismatch")
        except (StateContractError, TypeError, ValueError, OverflowError) as error:
            return self.reject_state(error, prior_fault=prior_fault)
        self.latch_runtime_faults(
            policy_finite=policy_finite,
            actuator_fault_code=state.actuator_fault_code,
            prior_fault=prior_fault,
        )
        # elapsed_s 相对生产者 run 起点，单位 s；以下阈值均为归档原值（硬编码）。
        t = state.elapsed_s
        tilt = max(abs(state.roll_rad), abs(state.pitch_rad))
        # 观察窗 1.5 s：倾倒 >0.8 rad 或离地 <0.15 m 只记录，本身不单独构成停车。
        observed_fall_or_low = bool(t > 1.5 and (tilt > 0.8 or state.body_clearance_m < 0.15))
        # contacts[0] 是 native 本体接触位；此处只置诊断位，不进入 fault 或 stop。
        observed_body_contact = bool(t > 1.5 and state.contacts[0] > 0)
        if observed_fall_or_low:
            self.fault = self.fault or "navigation_physical_fall_or_low_clearance"
        # 持续窗 2.0 s：倾倒 >1.1 rad 或离地 <0.10 m 才锁存持续倾倒状态。
        persistent = bool(t > 2.0 and (tilt > 1.1 or state.body_clearance_m < 0.10))
        if not persistent:
            self.fall_started = None
        elif self.fall_started is None:
            self.fall_started = t
        # 持续倾倒再观察 4.0 s 才结束观察并请求停车；fall_started_s 单位 s。
        observation_end = bool(self.fall_started is not None and t - self.fall_started >= 4.0)
        # 已有 fault 且 elapsed >= 2 s 同样停车；沿用归档阈值，不在此加严或放宽。
        stop = bool(observation_end or (self.fault is not None and t >= 2))
        return SimulationSafetyResult(
            fault=self.fault,
            stop=stop,
            reason=self.fault,
            observed_fall_or_low=observed_fall_or_low,
            observed_body_contact=observed_body_contact,
            persistent_fall=persistent,
            persistent_fall_observation_end=observation_end,
            fall_started_s=self.fall_started,
        )
