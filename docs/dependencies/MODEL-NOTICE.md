# Student权重来源、发布依据与许可状态

## 本次提供的文件

`student_model_4999_deterministic_jit.pt`为发布者项目`RL_for_unitree`本地训练、导出的既有去debug部署版，1,342,573字节。用于Student45/GRU256/12动作CPU推理；不是直接下载的第三方预训练Student。本次补回的是此前实际用于导航验收的部署文件，不是原始训练checkpoint，也不是017后续继续训练的9999模型；不重训、不重新导出、不改写参数。

原017 JIT导出为1,354,578字节；原发布基线的来源记录说明部署版仅移除了路径debug ZIP条目。现部署档案包含25个ZIP成员，无debug条目；本次未执行模型或新增内容哈希校验。

## 已追溯训练链

| 阶段 | 权重关系与证据 |
| --- | --- |
| `STAGE6-GO2-AER-GAIT-GEOMETRY-009` | 本地从零训练，取`model_5999`；配置`initialization: scratch`、`external_checkpoint: forbidden`；保存agent为`resume=False`且无load_run/load_checkpoint，正式启动命令无恢复参数 |
| `STAGE6-GO2-AER-ROUGH-TEACHER-002` | 以009的5999进行actor/critic warm start，重置optimizer/iteration，训练得到`model_9999` |
| `STAGE6-GO2-AER-HEIGHT-DISTRIBUTION-015-PHASE-A` | 以Rough002的9999为父权重，后续选`model_1000`作为冻结Teacher |
| `STAGE6-GO2-AER-DEPLOYABLE-COMPARISON-016/infra_retry2/student/formal` | seed42新Student与新Adam；仅加载Teacher，冻结其参数；45维Student以Huber动作蒸馏训练5000次更新，得到`model_4999.pt` |
| `STAGE6-GO2-AER-DEPLOYABLE-FOLLOWUP-017/deployment_export/cpu_parity_20261004_001` | 由016的4999与agent配置构造确定性JIT；既有导出审计记录action/hidden最大绝对误差均为0 |

这条记录链追至本地从零训练起点，未发现其加载外部预训练checkpoint；它说明训练来源，不以配置文件代替所有权法律证明，也不把训练框架许可证直接套用于权重。

可供拥有训练工程者核对的相对路径：

- `configs/experiments/stage6_go2_aer_gait_geometry_009.yaml`
- `configs/experiments/stage6_go2_aer_rough_teacher_002.yaml`
- `configs/experiments/stage6_go2_aer_height_distribution_015.json`
- `configs/experiments/stage6_go2_deployable_comparison_016.json`
- `scripts/train_go2_deployable_016.py`与`scripts/audit_go2_deployable_export_017.py`
- 016的`student/formal/initialization_audit.json`、`training_audit.json`及017的`student_export_audit.json`

训练原始日志、全部checkpoint和训练工程不随本导航分支发布；不能仅凭本分支或训练仓库HEAD声称可完整重训练。

## 发布依据与当前对外许可状态

此文件作为发布者项目自主训练/导出的部署产物，依据发布者对本项目的发布授权随本功能分支提供。原公开main也曾提供该部署资产；本次恢复依据还包括上述新增核实的自训链证据，不仅依据“以前公开过”。

本次没有为权重新增MIT、Apache或其它独立许可证，不自动继承训练框架、机器人资产或导航源码的第三方许可证。第三方对这份权重使用、修改、再分发的统一授权范围尚未单列声明；此NOTICE记录当前事实，不擅自增加授权条款。

L1网格属于另一资产，仍未随本分支提供，不能以本Student的发布依据覆盖L1或改变其碰撞几何。
