# 独立后端快照桥测试

本目录的 `backend_snapshot_flow.cpp` 与 `../src/backend/snapshot_bridge.cpp`
只依赖 C++17 标准库，不依赖 ROS/PCL/Eigen，也不读写前端滤波器、体素图或视觉 patch。
LIO/VIO 必须由同一 owner 顺序调用；传给 worker 的局部 LiDAR 点由预分配槽拥有。

从仓库根运行：

```bash
CORE=src/localization/frontend/fast_livo2_core
mkdir -p /tmp/livo-backend-build
g++ -std=c++17 -O2 -Wall -Wextra -Werror -pthread -I"$CORE/include" \
  "$CORE/src/backend/snapshot_bridge.cpp" "$CORE/tests/backend_snapshot_flow.cpp" \
  -o /tmp/livo-backend-build/backend_snapshot_flow
/tmp/livo-backend-build/backend_snapshot_flow
```

不指定参数时在新临时目录生成测试证据；也可传一个**尚未使用**的输出目录。
测试覆盖源云复用后后台云仍独立、LIO/VIO 精确戳配对与拒绝近戳、重复事件不双计、
VIO失败/缺席保留一次LIO、候选先筛后复制、队列/未消费spool有界丢新、worker故障
隔离、`bridge_status.json` 过载计数可见。10,000次提交输出本机观察值，不是实时保证。

可追加 `-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer` 做地址/UB检查。
若执行器使用ptrace，LeakSanitizer可能不支持；本云执行器已遇到此限制，地址/UB
检查使用 `ASAN_OPTIONS=detect_leaks=0` 单独验证，不能声称泄漏检查通过。

## 当前接入状态

旧6401e5fe原运行文件没有活跃改动。仅有可迁移提案补丁，位于本云工作区
`backend_implementation/checkpoint/provisional_frontend_hooks.patch`。用户最新指定的
clean资产分支需先确认，再审查迁移；不得把此独立编译当作ROS/Gazebo接入验收。

默认options队列4、点上限4096、未消费目录16。后端采样不改原前端云；只有选中KF
才复制，复制有实际开销并通过copy_ns观察。worker才运行ASCII PCD/JSON I/O，
原子目录rename后消费者可读。worker还每秒至多写一次状态（关闭时最终写一次）。
后台错误、队列满、spool满只计数/丢新，不等待worker。停止后close会flush pending
LIO并join，不可在实时帧处理内部调用。正常调用不做hash、源码绑定或全云完整性检查。
