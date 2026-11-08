# common/communication：DDS 通信配置

本目录集中管理项目的 Fast-DDS 运行环境、传输 XML 与只读的 writer QoS 检查工具，不承载应用层消息逻辑。

## 边界

- 不创建 DataWriter/订阅者、不发布消息、不直接修改已有 writer 的 QoS；启动前可预载命名 publisher profile，并读取、校验实际 writer 配置。
- 不修改 RMW 版本或 Build-ID；只要求使用仓库自建的 DDS 库。

## 关键文件与接口

- `dds_environment.py`：`configure(environment)` 选择 `install/dds/lib/libfastrtps.so.2.14.6`，把其目录放在 `LD_LIBRARY_PATH` 首位，并拒绝继承未知 `LD_PRELOAD`；`verify_environment()` 再校验；`native_environment(run, environment)` 校验传入环境后，仅返回含 `DEMO_RUN_DIR` 的覆写字典，不合并或修改传入环境。库文件缺失时抛 `FileNotFoundError`。
- `dds_transport.xml`：仅本机 UDP（127.0.0.1）+ SHM 的传输描述，供运行进程通过 `FASTRTPS_DEFAULT_PROFILES_FILE` 使用。
- `body_async.xml` / `metadata_async.xml`：分别把 `/demo/slam/body_odom`、`/demo/navigation/localization_bundle` 的 `publishMode` 设为 `ASYNCHRONOUS`，不改其他 QoS。
- `writer_qos.cpp` + `CMakeLists.txt`：编译 `writer_qos.so`，导出 `r119_read_writer`（只读获取 writer QoS 与 topic 一致性等 14 项值）与 `r119_parse_profile`（把 XML 加载到进程的 Fast DDS profile manager，再读取指定 profile 的 publish mode）；后者具有预载 profile 的进程内副作用。

## 输入输出与调用

输入是子进程环境字典；输出是修改后的环境字典或异常。[launch/main.py](../../launch/main.py)、[launch/building.py](../../launch/building.py)、[navigation/robogauge/benchmark.py](../../navigation/robogauge/benchmark.py) 在起进程前调用 `configure()`；[perception/sensors/async_writers.py](../../perception/sensors/async_writers.py) 通过 ctypes 使用 `writer_qos.so` 校验实际 writer。

## 测试与限制

本目录无专用测试；`dds_environment` 只经上述启动入口间接使用，`writer_qos.cpp` 需要本机构建产物才能验证。`install/` 中缺少 `libfastrtps.so.2.14.6` 或 `writer_qos.so` 属于本机未构建，不是源码缺失。传输 XML 与补丁库的兼容性只在既有构建环境核对，未在本会话重新构建或运行。
