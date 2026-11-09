"""默认关闭的 metadata 异步模式，以及独立的默认关闭 body-only 扩展。

不改变消息内容、QoS 可靠性/历史策略或时钟。AUTO 让 pinned 的 rmw_fastrtps 8.4.4
在保留原有 memory policy 与 data-sharing override 的同时，按单一 topic XML 选择
publication mode。实际 DataWriter QoS 只在启动时读一次，不从环境变量或 publish
耗时推断。

两个开关都在模块导入时读取：R119_METADATA_ASYNC（默认 "0"）、R119_BODY_ASYNC
（默认 "0"，且要求 metadata 已启用）；非法值直接 ValueError。当前
configs/launch/settings.json 的 features 把两者都配成 "1"。
"""

import ctypes
import hashlib
import json
import os
from pathlib import Path
import time

# 进程级开关：导入即求值，因此必须在任何 rclpy 实体创建之前设置。
KEY = "R119_METADATA_ASYNC"
OPTION = os.environ.get(KEY, "0")
if OPTION not in ("0", "1"):
    raise ValueError(KEY + " must be 0 or 1")
ENABLED = OPTION == "1"
# body-only 扩展：单独开关，但依赖 metadata ASYNC 已启用。
BODY_KEY = "R119_BODY_ASYNC"
BODY_OPTION = os.environ.get(BODY_KEY, "0")
if BODY_OPTION not in ("0", "1"):
    raise ValueError(BODY_KEY + " must be 0 or 1")
BODY_ENABLED = BODY_OPTION == "1"
if BODY_ENABLED and not ENABLED:
    raise ValueError("R119 body experiment requires existing metadata ASYNC ON")
from embodied_navigation.common.config import ROOT

HERE = ROOT / "src/embodied_navigation/common/communication"
TOPIC = "/demo/navigation/localization_bundle"
BODY_TOPIC = "/demo/slam/body_odom"
# 列名/顺序必须与 writer_qos.so 的 r119_read_writer 输出一致；每项对应启动时读取的
# DDS QoS/topic-name-check 快照中的一个 int64。
FIELDS = (
    "publish_mode",
    "reliability",
    "durability",
    "history",
    "history_depth",
    "max_samples",
    "max_instances",
    "max_samples_per_instance",
    "allocated_samples",
    "history_memory_policy",
    "data_sharing",
    "max_blocking_seconds",
    "max_blocking_nanosec",
    "topic_matches_original_rmw_name",
)
# 模块级缓存：动态库只加载一次；_PREPARED 保存本进程只配置一次的启动证据。
_LIB = None
_PREPARED = None


def digest(path):
    """返回文件原始字节的 SHA-256 十六进制摘要。"""
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def library():
    """惰性加载 writer_qos.so 并声明两个 C ABI 导出，返回缓存的 CDLL。

    失败时由 ctypes 抛 OSError（库文件缺失或无法加载）；成功后可复用同一句柄。
    """
    global _LIB
    if _LIB is not None:
        return _LIB
    lib = ctypes.CDLL(str(ROOT / "install/communication/lib/writer_qos.so"))
    lib.r119_read_writer.argtypes = [
        ctypes.c_size_t,
        ctypes.POINTER(ctypes.c_int64),
        ctypes.c_size_t,
    ]
    lib.r119_read_writer.restype = ctypes.c_int
    lib.r119_parse_profile.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
    lib.r119_parse_profile.restype = ctypes.c_int
    _LIB = lib
    return lib


def validated_transport_baseline(run, references=None):
    """校验当前进程的中间件/发布模式环境与 transport XML，而非 manifest 或源码摘要。

    返回：xml_path 与三个环境变量的证据字典。run 参数当前未参与判断。
    失败：环境变量不符时抛 RuntimeError；transport XML 不是文件时抛
    FileNotFoundError。
    """
    # 硬编码基线：pinned rmw_fastrtps_cpp、关闭全局 XML QoS override、发布模式同步。
    keys = ("RMW_IMPLEMENTATION", "RMW_FASTRTPS_USE_QOS_FROM_XML", "RMW_FASTRTPS_PUBLICATION_MODE")
    expected = ("rmw_fastrtps_cpp", "0", "SYNCHRONOUS")
    if any(os.environ.get(key) != value for key, value in zip(keys, expected)):
        raise RuntimeError("DDS middleware/publication settings invalid")
    xml = Path(os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", ""))
    if not xml.is_file():
        raise FileNotFoundError("DDS transport XML missing")
    return dict(xml_path=str(xml), environment={key: os.environ[key] for key in keys})


def configure(run):
    """在 OwnedRosLifecycle/rclpy.init 之前做一次性配置，只作用于选定进程。

    参数 run 为本次 run 目录，仅用于证据记录。未启用时返回 None；重复调用抛
    RuntimeError。启用时校验环境、文件与 XML 可解析性，把本进程的
    RMW_FASTRTPS_PUBLICATION_MODE 改为 AUTO，并返回启动证据 dict。
    失败：资源文件缺失、中间件/发布模式不符、XML 解析结果不是目标 ASYNCHRONOUS
    等，抛 RuntimeError 或 FileNotFoundError。副作用限于本进程环境变量与 FastDDS
    profile manager 的命名 profile 预载，不替换 inherited transport XML。
    """
    global _PREPARED
    if not ENABLED:
        return None
    if _PREPARED is not None:
        raise RuntimeError("R119 process already configured")
    run = Path(run).resolve()
    scope = run / "runtime_context.json"
    # 必需资源：metadata 的 topic XML 与 native ABI 库；body 扩展启用时再加 body XML。
    paths = [HERE / "metadata_async.xml", ROOT / "install/communication/lib/writer_qos.so"]
    if BODY_ENABLED:
        paths.append(HERE / "body_async.xml")
    for path in paths:
        if not path.is_file():
            raise FileNotFoundError(path)
    # 硬编码要求：pinned rmw_fastrtps_cpp、未启用全局 XML QoS override、进程发布模式
    # 必须已是 SYNCHRONOUS。
    if os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp") not in ("", "rmw_fastrtps_cpp"):
        raise RuntimeError("R119 requires pinned rmw_fastrtps_cpp")
    if os.environ.get("RMW_FASTRTPS_USE_QOS_FROM_XML", "0") not in ("", "0"):
        raise RuntimeError("R119 does not enable global XML QoS override")
    mode_before = os.environ.get("RMW_FASTRTPS_PUBLICATION_MODE")
    if mode_before != "SYNCHRONOUS":
        raise RuntimeError("R119 conflicting explicit process publication mode")
    baseline = validated_transport_baseline(run)
    xml = HERE / "metadata_async.xml"
    inherited = {}
    for key in ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE"):
        value = os.environ.get(key, "")
        if value:
            path = Path(value).resolve()
            if not path.is_file():
                raise FileNotFoundError(path)
            inherited[key] = dict(path=str(path))
    # 用实际安装的 FastDDS 库解析 XML，不创建任何实体。
    if library().r119_parse_profile(str(xml).encode(), TOPIC.encode()) != 1:
        raise RuntimeError("R119 XML did not parse as target ASYNCHRONOUS")
    if (
        BODY_ENABLED
        and library().r119_parse_profile(str(HERE / "body_async.xml").encode(), BODY_TOPIC.encode())
        != 1
    ):
        raise RuntimeError("R119 body-only XML did not parse as target ASYNCHRONOUS")
    # 只把命名 writer profile 预载进本进程的 FastDDS profile manager。rmw 在创建
    # participant 时仍加载原 inherited transport XML；其路径与字节都不替换。
    # 这里把发布模式选择改为 AUTO，让 XML 的命名 topic profile 决定实际模式。
    os.environ["RMW_FASTRTPS_PUBLICATION_MODE"] = "AUTO"
    _PREPARED = dict(
        schema="r119_metadata_async_setup/v1",
        run=str(run),
        pid=os.getpid(),
        requested=True,
        target_topic=TOPIC,
        xml=str(xml),
        before_rclpy_init=True,
        named_profile_preloaded=True,
        inherited_transport_xml=inherited,
        transport_xml_environment_unchanged=True,
        transport_baseline=baseline,
        mode_before=mode_before,
        mode_after=os.environ["RMW_FASTRTPS_PUBLICATION_MODE"],
        only_this_process_publication_mode_changed=True,
        requested_mode="AUTO; named selected topics ASYNCHRONOUS",
        actual_writer_checked=False,
        body_environment_key=BODY_KEY,
        body_environment_value=BODY_OPTION,
        body_async_enabled=BODY_ENABLED,
        body_topic=BODY_TOPIC,
        body_profile=str(HERE / "body_async.xml") if BODY_ENABLED else None,
        original_metadata_mode_preserved=True,
    )
    return dict(_PREPARED)


def read_writer(publisher):
    """用 ctypes 读取一个 live rclpy Publisher 的实际 DataWriter QoS。

    调用方必须持有 live Publisher handle；本函数不创建实体、不改 QoS。
    返回：FIELDS 各列（int64）外加 topic 与 rclpy 侧请求的 depth/reliability/
    history/durability。
    失败：底层读取返回非 0、实际 topic 名不等于原 rmw topic 名、实际 history depth
    与 rclpy QoS 不一致时抛 RuntimeError。
    """
    values = (ctypes.c_int64 * len(FIELDS))()
    # 持有 publisher.handle 期间读取，避免实体被并发销毁。
    with publisher.handle:
        code = library().r119_read_writer(publisher.handle.pointer, values, len(FIELDS))
    if code:
        raise RuntimeError("R119 readonly actual writer check failed: " + str(code))
    # 请求值来自构造 Publisher 时的 QoS，用于与实际 DDS 值对照。
    q = dict(zip(FIELDS, values))
    q.update(
        topic=publisher.topic_name,
        requested_history_depth=publisher.qos_profile.depth,
        requested_reliability=int(publisher.qos_profile.reliability),
        requested_history=int(publisher.qos_profile.history),
        requested_durability=int(publisher.qos_profile.durability),
    )
    # writer_qos.so 第 14 列检查实际 DDS topic 名是否等于 "rt" + 原 rmw topic 名。
    if q["topic_matches_original_rmw_name"] != 1:
        raise RuntimeError("R119 actual DDS topic differs from original rmw topic")
    if q["history_depth"] != publisher.qos_profile.depth:
        raise RuntimeError("R119 actual history depth differs from original rclpy QoS")
    return q


def verify(node, run):
    """启动后复核 5 个 publisher 的实际 DDS QoS，并写 r119 证据文件。

    仅在 ENABLED 时运行；返回结果 dict，同时以独占创建方式写
    run/r119_metadata_async_actual.json（文件已存在则 FileExistsError）。
    失败：未 configure、实际中间件不是 rmw_fastrtps_cpp，或 actual writer 的
    publication mode / 内存策略 / data-sharing / reliability / history depth 与
    期望不符时抛 RuntimeError。
    publication_calls_prove_delivery=False 表示证据只覆盖 QoS 配置，不证明投递。
    """
    if not ENABLED:
        return None
    from rclpy.utilities import get_rmw_implementation_identifier

    if _PREPARED is None or get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
        raise RuntimeError("R119 actual process middleware differs")
    publishers = dict(
        metadata=node.bundle_pub,
        body=node.pubs["body"],
        lidar=node.pubs["lidar"],
        cloud=node.cloud_pub,
        status=node.status_pub,
    )
    actual = {name: read_writer(pub) for name, pub in publishers.items()}
    # 期望值：metadata（以及启用时的 body）为 ASYNCHRONOUS=1，其余为 SYNCHRONOUS=0。
    for name, q in actual.items():
        if q["publish_mode"] != (1 if name == "metadata" or name == "body" and BODY_ENABLED else 0):
            raise RuntimeError("R119 actual publication mode differs: " + name)
        # 这两项由 rmw_fastrtps 8.4.4 保留，而不是由 XML override 设置。
        if q["history_memory_policy"] != 1 or q["data_sharing"] != 3:
            raise RuntimeError("R119 original memory/data-sharing policy differs: " + name)
        # reliability 期望 cloud=1、其余=2：硬编码的底层 Fast DDS kind 数值，
        # 本仓库未给出枚举定义。
        if q["reliability"] != (1 if name == "cloud" else 2):
            raise RuntimeError("R119 original writer reliability differs: " + name)
    result = dict(
        _PREPARED,
        actual_writer_checked=True,
        actual=actual,
        actual_middleware=get_rmw_implementation_identifier(),
        observed_monotonic_ns=time.monotonic_ns(),
        publication_calls_prove_delivery=False,
        root_component_OFF_ON_comparison_required=True,
        original_source_age_limit_ns=300000000,
        S1_enabled=False,
        navigation_PASS=False,
    )
    # 证据文件用 x 模式独占创建：同一进程重复 verify 会因文件已存在而失败。
    with (Path(run) / "r119_metadata_async_actual.json").open("x") as stream:
        json.dump(result, stream, allow_nan=False)
        stream.write("\n")
    return result
