"""直接选用工程自带的 DDS 库；不依赖版本号/Build-ID/SHA 契约。"""

from pathlib import Path
import os, re
from embodied_navigation.common.config import ROOT

# 路径与文件名是硬编码契约：DDS 库固定为 install/dds 下的 2.14.6 版本，
# 图查询目录固定为 runs/dds_graph；缺失时 library_path() 会显式失败。
LIBRARY = ROOT / "install/dds/lib/libfastrtps.so.2.14.6"
DOMAIN_QUERY_DIRECTORY = ROOT / "runs/dds_graph"
ENV_KEYS = ("LD_PRELOAD",)


def library_path():
    # 返回解析后的绝对路径；文件不在就抛出 FileNotFoundError，不做静默回退。
    if not LIBRARY.is_file():
        raise FileNotFoundError("Project DDS library missing: " + str(LIBRARY))
    return LIBRARY.resolve()


def _preloads(value):
    # LD_PRELOAD 允许冒号或空白分隔，空项丢弃。
    return [p for p in re.split(r"[:\s]+", value) if p]


def reject_unknown_preloads(environment, **unused):
    # 子进程不允许继承任何 LD_PRELOAD：预加载库会先于工程 DDS 生效，使
    # “选了哪个 DDS”不再可验证。
    if _preloads(environment.get("LD_PRELOAD", "")):
        raise RuntimeError("Unexpected inherited preload")
    return []


def configure(environment, **unused):
    """返回选中工程 DDS 库并剔除外部键的子进程环境。

    做法：把工程 DDS 目录放到 LD_LIBRARY_PATH 首位（已存在则先去重再前置），
    并删除其他运行环境遗留的 PRIVATE_DDS_*/OWNER_WAIT_PREFIX 键，避免跨 run
    或跨 DDS 实现的隐式契约泄漏。"""
    library_path()
    reject_unknown_preloads(environment)
    result = dict(environment)
    prefix = str(LIBRARY.parent)
    result["LD_LIBRARY_PATH"] = ":".join(
        [prefix, *[p for p in result.get("LD_LIBRARY_PATH", "").split(":") if p and p != prefix]]
    )
    # 这些键属于旧运行时的私有契约，不属于本次子进程环境；即使值无害也
    # 一并移除，保证环境可复现。
    for key in (
        "PRIVATE_DDS_RUNTIME_CONTRACT",
        "PRIVATE_DDS_RUNTIME_CONTRACT_SHA256",
        "PRIVATE_DDS_WITNESS_DESCRIPTOR",
        "PRIVATE_DDS_WITNESS_DIR",
        "OWNER_WAIT_PREFIX",
    ):
        result.pop(key, None)
    return result


def verify_environment(environment, **unused):
    """校验工程 DDS 目录位于 LD_LIBRARY_PATH 首位且没有预加载泄漏，否则失败。"""
    reject_unknown_preloads(environment)
    if environment.get("LD_LIBRARY_PATH", "").split(":")[0] != str(LIBRARY.parent):
        raise RuntimeError("Project DDS library directory missing from runtime environment")
    return True


def native_environment(run, environment):
    """给 native 子进程的、已校验的环境片段（本次运行的 DEMO_RUN_DIR）。

    ``run`` 会解析成绝对路径；调用前先执行 verify_environment。
    """
    verify_environment(environment)
    return dict(DEMO_RUN_DIR=str(Path(run).resolve()))
