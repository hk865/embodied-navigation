"""只读选择已验证的 observer UDP 接收配置，不扩大适用范围。"""

import hashlib
from pathlib import Path


PROFILE = "configs/simulation/dynanav/observer_udp_rx4m.xml"
PROFILE_KEYS = ("FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE")
DEFAULT_FILES = ("DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml")


def select_transport(mode, policy, environment, repo, cwd):
    """不修改调用方环境，也不推断未明确指定的传输方式。"""
    if policy not in ("auto", "baseline"):
        raise ValueError("observer transport must be auto or baseline")
    custom_keys = [key for key in (*PROFILE_KEYS, "FASTDDS_ENVIRONMENT_FILE", "SKIP_DEFAULT_XML")
                   if key in environment]
    default_files = [str(Path(cwd) / name) for name in DEFAULT_FILES
                     if (Path(cwd) / name).exists()]
    selected = False
    if policy == "baseline":
        reason = "baseline requested; inherited DDS configuration retained"
    elif mode != "livo":
        reason = "LIO mode unchanged"
    elif custom_keys or default_files:
        reason = "existing DDS configuration retained"
    elif environment.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp":
        reason = "FastRTPS implementation not explicitly selected"
    elif environment.get("FASTDDS_BUILTIN_TRANSPORTS") != "UDPv4":
        reason = "UDPv4 transport not explicitly selected"
    else:
        selected = True
        reason = "validated LIVO FastRTPS UDPv4 receive profile selected"
    profile = Path(repo) / PROFILE if selected else None
    rmem = Path("/proc/sys/net/core/rmem_max")
    try:
        rmem_max = int(rmem.read_text().strip())
    except (OSError, ValueError):
        rmem_max = None
    requested = 4194304 if selected else None
    return {
        "policy": policy,
        "reason": reason,
        "profile": str(profile) if profile else None,
        "profile_sha256": hashlib.sha256(profile.read_bytes()).hexdigest() if profile else None,
        "environment_assignment": {"FASTRTPS_DEFAULT_PROFILES_FILE": str(profile)} if profile else {},
        "inherited_environment": {key: environment[key] for key in (
            "RMW_IMPLEMENTATION", "FASTDDS_BUILTIN_TRANSPORTS", *PROFILE_KEYS,
            "FASTDDS_ENVIRONMENT_FILE", "SKIP_DEFAULT_XML") if key in environment},
        "inherited_default_xml": default_files,
        "receive_buffer_request_bytes": requested,
        "host_rmem_max_bytes": rmem_max,
        "host_limit_below_request": rmem_max < requested if rmem_max is not None and requested else None,
        "socket_buffer_verified": False,
        "verification_scope": "Configuration selection only. Verify owned native UDP sockets at runtime; Linux skmem rb includes doubled accounting. No sysctl or SO_RCVBUFFORCE used.",
    }
