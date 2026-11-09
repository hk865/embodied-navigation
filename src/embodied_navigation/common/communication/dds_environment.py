"""Select the project DDS library directly; no version/Build-ID/SHA contracts."""

from pathlib import Path
import os, re
from embodied_navigation.common.config import ROOT

LIBRARY = ROOT / "install/dds/lib/libfastrtps.so.2.14.6"
DOMAIN_QUERY_DIRECTORY = ROOT / "runs/dds_graph"
ENV_KEYS = ("LD_PRELOAD",)


def library_path():
    if not LIBRARY.is_file():
        raise FileNotFoundError("Project DDS library missing: " + str(LIBRARY))
    return LIBRARY.resolve()


def _preloads(value):
    return [p for p in re.split(r"[:\s]+", value) if p]


def reject_unknown_preloads(environment, **unused):
    if _preloads(environment.get("LD_PRELOAD", "")):
        raise RuntimeError("Unexpected inherited preload")
    return []


def configure(environment, **unused):
    """Return a child environment that selects the project DDS library and drops foreign keys."""
    library_path()
    reject_unknown_preloads(environment)
    result = dict(environment)
    prefix = str(LIBRARY.parent)
    result["LD_LIBRARY_PATH"] = ":".join(
        [prefix, *[p for p in result.get("LD_LIBRARY_PATH", "").split(":") if p and p != prefix]]
    )
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
    """Fail unless the project DDS directory is first on LD_LIBRARY_PATH and no preload leaked."""
    reject_unknown_preloads(environment)
    if environment.get("LD_LIBRARY_PATH", "").split(":")[0] != str(LIBRARY.parent):
        raise RuntimeError("Project DDS library directory missing from runtime environment")
    return True


def native_environment(run, environment):
    """Verified environment fragment given to a native child process for this run."""
    verify_environment(environment)
    return dict(DEMO_RUN_DIR=str(Path(run).resolve()))
