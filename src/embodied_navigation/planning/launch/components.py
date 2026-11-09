"""Planning process composition for one run directory."""

from embodied_navigation.common.config import path
from embodied_navigation.launch.launching import python, owned


def components(run):
    """Build the owned ``scan_supervisor`` process for ``run``.

    The planner binary and its default config come from the configured ROS prefix;
    the run directory supplies the v34 scan overrides.
    """
    prefix = path("ros_prefix")
    return owned(
        "scan_supervisor",
        python(
            "embodied_navigation.planning.scan_epoch_supervisor",
            "--run",
            run,
            "--planner",
            prefix / "lib/scan_planner/scan_planner_node",
            "--config",
            prefix / "share/scan_planner/config/planner.yaml",
            "--params",
            run / "v34_scan_overrides.yaml",
        ),
        run,
    )
