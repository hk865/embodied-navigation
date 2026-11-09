"""File-only dependency check: useful before ROS, PyTorch or native builds exist."""

from embodied_navigation.common.config import ROOT, path


def check():
    """List the files the normal launch needs, without starting anything.

    Entries marked as configured come from
    :func:`embodied_navigation.common.config.path`; the others are fixed
    project-relative assets and build outputs.  ``ready`` is True only when every
    entry is an existing regular file (``Path.is_file``, so directories and
    symlinks to directories do not count).  Each dependency reports its path
    relative to ``ROOT`` when it lives inside the project, otherwise the absolute
    path (e.g. an actor interpreter outside the checkout).  No process,
    simulation or socket is started here, which is why ``simulation_started`` is
    always ``False``.
    """
    files = {
        "Student": path("student_model"),  # configured runtime path
        "Actor Python": path("actor_python"),  # configured runtime path ('auto' sentinel)
        "Go2 URDF": ROOT / "assets/world/go2_measured.urdf",
        "Go2 SDF": ROOT / "assets/world/go2_converted.sdf",
        "world": ROOT / "assets/world/three_floors.sdf",
        "route": ROOT / "assets/geometry/route.json",
        "L1 mesh": ROOT / "assets/meshes/L1_4D_Lidar.dae",
        "actuator": path("actuator"),  # configured runtime path
        "writer helper": ROOT / "install/communication/lib/writer_qos.so",
        "patched DDS": ROOT / "install/dds/lib/libfastrtps.so.2.14.6",
        "ROS install": path("ros_prefix") / "local_setup.bash",
        "SLAM": path("ros_prefix") / "lib/fast_livo2_ros/fastlivo_mapping",
        "SCAN": path("ros_prefix") / "lib/scan_planner/scan_planner_node",
    }
    result = {
        name: dict(
            # Keep in-project paths relative for stable reporting; anything
            # outside the checkout keeps its absolute form.
            path=str(p.relative_to(ROOT)) if p.is_relative_to(ROOT) else str(p),
            present=p.is_file(),
        )
        for name, p in files.items()
    }
    return dict(
        ready=all(v["present"] for v in result.values()),
        dependencies=result,
        simulation_started=False,
    )
