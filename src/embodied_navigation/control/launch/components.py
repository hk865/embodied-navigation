"""Control-side launch actions: controller, command bridge and mission runtime.

The controller publishes ``/demo/cmd_vel``; the bridge validates that stream
and writes ``navigation_command.json`` for the actor. The mission runtime
coordinates requests and status rather than writing the actor command file.
"""

from embodied_navigation.launch.launching import python, owned


def components(run):
    """Return the ordered LaunchActions for controller, bridge and mission."""
    commands = [
        ("controller", python("embodied_navigation.navigation.pid_navigation", "--run", run)),
        (
            "command_bridge",
            python(
                "embodied_navigation.robot.unitree.bridge",
                "--runtime-config",
                run / "runtime_context.json",
                "--command-file",
                run / "navigation_command.json",
            ),
        ),
        ("mission", python("embodied_navigation.navigation.mission.mission_runtime", "--run", run)),
    ]
    return [action for role, command in commands for action in owned(role, command, run)]
