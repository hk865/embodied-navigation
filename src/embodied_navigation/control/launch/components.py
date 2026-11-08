from embodied_navigation.launch.launching import python, owned
def components(run):
    commands=[('controller',python('embodied_navigation.navigation.pid_navigation','--run',run)),
        ('command_bridge',python('embodied_navigation.robot.unitree.bridge','--runtime-config',run/'runtime_context.json','--command-file',run/'navigation_command.json')),
        ('mission',python('embodied_navigation.navigation.mission.mission_runtime','--run',run))]
    return [action for role,command in commands for action in owned(role,command,run)]
