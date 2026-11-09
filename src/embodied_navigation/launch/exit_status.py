"""Report real managed-process failures, including children of ROS launch."""


def failed_exits(process_exit_codes, component_exits):
    failures = [
        dict(role=role, returncode=code) for role, code in process_exit_codes.items() if code != 0
    ]
    failures.extend(
        dict(role=row["role"], returncode=row["returncode"])
        for row in component_exits
        if row["returncode"] != 0
    )
    return failures
