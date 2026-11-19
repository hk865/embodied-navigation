#!/usr/bin/env python3
"""Run a child with frozen log budgets and its original exit/signal semantics."""

import argparse
from pathlib import Path

from embodied_navigation.common.process_log import exit_like_child, read_log_config, run_logged


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command[1:] if args.command[:1] == ["--"] else args.command
    if not command:
        parser.error("a child command is required after --")
    try:
        config = read_log_config(args.config)
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.error(str(error))
    exit_like_child(run_logged(command, args.output_dir, config))


if __name__ == "__main__":
    main()
