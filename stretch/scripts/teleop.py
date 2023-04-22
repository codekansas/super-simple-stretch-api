#!/usr/bin/env python

import argparse
import logging
import re
import time
from dataclasses import dataclass
from typing import Literal

from stretch.utils.colors import colorize
from stretch.utils.formatting import format_time
from stretch.utils.logging import configure_logging

logger = logging.getLogger(__name__)

Action = Literal["move-forward", "move-backward", "turn-left", "turn-right", "quit"]


@dataclass
class Command:
    action: Action
    amount: float | None = None


def show_help() -> None:
    get_cmd = lambda s, color: colorize(s[:1], color, bold=True) + colorize(s[1:], color, bold=False)

    print("Usage:")
    print(f" ↪ {get_cmd('forward', 'cyan')} {colorize('(N meters)', 'grey')} to move forward")
    print(f" ↪ {get_cmd('backward', 'cyan')} {colorize('(N meters)', 'grey')} to move backward")
    print(f" ↪ {get_cmd('left', 'cyan')} {colorize('(N degrees)', 'grey')} to turn left")
    print(f" ↪ {get_cmd('right', 'cyan')} {colorize('(N degrees)', 'grey')} to turn right")
    print(f" ↪ {get_cmd('quit', 'red')} to quit")


def get_amount(s: list[str]) -> float | None:
    if len(s) == 0:
        return None
    if len(s) == 1:
        return float(s[0])
    raise ValueError("too many arguments")


def get_next_command() -> Command:
    """Gets the next command from the user.

    Returns:
        The command, parsed from the user.
    """

    while True:
        action, *rest = re.split(r"\s+", input("? ").strip().lower())
        try:
            match action.lower():
                case "f" | "forward":
                    return Command("move-forward", get_amount(rest))
                case "b" | "backward":
                    return Command("move-backward", get_amount(rest))
                case "l" | "left":
                    return Command("turn-left", get_amount(rest))
                case "r" | "right":
                    return Command("turn-right", get_amount(rest))
                case "q" | "quit":
                    return Command("quit")
                case _:
                    show_help()
        except ValueError as e:
            print(colorize(str(e), "red"))


def main() -> None:
    configure_logging()
    logger.info("Starting teleop...")

    parser = argparse.ArgumentParser()
    parser.add_argument("--small-speed", type=float, default=0.1, help="Speed for small movements, in m/s")
    parser.add_argument("--large-speed", type=float, default=0.5, help="Speed for large movements, in m/s")
    parser.add_argument("--small-rot", type=float, default=10, help="Rotation speed for small movements, in deg/s")
    parser.add_argument("--large-rot", type=float, default=45, help="Rotation speed for large movements, in deg/s")
    parser.parse_args()

    start_time = time.time()

    while (command := get_next_command()).action != "quit":
        print(command)

    end_time = time.time()
    logger.info("Teleop session ended after %s", format_time(start_time - end_time))


if __name__ == "__main__":
    # python -m stretch.scripts.teleop
    main()
