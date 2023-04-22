#!/usr/bin/env python

import argparse
import logging
import re
import time
from dataclasses import dataclass
from typing import Literal, get_args

from stretch.utils.colors import Color, colorize
from stretch.utils.formatting import format_time
from stretch.utils.logging import configure_logging

logger = logging.getLogger(__name__)

Action = Literal[
    # Base operations.
    "move-forward",
    "move-backward",
    "turn-left",
    "turn-right",
    # Arm operations.
    "extend-arm",
    "retract-arm",
    "raise-arm",
    "lower-arm",
    # Gripper operations.
    "open-gripper",
    "close-gripper",
    # Head operations.
    "look",
    # Speaker operations.
    "say",
    # Quit.
    "quit",
]


LookAt = Literal["forward", "down", "gripper"]


@dataclass
class Command:
    action: Action
    amount: float | None = None
    look: LookAt | None = None
    say: str | None = None


def get_cmd(s: str, color: Color, i: int = 0) -> str:
    return colorize(s[:i], color) + colorize(s[i : i + 1], color, bold=True) + colorize(s[i + 1 :], color, bold=False)


def show_help() -> None:
    print("Usage:")
    print(f" ↪ {get_cmd('forward', 'cyan', 0)} {colorize('(N meters)', 'grey')} to move forward")
    print(f" ↪ {get_cmd('backward', 'cyan', 0)} {colorize('(N meters)', 'grey')} to move backward")
    print(f" ↪ {get_cmd('left', 'cyan', 0)} {colorize('(N degrees)', 'grey')} to turn left")
    print(f" ↪ {get_cmd('right', 'cyan', 0)} {colorize('(N degrees)', 'grey')} to turn right")
    print(f" ↪ {get_cmd('extend-arm', 'cyan', 0)} {colorize('(N meters)', 'grey')} to extend the arm")
    print(f" ↪ {get_cmd('retract-arm', 'cyan', 2)} {colorize('(N meters)', 'grey')} to retract the arm")
    print(f" ↪ {get_cmd('raise-arm', 'cyan', 1)} {colorize('(N meters)', 'grey')} to raise the arm")
    print(f" ↪ {get_cmd('lower-arm', 'cyan', 2)} {colorize('(N meters)', 'grey')} to lower the arm")
    print(f" ↪ {get_cmd('open-gripper', 'cyan', 0)} to open the gripper")
    print(f" ↪ {get_cmd('close-gripper', 'cyan', 0)} to close the gripper")
    print(f" ↪ {get_cmd('look', 'cyan', 3)} {colorize('what', 'light-blue')} to look at something")
    print(f" ↪ {get_cmd('say', 'cyan', 0)} {colorize('what', 'light-blue')} to say something")
    print(f" ↪ {get_cmd('quit', 'red', 0)} to quit")


def get_amount(s: list[str]) -> float | None:
    if len(s) == 0:
        return None
    if len(s) == 1:
        return float(s[0])
    raise ValueError("too many arguments")


def get_look_at(what: str) -> LookAt:
    match what.lower():
        case "f" | "forward":
            return "forward"
        case "d" | "down":
            return "down"
        case "g" | "gripper":
            return "gripper"
        case _:
            raise ValueError(f"unknown look target '{what}' - choices are: {get_args(LookAt)}")


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
                case "e" | "extend-arm":
                    return Command("extend-arm", get_amount(rest))
                case "t" | "retract-arm":
                    return Command("retract-arm", get_amount(rest))
                case "a" | "raise-arm":
                    return Command("raise-arm", get_amount(rest))
                case "w" | "lower-arm":
                    return Command("lower-arm", get_amount(rest))
                case "o" | "open-gripper":
                    return Command("open-gripper")
                case "c" | "close-gripper":
                    return Command("close-gripper")
                case "k" | "look":
                    return Command("look", look=get_look_at(" ".join(rest)))
                case "s" | "say":
                    return Command("say", say=" ".join(rest))
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
