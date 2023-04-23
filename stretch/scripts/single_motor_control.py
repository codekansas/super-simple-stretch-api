import argparse
import logging
import time

from stretch.motors.stepper import Mode, Stepper
from stretch.utils.config import Config
from stretch.utils.logging import configure_logging

logger = logging.getLogger(__name__)


def main() -> None:
    configure_logging()

    parser = argparse.ArgumentParser()
    parser.add_argument("device", type=str, help="Device name")
    parser.add_argument("-p", "--path", help="Path to the config file")
    args = parser.parse_args()

    # Loads the config for the specific stepper object.
    config = Config.load(file_path=args.path)
    if args.device not in config.stepper:
        raise ValueError(f"Device '{args.device}' not found in the config file")
    stepper_config = config.stepper[args.device]
    stepper = Stepper(stepper_config)

    # Starts the stepper motor.
    stepper.startup()

    stepper.set_command(
        mode=Mode.POS_TRAJ,
        x_des=0.1,
        v_des=0.0,
        a_des=0.0,
    )

    time.sleep(1.0)

    # Stops the stepper motor.
    stepper.stop()


if __name__ == "__main__":
    # python -m stretch.scripts.home
    main()
