import argparse
import asyncio
import logging

from stretch.motors.async_stepper import AsyncStepper, Mode
from stretch.utils.config import Config
from stretch.utils.logging import configure_logging

logger = logging.getLogger(__name__)


async def main() -> None:
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
    stepper = AsyncStepper(stepper_config)

    # Starts the stepper motor.
    await stepper.startup()

    await stepper.send_load_test_payload()

    stepper.gains.enable_guarded_mode = True
    stepper.gains.enable_sync_mode = False
    await stepper.send_gains()

    await stepper.send_trigger(reset_pos_calibrated=True)

    await stepper.run_command(
        mode=Mode.POS_TRAJ_INCR,
        x_des=stepper.translate_m_to_motor_rad(0.05),
    )

    logger.info("Done; sleeping...")
    await asyncio.sleep(1.0)

    # Stops the stepper motor.
    await stepper.stop()


if __name__ == "__main__":
    # python -m stretch.scripts.single_motor_control
    asyncio.run(main())
