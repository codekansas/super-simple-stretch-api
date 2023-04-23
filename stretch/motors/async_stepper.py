import array as arr
import asyncio
import logging

from stretch.motors.stepper import RPC, BoardInfo, Command, Mode, Protocol
from stretch.uart.async_transport import AsyncTransport
from stretch.utils.config import StepperConfig


class AsyncStepper:
    def __init__(self, params: StepperConfig, name: str | None = None) -> None:
        self.name = params.usb[5:] if name is None else name
        self.usb = params.usb
        self.params = params

        self.logger = logging.getLogger(f"stepper.{self.name}")
        self.lock = asyncio.Lock()
        self.transport = AsyncTransport(usb=self.usb, name=self.name)

        # Copies command from stepper configuration.
        self.command = Command(
            mode=Mode.SAFETY,
            x_des=0.0,
            v_des=params.motion.vel,
            a_des=params.motion.acc,
            stiffness=1.0,
            i_feedforward=0.0,
            i_contact_pos=params.gains.i_contact_pos,
            i_contact_neg=params.gains.i_contact_neg,
        )

        # Read from the board.
        self.board_info: BoardInfo | None = None
        self.protocol: Protocol | None = None

    async def startup(self) -> None:
        async with self.lock:
            await self.transport.startup()

            reply = await self.transport.send(arr.array("B", [RPC.GET_STEPPER_BOARD_INFO]), throw_on_error=True)
            if reply[0] != RPC.REPLY_STEPPER_BOARD_INFO:
                raise IOError(f"Unexpected reply: {reply[0]}")
            self.board_info, _ = BoardInfo.unpack(reply[1:])

            match self.board_info.protocol_version:
                case "p0":
                    self.protocol = "P0"
                case "p1":
                    self.protocol = "P1"
                case "p2":
                    self.protocol = "P2"
                case _:
                    self.logger.error(
                        "Firmware protocol mismatch. Protocol on board is %s. Disabling device. "
                        "Please upgrade the firmware or version running on the Stretch.",
                        self.board_info.protocol_version,
                    )
                    await self.transport.stop()

    async def stop(self) -> None:
        async with self.lock:
            await self.transport.stop()

    async def run_command(
        self,
        mode: int | None = None,
        x_des: float | None = None,
        v_des: float | None = None,
        a_des: float | None = None,
        i_des: float | None = None,
        stiffness: float | None = None,
        i_feedforward: float | None = None,
        i_contact_pos: float | None = None,
        i_contact_neg: float | None = None,
    ) -> None:
        """Runs the command with the given parameters.

        If a parameter is missing, the value from the last command is used.

        Args:
            mode: The mode to run the stepper in.
            x_des: The position to move to.
            v_des: The velocity to move at.
            a_des: The acceleration to move at.
            i_des: The current to move at.
            stiffness: The stiffness to move at.
            i_feedforward: The current feedforward to move at.
            i_contact_pos: The positive contact current to move at.
            i_contact_neg: The negative contact current to move at.
        """

        with self.lock:
            if mode is not None:
                self.command.mode = mode

            if x_des is not None:
                self.command.x_des = x_des
                if self.command.mode == Mode.POS_TRAJ_INCR:
                    self.command.incr_trigger = (self.command.incr_trigger + 1) % 255

            if v_des is not None:
                self.command.v_des = v_des

            if a_des is not None:
                self.command.a_des = a_des

            if stiffness is not None:
                self.command.stiffness = stiffness

            if i_feedforward is not None:
                self.command.i_feedforward = i_feedforward

            if i_des is not None:
                if self.command.mode == Mode.CURRENT:
                    self.command.i_feedforward = i_des
                else:
                    self.logger.warning("i_des is ignored in non-current mode")

            if i_contact_pos is not None:
                self.command.i_contact_pos = i_contact_pos

            if i_contact_neg is not None:
                self.command.i_contact_neg = i_contact_neg

            payload = arr.array("B", [RPC.SET_COMMAND] + [0] * self.command.total_bytes())
            self.command.pack(payload, 1)
            reply = await self.transport.send(payload, throw_on_error=True)
            if reply[0] != RPC.REPLY_COMMAND:
                self.logger.error("Error setting command: %d", reply[0])
