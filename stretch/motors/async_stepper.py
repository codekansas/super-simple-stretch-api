import array as arr
import asyncio
import logging

from stretch.motors.stepper import RPC, BoardInfo, Command, Gains, Mode, Protocol
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

        # Test payload.
        self.load_test_payload = arr.array("B", range(256)) * 4

        # Copies gains from stepper configuration.
        self.gains = Gains(
            pKp_d=params.gains.pKp_d,
            pKi_d=params.gains.pKi_d,
            pKd_d=params.gains.pKd_d,
            pLPF=params.gains.pLPF,
            pKi_limit=params.gains.pKi_limit,
            vKp_d=params.gains.vKp_d,
            vKi_d=params.gains.vKi_d,
            vKd_d=params.gains.vKd_d,
            vLPF=params.gains.vLPF,
            vKi_limit=params.gains.vKi_limit,
            vTe_d=params.gains.vTe_d,
            iMax_pos=params.gains.iMax_pos,
            iMax_neg=params.gains.iMax_neg,
            phase_advance_d=params.gains.phase_advance_d,
            pos_near_setpoint_d=params.gains.pos_near_setpoint_d,
            vel_near_setpoint_d=params.gains.vel_near_setpoint_d,
            vel_status_LPF=params.gains.vel_status_LPF,
            effort_LPF=params.gains.effort_LPF,
            safety_stiffness=params.gains.safety_stiffness,
            i_safety_feedforward=params.gains.i_safety_feedforward,
            safety_hold=params.gains.safety_hold,
            enable_runstop=params.gains.enable_runstop,
            enable_sync_mode=params.gains.enable_sync_mode,
            enable_guarded_mode=params.gains.enable_guarded_mode,
            flip_encoder_polarity=params.gains.flip_encoder_polarity,
            flip_effort_polarity=params.gains.flip_effort_polarity,
            enable_vel_watchdog=params.gains.enable_vel_watchdog,
        )

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

    async def send_load_test_payload(self) -> None:
        async with self.lock:
            payload = arr.array("B", [RPC.LOAD_TEST]) + self.load_test_payload
            reply = (await self.transport.send(payload, throw_on_error=True))[1:]
            for i in range(1024):
                if reply[i] != (p := self.load_test_payload[(i + 1) % 1024]):
                    self.logger.warning("Load test bad data; %d != %d", reply[i + 1], p)
                    break
            else:
                self.load_test_payload = reply

    async def send_gains(self) -> None:
        async with self.lock:
            payload = arr.array("B", [RPC.SET_GAINS] + [0] * self.gains.total_bytes())
            self.gains.pack(payload, 1)
            reply = await self.transport.send(payload, throw_on_error=True)
            if reply[0] != RPC.REPLY_GAINS:
                self.logger.error("Error setting command: %d", reply[0])

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

        async with self.lock:
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
