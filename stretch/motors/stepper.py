import array as arr
import asyncio
import logging
import math
from dataclasses import dataclass
from typing import Literal

from stretch.uart.packing import Bytes, byte_field, flag_field
from stretch.uart.transport import Transport
from stretch.utils.config import StepperConfig


@dataclass
class Command(Bytes):
    mode: int = byte_field("uint8")
    x_des: float = byte_field("float")
    v_des: float = byte_field("float")
    a_des: float = byte_field("float")
    stiffness: float = byte_field("float")
    i_feedforward: float = byte_field("float")
    i_contact_pos: float = byte_field("float")
    i_contact_neg: float = byte_field("float")
    incr_trigger: int = byte_field("uint8")


@dataclass
class Gains(Bytes):
    pKp_d: float = byte_field("float")
    pKi_d: float = byte_field("float")
    pKd_d: float = byte_field("float")
    pLPF: float = byte_field("float")
    pKi_limit: float = byte_field("float")
    vKp_d: float = byte_field("float")
    vKi_d: float = byte_field("float")
    vKd_d: float = byte_field("float")
    vLPF: float = byte_field("float")
    vKi_limit: float = byte_field("float")
    vTe_d: float = byte_field("float")
    iMax_pos: float = byte_field("float")
    iMax_neg: float = byte_field("float")
    phase_advance_d: float = byte_field("float")
    pos_near_setpoint_d: float = byte_field("float")
    vel_near_setpoint_d: float = byte_field("float")
    vel_status_LPF: float = byte_field("float")
    effort_LPF: float = byte_field("float")
    safety_stiffness: float = byte_field("float")
    i_safety_feedforward: float = byte_field("float")
    config: int = byte_field("uint8")

    safety_hold: bool = flag_field("config", 0)
    enable_runstop: bool = flag_field("config", 1)
    enable_sync_mode: bool = flag_field("config", 2)
    enable_guarded_mode: bool = flag_field("config", 3)
    flip_encoder_polarity: bool = flag_field("config", 4)
    flip_effort_polarity: bool = flag_field("config", 5)
    enable_vel_watchdog: bool = flag_field("config", 6)


@dataclass
class MotionLimits(Bytes):
    limit_neg: float = byte_field("float")
    limit_pos: float = byte_field("float")


@dataclass
class StatusP0(Bytes):
    mode: int = byte_field("uint8")
    effort_ticks: float = byte_field("float")
    pos: float = byte_field("double")
    vel: float = byte_field("float")
    err: float = byte_field("float")
    diag: int = byte_field("uint32")
    timestamp: int = byte_field("uint32")
    debug: float = byte_field("float")
    guarded_event: int = byte_field("uint32")

    pos_calibrated: bool = flag_field("diag", 0)  # Has a pos zero RPC been received since powerup?
    runstop_on: bool = flag_field("diag", 1)  # Is controller in runstop mode?
    near_pos_setpoint: bool = flag_field("diag", 2)  # Is position controller within `gains.pAs_d` of setpoint?
    near_vel_setpoint: bool = flag_field("diag", 3)  # Is velocity controller within `gains.vAs_d` of setpoint?
    is_moving: bool = flag_field("diag", 4)  # Is measured velocity greater than `gains.vAs_d`?
    at_current_limit: bool = flag_field("diag", 5)  # Is controller current saturated?
    is_mg_accelerating: bool = flag_field("diag", 6)  # Is controller motion generator acceleration non-zero?
    is_mg_moving: bool = flag_field("diag", 7)  # Is controller motion generator velocity non-zero?
    calibration_rcvd: bool = flag_field("diag", 8)  # Is calibration table in flash?
    in_guarded_event: bool = flag_field("diag", 9)  # Guarded event occurred during motion?
    in_safety_event: bool = flag_field("diag", 10)  # Is it forced into safety mode?
    waiting_on_sync: bool = flag_field("diag", 11)  # Command received but no sync yet?
    traj_active: bool = flag_field("diag", 12)  # Is a waypoint trajectory actively executing?
    traj_waiting_on_sync: bool = flag_field("diag", 13)  # Waiting on a sync signal before starting trajectory?
    in_sync_mode: bool = flag_field("diag", 14)  # Currently running in sync mode?
    is_trace_on: bool = flag_field("diag", 15)  # Is trace recording?


@dataclass
class StatusP1(Bytes):
    mode: int = byte_field("uint8")
    effort_ticks: float = byte_field("float")
    pos: float = byte_field("double")
    vel: float = byte_field("float")
    err: float = byte_field("float")
    diag: int = byte_field("uint32")
    timestamp: int = byte_field("uint64")
    debug: float = byte_field("float")
    guarded_event: int = byte_field("uint32")
    waypoint_traj_setpoint: float = byte_field("float")
    waypoint_traj_segment_id: int = byte_field("uint16")

    pos_calibrated: bool = flag_field("diag", 0)
    runstop_on: bool = flag_field("diag", 1)
    near_pos_setpoint: bool = flag_field("diag", 2)
    near_vel_setpoint: bool = flag_field("diag", 3)
    is_moving: bool = flag_field("diag", 4)
    at_current_limit: bool = flag_field("diag", 5)
    is_mg_accelerating: bool = flag_field("diag", 6)
    is_mg_moving: bool = flag_field("diag", 7)
    calibration_rcvd: bool = flag_field("diag", 8)
    in_guarded_event: bool = flag_field("diag", 9)
    in_safety_event: bool = flag_field("diag", 10)
    waiting_on_sync: bool = flag_field("diag", 11)
    waypoint_traj_active: bool = flag_field("diag", 12)
    waypoint_traj_waiting_on_sync: bool = flag_field("diag", 13)
    in_sync_mode: bool = flag_field("diag", 14)

    @property
    def waypoint_traj_idle(self) -> bool:
        return not self.waypoint_traj_active and not self.waypoint_traj_waiting_on_sync


@dataclass
class StatusP2(Bytes):
    mode: int = byte_field("uint8")
    effort_ticks: float = byte_field("float")
    pos: float = byte_field("double")
    vel: float = byte_field("float")
    err: float = byte_field("float")
    diag: int = byte_field("uint32")
    timestamp: int = byte_field("uint64")
    debug: float = byte_field("float")
    guarded_event: int = byte_field("uint32")
    waypoint_traj_setpoint: float = byte_field("float")
    waypoint_traj_segment_id: int = byte_field("uint16")

    pos_calibrated: bool = flag_field("diag", 0)
    runstop_on: bool = flag_field("diag", 1)
    near_pos_setpoint: bool = flag_field("diag", 2)
    near_vel_setpoint: bool = flag_field("diag", 3)
    is_moving: bool = flag_field("diag", 4)
    at_current_limit: bool = flag_field("diag", 5)
    is_mg_accelerating: bool = flag_field("diag", 6)
    is_mg_moving: bool = flag_field("diag", 7)
    calibration_rcvd: bool = flag_field("diag", 8)
    in_guarded_event: bool = flag_field("diag", 9)
    in_safety_event: bool = flag_field("diag", 10)
    waiting_on_sync: bool = flag_field("diag", 11)
    waypoint_traj_active: bool = flag_field("diag", 12)
    waypoint_traj_waiting_on_sync: bool = flag_field("diag", 13)
    in_sync_mode: bool = flag_field("diag", 14)

    @property
    def waypoint_traj_idle(self) -> bool:
        return not self.waypoint_traj_active and not self.waypoint_traj_waiting_on_sync


Protocol = Literal["P0", "P1", "P2"]


@dataclass
class Trigger(Bytes):
    value: int = byte_field("uint16")

    mark_pos: bool = flag_field("value", 0)
    reset_motion_gen: bool = flag_field("value", 1)
    board_reset: bool = flag_field("value", 2)
    write_gains_to_flash: bool = flag_field("value", 3)
    reset_pos_calibrated: bool = flag_field("value", 4)
    pos_calibrated: bool = flag_field("value", 5)
    mark_pos_on_contact: bool = flag_field("value", 6)
    enable_trace: bool = flag_field("value", 7)
    disable_trace: bool = flag_field("value", 8)


@dataclass
class Dirty:
    command: bool = False
    gains: bool = False
    trigger: bool = False
    read_gains_from_flash: bool = False
    load_test: bool = False


@dataclass
class BoardInfo(Bytes):
    board_variant: str = byte_field("string", 20)
    firmware_version: str = byte_field("string", 20)

    @property
    def hardware_id(self) -> int:
        return int(self.board_variant[-1]) if len(self.board_variant) == 9 else 0

    @property
    def protocol_version(self) -> str:
        return self.firmware_version[self.firmware_version.rfind("p") :] if self.firmware_version else ""


class RPC:
    SET_COMMAND = 1
    REPLY_COMMAND = 2
    GET_STATUS = 3
    REPLY_STATUS = 4
    SET_GAINS = 5
    REPLY_GAINS = 6
    LOAD_TEST = 7
    REPLY_LOAD_TEST = 8
    SET_TRIGGER = 9
    REPLY_SET_TRIGGER = 10
    SET_ENC_CALIB = 11
    REPLY_ENC_CALIB = 12
    READ_GAINS_FROM_FLASH = 13
    REPLY_READ_GAINS_FROM_FLASH = 14
    SET_MENU_ON = 15
    REPLY_MENU_ON = 16
    GET_STEPPER_BOARD_INFO = 17
    REPLY_STEPPER_BOARD_INFO = 18
    SET_MOTION_LIMITS = 19
    REPLY_MOTION_LIMITS = 20
    SET_NEXT_TRAJECTORY_SEG = 21
    REPLY_SET_NEXT_TRAJECTORY_SEG = 22
    START_NEW_TRAJECTORY = 23
    REPLY_START_NEW_TRAJECTORY = 24
    RESET_TRAJECTORY = 25
    REPLY_RESET_TRAJECTORY = 26
    READ_TRACE = 27
    REPLY_READ_TRACE = 28


class Mode:
    SAFETY = 0
    FREEWHEEL = 1
    HOLD = 2
    POS_PID = 3
    VEL_PID = 4
    POS_TRAJ = 5
    VEL_TRAJ = 6
    CURRENT = 7
    POS_TRAJ_INCR = 8
    POS_TRAJ_WAYPOINT = 9


class TraceType:
    STATUS = 0
    DEBUG = 1
    PRINT = 2


@dataclass
class WaypointTrajectory(Bytes):
    duration: float = byte_field("float")
    a0: float = byte_field("float")
    a1: float = byte_field("float")
    a2: float = byte_field("float")
    a3: float = byte_field("float")
    a4: float = byte_field("float")
    a5: float = byte_field("float")
    segment_id: int = byte_field("uint8")
    timestamp: float = 0.0


@dataclass
class WaypointTrajectoryResponse(Bytes):
    start_success_value: int = byte_field("uint8")
    start_error_msg: str = byte_field("string", 100)

    @property
    def start_success(self) -> bool:
        return self.start_success_value == 1


@dataclass
class DebugTrace(Bytes):
    u8_1: int = byte_field("uint8")
    u8_2: int = byte_field("uint8")
    f_1: float = byte_field("float")
    f_2: float = byte_field("float")
    f_3: float = byte_field("float")


@dataclass
class PrintTrace(Bytes):
    timestamp: int = byte_field("uint64")
    msg: str = byte_field("string", 32)
    x: float = byte_field("float")


class Stepper:
    def __init__(self, params: StepperConfig, name: str | None = None) -> None:
        self.name = params.usb[5:] if name is None else name
        self.usb = params.usb
        self.params = params

        self.motion_limits = MotionLimits(
            limit_neg=params.motion.limit_neg,
            limit_pos=params.motion.limit_pos,
        )

        self.logger = logging.getLogger(f"stepper.{self.name}")
        self.lock = asyncio.Lock()
        self.transport = Transport(usb=self.usb, name=self.name)

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
        self._board_info: BoardInfo | None = None
        self._protocol: Protocol | None = None
        self._status: StatusP0 | StatusP1 | StatusP2 | None = None

    async def startup(self) -> None:
        async with self.lock:
            await self.transport.startup()

            reply = await self.transport.send(arr.array("B", [RPC.GET_STEPPER_BOARD_INFO]))
            if reply[0] != RPC.REPLY_STEPPER_BOARD_INFO:
                raise IOError(f"Unexpected reply: {reply[0]}")
            self._board_info, _ = BoardInfo.unpack(reply[1:])

            match self._board_info.protocol_version:
                case "p0":
                    self._protocol = "P0"
                case "p1":
                    self._protocol = "P1"
                case "p2":
                    self._protocol = "P2"
                case _:
                    self.logger.error(
                        "Firmware protocol mismatch. Protocol on board is %s. Disabling device. "
                        "Please upgrade the firmware or version running on the Stretch.",
                        self._board_info.protocol_version,
                    )
                    await self.transport.stop()

            # Logs protocol.
            self.logger.info("Protocol: %s", self._protocol)

            await self.send_command(mode=Mode.SAFETY)

    @property
    def status(self) -> StatusP0 | StatusP1 | StatusP2:
        if self._status is None:
            if self._protocol is None:
                raise RuntimeError("Protocol not yet set; `startup()` needs to succeed first.")
            match self._protocol:
                case "P0":
                    self._status = StatusP0()
                case "P1":
                    self._status = StatusP1()
                case "P2":
                    self._status = StatusP2()
                case _:
                    raise ValueError(f"Invalid status type: {self._protocol}")
        return self._status

    @property
    def board_info(self) -> BoardInfo:
        assert self._board_info is not None, "Board info not read yet; call startup() first."
        return self._board_info

    async def stop(self) -> None:
        async with self.lock:
            await self.send_command(mode=Mode.SAFETY)
            await self.transport.stop()

    async def pull_gains(self) -> None:
        with self.lock:
            reply = await self.transport.send(arr.array("B", [RPC.READ_GAINS_FROM_FLASH]))
            if reply[0] != RPC.REPLY_GAINS:
                raise IOError(f"Unexpected reply: {reply[0]}")
            self.gains, _ = self.gains.unpack(reply[1:])

    async def pull_status(self) -> None:
        with self.lock:
            reply = await self.transport.send(arr.array("B", [RPC.GET_STATUS]))
            if reply[0] != RPC.REPLY_STATUS:
                raise IOError(f"Unexpected reply: {reply[0]}")
            self._status, _ = self.status.unpack(reply[1:])

    async def send_trigger(
        self,
        mark_pos: bool = False,
        reset_motion_gen: bool = False,
        board_reset: bool = False,
        write_gains_to_flash: bool = False,
        reset_pos_calibrated: bool = False,
        pos_calibrated: bool = False,
        mark_pos_on_contact: bool = False,
        enable_trace: bool = False,
        disable_trace: bool = False,
    ) -> None:
        trigger = Trigger()
        trigger.mark_pos = mark_pos
        trigger.reset_motion_gen = reset_motion_gen
        trigger.board_reset = board_reset
        trigger.write_gains_to_flash = write_gains_to_flash
        trigger.reset_pos_calibrated = reset_pos_calibrated
        trigger.pos_calibrated = pos_calibrated
        trigger.mark_pos_on_contact = mark_pos_on_contact
        trigger.enable_trace = enable_trace
        trigger.disable_trace = disable_trace

        async with self.lock:
            payload = arr.array("B", [RPC.REPLY_SET_TRIGGER] + [0] * trigger.total_bytes())
            trigger.pack(payload, 1)
            reply = await self.transport.send(payload)
            breakpoint()
            if reply[0] != RPC.REPLY_SET_TRIGGER:
                self.logger.error("Error setting trigger: %d", reply[0])

    async def send_load_test_payload(self) -> None:
        async with self.lock:
            payload = arr.array("B", [RPC.LOAD_TEST]) + self.load_test_payload
            reply = (await self.transport.send(payload))[1:]
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
            reply = await self.transport.send(payload)
            if reply[0] != RPC.REPLY_GAINS:
                self.logger.error("Error setting command: %d", reply[0])

    async def set_motion_limits(self, limit_neg: float, limit_pos: float) -> None:
        if limit_neg != self.motion_limits.limit_neg or limit_pos != self.motion_limits.limit_pos:
            with self.lock:
                self.motion_limits.limit_neg, self.motion_limits.limit_pos = limit_neg, limit_pos
                payload = arr.array("B", [RPC.SET_MOTION_LIMITS] + [0] * self.motion_limits.total_bytes())
                self.motion_limits.pack(payload, 1)
                reply = await self.transport.send(payload)
                if reply[0] != RPC.REPLY_MOTION_LIMITS:
                    self.logger.error("Error setting motion limits: %d", reply[0])

    def motor_rad_to_translate_m(self, ang: float) -> float:
        """Converts from motor radians to meters of translation.

        Args:
            ang: The angle in radians

        Returns:
            The translated motion, derived from the motor properties
        """

        cfg = self.params.chain
        return ang * cfg.pitch * cfg.sprocket_teeth / (cfg.gr_spur * math.pi * 2)

    def translate_m_to_motor_rad(self, x: float) -> float:
        """Converts from meters of translation to motor radians.

        Args:
            x: The translated motion

        Returns:
            The angle in radians, derived from the motor properties
        """

        cfg = self.params.chain
        return x * cfg.gr_spur * math.pi * 2 / (cfg.pitch * cfg.sprocket_teeth)

    async def send_command(
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
            reply = await self.transport.send(payload)
            if reply[0] != RPC.REPLY_COMMAND:
                self.logger.error("Error setting command: %d", reply[0])

    def current_to_effort_ticks(self, i_A: float) -> int:
        if self.board_info.hardware_id == 0:
            # I = Vref / (10 * R), Rs = 0.10 Ohm, Vref = 3.3V -->3.3A
            mA_per_tick = (3300 / 255) / (10 * 0.1)
        if self.board_info.hardware_id >= 1:
            # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A
            mA_per_tick = (3300 / 255) / (5 * 0.15)
        effort_ticks = (i_A * 1000.0) / mA_per_tick
        return min(255, max(-255, int(effort_ticks)))

    def effort_ticks_to_current(self, e: int) -> float:
        if self.board_info.hardware_id == 0:
            # I = Vref / (10 * R), Rs = 0.10 Ohm, Vref = 3.3V -->3.3A
            mA_per_tick = (3300 / 255) / (10 * 0.1)
        if self.board_info.hardware_id >= 1:
            # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A
            mA_per_tick = (3300 / 255) / (5 * 0.15)
        return e * mA_per_tick / 1000.0

    def current_to_effort_pct(self, i_A: float) -> float:
        # Effort_pct is defined as a percentage of the maximum allowable
        # motor winding current. Range is -100.0 to 100.0
        if i_A > 0:
            return 100 * max(0.0, min(1.0, i_A / self.gains.iMax_pos))
        return 100 * min(0.0, max(-1.0, i_A / abs(self.gains.iMax_neg)))

    def effort_pct_to_current(self, e_pct: float) -> float:
        if e_pct > 0:
            return min(1.0, e_pct / 100.0) * self.gains.iMax_pos
        return max(-1.0, e_pct / 100.0) * abs(self.gains.iMax_neg)

    def contact_thresh_to_motor_current(
        self, contact_thresh_pos: float, contact_thresh_neg: float
    ) -> tuple[float, float]:
        e_cn = contact_thresh_neg
        e_cp = contact_thresh_pos
        contact_thresh_neg, contact_thresh_pos = self.params.calibration.contact_thresh_max
        i_contact_neg = self.effort_pct_to_current(max(e_cn, contact_thresh_neg))
        i_contact_pos = self.effort_pct_to_current(min(e_cp, contact_thresh_pos))
        return i_contact_pos, i_contact_neg

    async def turn_menu_interface_on(self) -> None:
        reply = await self.transport.send(arr.array("B", [RPC.SET_MENU_ON]))
        if reply[0] != RPC.REPLY_MENU_ON:
            raise RuntimeError("Error getting chip id")

    async def menu_transaction(
        self,
        x: bytes | bytearray | memoryview | str,
        sleep_time: float = 0.01,
        do_log: bool = False,
    ) -> list[bytes]:
        if self.transport.ser is None:
            raise RuntimeError("Serial port not open")

        with self.lock:
            self.transport.ser.write(x)
            await asyncio.sleep(sleep_time)
            reply = []
            while self.transport.ser.inWaiting():
                r = self.transport.ser.readline()
                if do_log:
                    if type(r) == bytes:
                        self.logger.info(r.decode("UTF-8"))
                    else:
                        self.logger.info(r)
                reply.append(r)
            return reply

    async def turn_rpc_inferface_on(self) -> None:
        await self.menu_transaction(b"zyx")

    async def get_chip_id(self) -> str:
        await self.turn_menu_interface_on()
        cid = (await self.menu_transaction(b"b"))[0][:-2]
        await self.turn_rpc_interface_on()
        return cid.decode("utf-8")

    async def home(self, to_positive_stop: bool = True) -> None:
        contact_thresh_neg, contact_thresh_pos = self.params.calibration.contact_thresh_max

        prev_guarded = self.params.gains.enable_guarded_mode
        prev_sync = self.params.gains.enable_sync_mode

        # Sets gains.
        self.gains.enable_guarded_mode = True
        self.gains.enable_sync_mode = False
        await self.send_gains()

        # Sets trigger.
        await self.send_trigger(reset_pos_calibrated=True)

        # Move to first endstop.
        i_contact_pos, i_contact_neg = self.contact_thresh_to_motor_current(contact_thresh_pos, contact_thresh_neg)
        await self.send_command(
            mode=Mode.POS_TRAJ_INCR,
            x_des=self.translate_m_to_motor_rad(5.0 if to_positive_stop else -5.0),
            i_contact_neg=i_contact_neg,
            i_contact_pos=i_contact_pos,
        )

        # Move to second endstop.
        await self.send_command(
            mode=Mode.POS_TRAJ_INCR,
            x_des=self.translate_m_to_motor_rad(-5.0 if to_positive_stop else 5.0),
            i_contact_neg=i_contact_neg,
            i_contact_pos=i_contact_pos,
        )

        # Reverts guard mode.
        self.gains.enable_guarded_mode = prev_guarded
        self.gains.enable_sync_mode = prev_sync
        await self.send_gains()
