import array as arr
import copy
import logging
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Literal

from omegaconf import MISSING

from stretch.device import Device
from stretch.uart.packing import Bytes, byte_field, flag_field, pack
from stretch.uart.transport import Transport
from stretch.utils.trajectories import PolyCoefs

logger = logging.getLogger(__name__)


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


@dataclass
class GainsParams:
    i_contact_pos: float = field(default=MISSING)
    i_contact_neg: float = field(default=MISSING)


@dataclass
class MotionParams:
    vel: float = field(default=MISSING)
    accel: float = field(default=MISSING)
    limit_pos: float = field(default=MISSING)
    limit_neg: float = field(default=MISSING)


@dataclass
class Params:
    gains: GainsParams = field(default=GainsParams())
    motion: MotionParams = field(default=MotionParams())


class Stepper(Device):
    def __init__(self, usb: str, params: Params, name: str | None = None) -> None:
        super().__init__(name=usb[5:] if name is None else name)

        self.usb = usb
        self.params = params

        self.lock = threading.RLock()
        self.transport = Transport(usb=self.usb)

        self.command = Command()
        self.board_info = BoardInfo()
        self.motion_limits = MotionLimits(
            limit_neg=params.motion.limit_neg,
            limit_pos=params.motion.limit_pos,
        )

        # The protocol of the stepper motor is determined at startup.
        self.protocol: Protocol | None = None
        self._status: StatusP0 | StatusP1 | StatusP2 | None = None

        # Waypoints, used by P1.
        self.waypoint_traj: WaypointTrajectory | None = None
        self.waypoint_traj_response: WaypointTrajectoryResponse | None = None
        self.waypoint_next_traj_response: WaypointTrajectoryResponse | None = None

        # Traces, used by P2.
        self.status_trace = copy.deepcopy(self.status)
        self.debug_trace: DebugTrace | None = None
        self.print_trace: PrintTrace | None = None
        self.n_trace_read = 0

        self.ts_last_syncd_motion: float = 0.0
        self.dirty = Dirty()
        self.trigger = Trigger()
        self.trigger_position = 0.0
        self.load_test_payload = arr.array("B", range(256)) * 4
        self.hw_valid = False
        self.gains = Gains()
        self.gains_flash = copy.deepcopy(self.gains)

    @property
    def status(self) -> StatusP0 | StatusP1 | StatusP2:
        if self._status is None:
            if self.protocol is None:
                raise RuntimeError("Protocol not yet set; need to call `startup()` first.")
            match self.protocol:
                case "P0":
                    self._status = StatusP0()
                case "P1":
                    self._status = StatusP1()
                case "P2":
                    self._status = StatusP2()
                case _:
                    raise ValueError(f"Invalid status type: {self.protocol}")
        return self._status

    def startup(self, threaded: bool = False) -> bool:
        """Starts the stepper motor.

        Args:
            threaded: If True, starts the stepper motor in a separate thread.

        Returns:
            True if the stepper motor was started successfully.
        """

        def rpc_board_info_reply(reply: arr.array) -> None:
            if reply[0] == RPC.REPLY_STEPPER_BOARD_INFO:
                self.board_info, _ = BoardInfo.unpack(reply[1:])
            else:
                self.logger.error("Error RPC_REPLY_STEPPER_BOARD_INFO, %d", reply[0])

        try:
            super().startup(threaded=threaded)

            with self.lock:
                self.hw_valid = self.transport.startup()
                if self.hw_valid:
                    self.transport.payload_out[0] = RPC.GET_STEPPER_BOARD_INFO
                    self.transport.queue_rpc(1, rpc_board_info_reply)
                    self.transport.step(exiting=False)

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
                            self.hw_valid = False
                            self.transport.stop()
                            return False

                    self.enable_safety()
                    self.dirty.gains = True
                    self.pull_status()
                    self.push_command()

                    return True
                return False

        except KeyError:
            self.hw_valid = False
            return False

    def stop(self) -> None:
        super().stop()

        if not self.hw_valid:
            return

        with self.lock:
            self.logger.debug("Shutting down")
            self.enable_safety()
            self.push_command(exiting=True)
            self.transport.stop()
            self.hw_valid = False

    def is_sync_required(self, ts_last_sync: float) -> bool:
        return self.status.in_sync_mode and self.ts_last_syncd_motion > ts_last_sync

    def push_command(self, exiting: bool = False) -> None:
        if not self.hw_valid:
            return

        with self.lock:
            if self.dirty.load_test:

                def rpc_load_test_reply(reply: arr.array) -> None:
                    if reply[0] == RPC.REPLY_LOAD_TEST:
                        d = reply[1:]
                        for i in range(1024):
                            if d[i] != (p := self.load_test_payload[(i + 1) % 1024]):
                                self.logger.debug("Load test bad data; %d != %d", d[i], p)
                        self.load_test_payload = d
                    else:
                        self.logger.error("Error RPC_REPLY_LOAD_TEST, %d", reply[0])

                self.transport.payload_out[0] = RPC.LOAD_TEST
                self.transport.payload_out[1:] = self.load_test_payload
                self.transport.queue_rpc2(1024 + 1, rpc_load_test_reply)
                self.dirty.load_test = False

            if self.dirty.trigger:

                def rpc_trigger_reply(reply: arr.array) -> None:
                    if reply[0] != RPC.REPLY_SET_TRIGGER:
                        self.logger.error("Error RPC_REPLY_SET_TRIGGER, %d", reply[0])

                self.transport.payload_out[0] = RPC.SET_TRIGGER
                sidx = self.trigger.pack(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, rpc_trigger_reply)
                self.trigger.value = 0
                self.dirty.trigger = False

            if self.dirty.gains:

                def rpc_gains_reply(reply: arr.array) -> None:
                    if reply[0] != RPC.REPLY_GAINS:
                        self.logger.error("Error RPC_REPLY_GAINS, %d", reply[0])

                self.transport.payload_out[0] = RPC.SET_GAINS
                sidx = self.gains.pack(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, rpc_gains_reply)
                self.dirty.gains = False

            if self.dirty.command:

                def rpc_command_reply(reply: arr.array) -> None:
                    if reply[0] != RPC.REPLY_COMMAND:
                        self.logger.error("Error RPC_REPLY_COMMAND, %d", reply[0])

                if self.status.in_sync_mode:
                    self.ts_last_syncd_motion = time.time()
                else:
                    self.ts_last_syncd_motion = 0
                self.transport.payload_out[0] = RPC.SET_COMMAND
                sidx = self.command.pack(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, rpc_command_reply)
                self.dirty.command = False

            self.transport.step2(exiting=exiting)

    def pull_status(self, exiting: bool = False) -> None:
        if not self.hw_valid:
            return

        def rpc_read_gains_from_flash_reply(reply: arr.array) -> None:
            if reply[0] == RPC.REPLY_READ_GAINS_FROM_FLASH:
                self.gains, _ = self.gains.unpack(reply[1:])
            else:
                self.logger.error("Error RPC_REPLY_READ_GAINS_FROM_FLASH, %d", reply[0])

        def rpc_status_reply(reply: arr.array) -> None:
            if reply[0] == RPC.REPLY_STATUS:
                self._status, _ = self.status.unpack(reply[1:])
                self.timestamp.set(self.status.timestamp)
            else:
                self.logger.error("Error RPC_REPLY_STATUS, %d", reply[0])

        with self.lock:
            if self.dirty.read_gains_from_flash:
                self.transport.payload_out[0] = RPC.READ_GAINS_FROM_FLASH
                self.transport.queue_rpc(1, rpc_read_gains_from_flash_reply)
                self.dirty.read_gains_from_flash = False

            # Queue Status RPC.
            self.transport.payload_out[0] = RPC.GET_STATUS
            self.transport.queue_rpc(1, rpc_status_reply)
            self.transport.step(exiting=exiting)

    def set_load_test(self) -> None:
        self.dirty.load_test = True

    def set_motion_limits(self, limit_neg: float, limit_pos: float) -> None:
        if limit_neg != self.motion_limits.limit_neg or limit_pos != self.motion_limits.limit_pos:

            def rpc_motion_limits_reply(reply: arr.array) -> None:
                if reply[0] != RPC.REPLY_MOTION_LIMITS:
                    self.logger.error("Error RPC_REPLY_MOTION_LIMITS, %d", reply[0])

            with self.lock:
                self.motion_limits.limit_neg, self.motion_limits.limit_pos = limit_neg, limit_pos
                self.transport.payload_out[0] = RPC.SET_MOTION_LIMITS
                sidx = self.motion_limits.pack(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, rpc_motion_limits_reply)
                self.transport.step2()

    def set_gains(self, g: Gains) -> None:
        with self.lock:
            self.gains = copy.deepcopy(g)
            self.dirty.gains = True

    def write_gains_to_flash(self) -> None:
        with self.lock:
            self.trigger.write_gains_to_flash = True
            self.dirty.trigger = True

    def read_gains_from_flash(self) -> None:
        self.dirty.read_gains_from_flash = True

    def board_reset(self) -> None:
        with self.lock:
            self.trigger.board_reset = True
            self.dirty.trigger = True

    def mark_position_on_contact(self, x: float) -> None:
        with self.lock:
            self.trigger_position = x
            self.trigger.mark_pos_on_contact = True
            self.dirty.trigger = True

    def mark_position(self, x: float) -> None:
        if self.status.mode != Mode.SAFETY:
            self.logger.warning("Can not mark position. Must be in MODE_SAFETY")
            return

        with self.lock:
            self.trigger_position = x
            self.trigger.mark_pos = True
            self.dirty.trigger = True

    def reset_motion_gen(self) -> None:
        with self.lock:
            self.trigger.reset_motion_gen = True
            self.dirty.trigger = True

    def reset_pos_calibrated(self) -> None:
        with self.lock:
            self.trigger.reset_pos_calibrated = True
            self.dirty.trigger = True

    def set_pos_calibrated(self) -> None:
        with self.lock:
            self.trigger.pos_calibrated = True
            self.dirty.trigger = True

    def enable_firmware_trace(self) -> None:
        assert isinstance(self.status, StatusP2), "Only supported on P2"
        with self.lock:
            self.trigger.enable_trace = True
            self.dirty.trigger = True

    def disable_firmware_trace(self) -> None:
        assert isinstance(self.status, StatusP2), "Only supported on P2"
        with self.lock:
            self.trigger.disable_trace = True
            self.dirty.trigger = True

    def enable_safety(self) -> None:
        self.set_command(mode=Mode.SAFETY)

    def enable_freewheel(self) -> None:
        self.set_command(mode=Mode.FREEWHEEL)

    def enable_hold(self) -> None:
        self.set_command(mode=Mode.HOLD)

    def enable_vel_pid(self) -> None:
        self.set_command(mode=Mode.VEL_PID, v_des=0)

    def enable_pos_pid(self) -> None:
        self.set_command(mode=Mode.POS_PID, x_des=self.status.pos)

    def enable_vel_traj(self) -> None:
        self.set_command(mode=Mode.VEL_TRAJ, v_des=0)

    def enable_pos_traj(self) -> None:
        self.set_command(mode=Mode.POS_TRAJ, x_des=self.status.pos)

    def enable_pos_traj_incr(self) -> None:
        self.set_command(mode=Mode.POS_TRAJ_INCR, x_des=0)

    def enable_current(self) -> None:
        self.set_command(mode=Mode.CURRENT, i_des=0)

    def enable_sync_mode(self) -> None:
        self.gains.enable_sync_mode = True
        self.dirty.gains = True

    def disable_sync_mode(self) -> None:
        self.gains.enable_sync_mode = False
        self.dirty.gains = True

    def enable_runstop(self) -> None:
        self.gains.enable_runstop = True
        self.dirty.gains = True

    def disable_runstop(self) -> None:
        self.gains.enable_runstop = False
        self.dirty.gains = True

    def enable_guarded_mode(self) -> None:
        self.gains.enable_guarded_mode = True
        self.dirty.gains = True

    def disable_guarded_mode(self) -> None:
        self.gains.enable_guarded_mode = False
        self.dirty.gains = True

    def set_command(
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
        with self.lock:
            if mode is not None:
                self.command.mode = mode

            if x_des is not None:
                self.command.x_des = x_des
                if self.command.mode == Mode.POS_TRAJ_INCR:
                    self.command.incr_trigger = (self.command.incr_trigger + 1) % 255

            if v_des is not None:
                self.command.v_des = v_des
            else:
                if mode == Mode.VEL_PID or mode == Mode.VEL_TRAJ:
                    self.command.v_des = 0
                else:
                    self.command.v_des = self.params.motion.vel

            if a_des is not None:
                self.command.a_des = a_des
            else:
                self.command.a_des = self.params.motion.accel

            if stiffness is not None:
                self.command.stiffness = max(0.0, min(1.0, stiffness))
            else:
                self.command.stiffness = 1

            if i_feedforward is not None:
                self.command.i_feedforward = i_feedforward
            else:
                self.command.i_feedforward = 0

            if i_des is not None and mode == Mode.CURRENT:
                self.command.i_feedforward = i_des

            if i_contact_pos is not None:
                self.command.i_contact_pos = i_contact_pos
            else:
                self.command.i_contact_pos = self.params.gains.i_contact_pos

            if i_contact_neg is not None:
                self.command.i_contact_neg = i_contact_neg
            else:
                self.command.i_contact_neg = self.params.gains.i_contact_neg

        self.dirty.command = True

    def poll_until(self, condition: Callable[[], bool], timeout: float = 15.0, sleep_time: float = 0.1) -> bool:
        """Poll until is moving flag is false.

        Args:
            condition: Function that returns true when the condition is met.
            timeout: Timeout in seconds.
            sleep_time: Sleep time in seconds.

        Returns:
            True if is moving flag is false, False if timeout is reached.
        """

        ts = time.time()
        self.pull_status()
        while not (value := condition()) and time.time() - ts < timeout:
            time.sleep(sleep_time)
            self.pull_status()
        return value

    def wait_while_is_moving(self, timeout: float = 15.0, sleep_time: float = 0.1) -> bool:
        return self.poll_until(lambda: not self.status.is_moving, timeout, sleep_time)

    def wait_until_at_setpoint(self, timeout: float = 15.0, sleep_time: float = 0.1) -> bool:
        return self.poll_until(lambda: self.status.near_pos_setpoint, timeout, sleep_time)

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

    def get_chip_id(self, sleep_time: float = 0.5) -> str:
        self.turn_menu_interface_on()
        time.sleep(sleep_time)
        cid = self.menu_transaction(b"b")[0][:-2]
        self.turn_rpc_interface_on()
        time.sleep(sleep_time)
        return cid.decode("utf-8")

    def read_encoder_calibration_from_flash(self, sleep_time: float = 1.0) -> list[float]:
        self.turn_menu_interface_on()
        time.sleep(sleep_time)
        self.logger.debug("Reading encoder calibration...")
        e = self.menu_transaction(b"q")[19]
        self.turn_rpc_interface_on()
        self.push_command()
        self.logger.debug("Resetting board")
        self.board_reset()
        self.push_command()
        e = e[:-4].decode("utf-8")
        enc_calib = []
        while len(e):
            ff = e.find(",")
            if ff != -1:
                enc_calib.append(float(e[:ff]))
                e = e[ff + 2 :]
            else:
                enc_calib.append(float(e))
                break
        if len(enc_calib) == 16384:
            self.logger.debug("Successful read of encoder calibration")
        else:
            self.logger.debug("Failed to read encoder calibration")
        return enc_calib

    def write_encoder_calibration_to_flash(self, data: list[float]) -> None:
        if not self.hw_valid:
            return

        def rpc_enc_calib_reply(reply: arr.array) -> None:
            if reply[0] != RPC.REPLY_ENC_CALIB:
                self.logger.debug("Error RPC_REPLY_ENC_CALIB", reply[0])

        if len(data) != 16384:
            self.logger.warning("Bad encoder data")

        else:
            self.logger.debug("Writing encoder calibration...")
            for p in range(256):
                if p % 10 == 0:
                    sys.stdout.write(".")
                    sys.stdout.flush()
                self.transport.payload_out[0] = RPC.SET_ENC_CALIB
                self.transport.payload_out[1] = p
                sidx = 2
                for i in range(64):
                    pack(data[p * 64 + i], "float", self.transport.payload_out, sidx)
                    sidx += 4
                self.transport.queue_rpc(sidx, rpc_enc_calib_reply)
                self.transport.step()

    def turn_rpc_interface_on(self) -> None:
        with self.lock:
            self.menu_transaction(b"zyx")

    def turn_menu_interface_on(self) -> None:
        if not self.hw_valid:
            return

        def rpc_menu_on_reply(reply: arr.array) -> None:
            if reply[0] != RPC.REPLY_MENU_ON:
                self.logger.error("Error RPC_REPLY_MENU_ON, %d", reply[0])

        with self.lock:
            self.transport.payload_out[0] = RPC.SET_MENU_ON
            self.transport.queue_rpc(1, rpc_menu_on_reply)
            self.transport.step()

    def log_menu(self) -> None:
        with self.lock:
            self.menu_transaction(b"m")

    def menu_transaction(
        self,
        x: bytes | bytearray | memoryview | str,
        sleep_time: float = 0.1,
        do_log: bool = False,
    ) -> list[bytes]:
        if not self.hw_valid:
            return []

        with self.lock:
            self.transport.ser.write(x)
            time.sleep(sleep_time)
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

    def enable_pos_traj_waypoint(self) -> None:
        assert isinstance(self.status, StatusP1), "Only supported on P1"
        self.set_command(mode=Mode.POS_TRAJ_WAYPOINT)

    def start_waypoint_trajectory(self, duration: float, coefs: PolyCoefs, segment_id: int) -> bool:
        """Starts execution of a waypoint trajectory on hardware.

        Args:
            duration: Duration of the trajectory in seconds.
            coefs: Polynomial coefficients for the trajectory.
            segment_id: Segment ID of the trajectory.

        Returns:
            If uC successfully initiated a new trajectory
        """

        assert isinstance(self.status, StatusP1), "Only supported on P1"

        a0, a1, a2, a3, a4, a5 = coefs
        self.waypoint_traj = WaypointTrajectory(
            duration=duration,
            a0=a0,
            a1=a1,
            a2=a2,
            a3=a3,
            a4=a4,
            a5=a5,
            segment_id=segment_id,
            timestamp=time.time(),
        )

        def rpc_start_new_traj_reply(reply: arr.array) -> None:
            if reply[0] == RPC.REPLY_START_NEW_TRAJECTORY:
                with self.lock:
                    self.waypoint_traj_response, _ = WaypointTrajectoryResponse.unpack(reply[1:])
            else:
                self.logger.error("RPC_REPLY_START_NEW_TRAJECTORY replied %d", reply[0])

        with self.lock:
            if self.waypoint_traj is not None:
                self.transport.payload_out[0] = RPC.START_NEW_TRAJECTORY
                sidx = self.waypoint_traj.pack(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, rpc_start_new_traj_reply)
            self.transport.step2()
            if self.waypoint_traj_response is None:
                return False
            if not self.waypoint_traj_response.start_success:
                self.logger.warning(
                    "start_waypoint_trajectory: %s",
                    self.waypoint_traj_response.start_error_msg.capitalize(),
                )
            return self.waypoint_traj_response.start_success > 0

    def set_next_trajectory_segment(self, duration: float, coefs: PolyCoefs, segment_id: int) -> bool:
        """Sets the next segment for the hardware to execute

        This method is generally called multiple times while the prior segment
        is executing. This provides the hardware with the next segment to
        gracefully transition across the entire spline, while allowing users to
        preempt or modify the future trajectory in real time.

        This method will return False if there is not already an segment
        executing on the uC.

        Args:
            duration: Duration of the trajectory in seconds.
            coefs: Polynomial coefficients for the trajectory.
            segment_id: Segment ID of the trajectory.

        Returns:
            True if uC successfully queued next trajectory
        """

        assert isinstance(self.status, StatusP1), "Only supported on P1"

        if self.waypoint_traj is not None:
            self.waypoint_traj.duration = duration
            self.waypoint_traj.a0 = coefs[0]
            self.waypoint_traj.a1 = coefs[1]
            self.waypoint_traj.a2 = coefs[2]
            self.waypoint_traj.a3 = coefs[3]
            self.waypoint_traj.a4 = coefs[4]
            self.waypoint_traj.a5 = coefs[5]
            self.waypoint_traj.segment_id = segment_id

        def rpc_set_next_traj_seg_reply(reply: arr.array) -> None:
            if reply[0] == RPC.REPLY_SET_NEXT_TRAJECTORY_SEG:
                with self.lock:
                    self.waypoint_next_traj_response, _ = WaypointTrajectoryResponse.unpack(reply[1:])
            else:
                self.logger.error("RPC_REPLY_SET_NEXT_TRAJECTORY_SEG replied %s", reply[0])

        with self.lock:
            if self.waypoint_traj is not None:
                self.transport.payload_out[0] = RPC.SET_NEXT_TRAJECTORY_SEG
                sidx = self.waypoint_traj.pack(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, rpc_set_next_traj_seg_reply)
            self.transport.step2()
            if self.waypoint_next_traj_response is None:
                return False
            if not self.waypoint_next_traj_response.start_success:
                self.logger.warning(
                    "set_next_trajectory_segment: %s",
                    self.waypoint_next_traj_response.start_error_msg.capitalize(),
                )
            return self.waypoint_next_traj_response.start_success > 0

    def stop_waypoint_trajectory(self) -> None:
        """Stops execution of the waypoint trajectory running in hardware."""

        assert isinstance(self.status, StatusP1), "Only supported on P1"

        def rpc_reset_traj_reply(reply: arr.array) -> None:
            if reply[0] != RPC.REPLY_RESET_TRAJECTORY:
                self.logger.error("RPC_REPLY_RESET_TRAJECTORY replied %d", reply[0])

        self._waypoint_ts = None
        with self.lock:
            self.transport.payload_out[0] = RPC.RESET_TRAJECTORY
            self.transport.queue_rpc2(1, rpc_reset_traj_reply)
            self.transport.step2()

    def read_firmware_trace(self, timeout: float = 60.0, sleep_time: float = 0.001) -> None:
        assert isinstance(self.status, StatusP2), "Only supported on P2"

        def rpc_read_firmware_trace_reply(reply: arr.array) -> None:
            if len(reply) > 0 and reply[0] == RPC.REPLY_READ_TRACE:
                self.n_trace_read = reply[1]

                if reply[2] == TraceType.STATUS:
                    self.status_trace, _ = StatusP2.unpack(reply[3:])
                    self.timestamp.set(self.status_trace.timestamp)
                elif reply[2] == TraceType.DEBUG:
                    self.debug_trace, _ = DebugTrace.unpack(reply[3:])
                elif reply[2] == TraceType.PRINT:
                    self.print_trace, _ = PrintTrace.unpack(reply[3:])
                else:
                    self.logger.error("Unrecognized trace type %d", reply[2])

            else:
                self.logger.error("RPC_REPLY_READ_TRACE")
                self.n_trace_read = 0

        with self.lock:
            self.timestamp.reset()  # Timestamp holds state, reset within lock to avoid threading issues
            self.n_trace_read = 1
            ts = time.time()

            while self.n_trace_read > 0 and time.time() - ts < timeout:
                self.transport.payload_out[0] = RPC.READ_TRACE
                self.transport.queue_rpc(1, rpc_read_firmware_trace_reply)
                self.transport.step()
                time.sleep(sleep_time)
