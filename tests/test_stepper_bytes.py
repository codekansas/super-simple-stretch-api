from typing import Type

import pytest

from stretch.motors.stepper import Command, Gains, StatusP0, StatusP1, StatusP2, Trigger, WaypointTrajectory
from stretch.uart.packing import Bytes


@pytest.mark.parametrize("cls", [Command, Gains, StatusP0, StatusP1, StatusP2, Trigger, WaypointTrajectory])
def test_bytes(cls: Type[Bytes]) -> None:
    cmd = cls()
    total_bytes = cmd.total_bytes()
    s = bytearray(total_bytes)
    size = cmd.pack(s, 0)
    assert size == total_bytes
    rcmd, sout = cls.unpack(s)
    assert len(sout) == 0
    assert rcmd == cmd


def test_gains() -> None:
    gains = Gains()
    gains.safety_hold = True
    gains.enable_sync_mode = True
    total_bytes = gains.total_bytes()
    s = bytearray(total_bytes)
    size = gains.pack(s, 0)
    assert size == total_bytes
    rgains, sout = Gains.unpack(s)
    assert len(sout) == 0
    assert rgains == gains
