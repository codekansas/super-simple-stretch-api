from typing import get_args

import pytest

from stretch.uart.packing import FloatDtype, IntDtype, pack, unpack


@pytest.mark.parametrize("dtype", get_args(IntDtype))
@pytest.mark.parametrize("value", [1, 3])
def test_int_byte_packing(dtype: IntDtype, value: int) -> None:
    # Pack the value into a bytearray.
    buffer = bytearray(8)
    size = pack(value, dtype, buffer, 0)

    # Unpack the value from the bytearray.
    result, rest = unpack(dtype, buffer)

    # Verify the result.
    assert len(rest) + size == len(buffer)
    assert result == value


@pytest.mark.parametrize("dtype", get_args(FloatDtype))
@pytest.mark.parametrize("value", [1.0, 3.0])
def test_float_byte_packing(dtype: FloatDtype, value: float) -> None:
    # Pack the value into a bytearray.
    buffer = bytearray(8)
    size = pack(value, dtype, buffer, 0)

    # Unpack the value from the bytearray.
    result, rest = unpack(dtype, buffer)

    # Verify the result.
    assert len(rest) + size == len(buffer)
    assert result == pytest.approx(value)


@pytest.mark.parametrize("value", ["test", "hello!"])
def test_string_byte_packing(value: str) -> None:
    # Pack the value into a bytearray.
    buffer = bytearray(8)
    size = pack(value, "string", buffer, 0)

    # Unpack the value from the bytearray.
    result, rest = unpack("string", buffer, size)

    # Verify the result.
    assert len(rest) + size == len(buffer)
    assert result == value
