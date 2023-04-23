import array as arr
import asyncio
import fcntl
import logging
from typing import Literal, overload

import serial

from stretch.uart.cobbs_framing import CobbsFraming
from stretch.uart.transport import RPC, Status, TransportError


class AsyncTransport:
    def __init__(self, usb: str, write_timeout: float = 1.0) -> None:
        self.usb = usb
        self.write_timeout = write_timeout

        self.logger = logging.getLogger(f"transport.{usb}")

        self.lock = asyncio.Lock()
        self.buf = arr.array("B", [0] * (RPC.BLOCK_SIZE * 2))

        self.write_error = 0
        self.read_error = 0
        self.itr = 0
        self.itr_time = 0.0
        self.tlast = 0.0

        self.logger.debug("Starting connection")

        self.ser: serial.Serial | None = None
        self.framer = CobbsFraming()
        self.status = Status(
            rate=0,
            read_error=0,
            write_error=0,
            itr=0,
            transaction_time_avg=0,
            transaction_time_max=0,
        )

    async def startup(self) -> None:
        async with self.lock:
            self.ser = serial.Serial(self.usb, write_timeout=self.write_timeout)
            assert self.ser.isOpen(), "Serial port not open"
            fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)

    async def stop(self) -> None:
        assert self.ser is not None, "Serial port not open"
        async with self.lock:
            self.ser.close()
            self.ser = None

    @overload
    async def step_rpc(self, rpc: arr.array, *, timeout: float = 0.2) -> arr.array | None:
        ...

    @overload
    async def step_rpc(self, rpc: arr.array, *, timeout: float = 0.2, throw_on_error: Literal[True]) -> arr.array:
        ...

    async def step_rpc(self, rpc: arr.array, *, timeout: float = 0.2, throw_on_error: bool = False) -> arr.array | None:
        if self.ser is None:
            raise IOError("Device is not connected; run `transport.startup()` first")

        async with self.lock:
            try:
                # Sends START_NEW_RPC
                self.buf[0] = RPC.START_NEW_RPC
                self.framer.send_framed_data(self.buf, 1, self.ser)
                crc, nr = self.framer.receive_framed_data(self.buf, self.ser, timeout=timeout)

                if crc != 1 or self.buf[0] != RPC.ACK_NEW_RPC:
                    raise TransportError(f"Transport RX Error on ACK_NEW_RPC {crc} {nr} {self.buf[0]}")

                ntx = 0
                while ntx < len(rpc):
                    # Number of bytes to send
                    nb = min(len(rpc) - ntx, RPC.BLOCK_SIZE)
                    b = rpc[ntx : ntx + nb]
                    ntx = ntx + nb

                    if ntx == len(rpc):  # Last block
                        self.buf[0] = RPC.SEND_BLOCK_LAST
                        self.buf[1 : len(b) + 1] = b
                        self.framer.send_framed_data(self.buf, nb + 1, self.ser)
                        crc, nr = self.framer.receive_framed_data(self.buf, self.ser)
                        if crc != 1 or self.buf[0] != RPC.ACK_SEND_BLOCK_LAST:
                            raise TransportError(f"Transport RX Error on ACK_SEND_BLOCK_LAST {crc} {nr} {self.buf[0]}")

                    else:
                        self.buf[0] = RPC.SEND_BLOCK_MORE
                        self.buf[1 : len(b) + 1] = b
                        self.framer.send_framed_data(self.buf, nb + 1, self.ser)
                        crc, nr = self.framer.receive_framed_data(self.buf, self.ser)
                        if crc != 1 or self.buf[0] != RPC.ACK_SEND_BLOCK_MORE:
                            raise TransportError(f"Transport RX Error on ACK_SEND_BLOCK_MORE {crc} {nr} {self.buf[0]}")

                reply = arr.array("B")
                while True:
                    self.buf[0] = RPC.GET_BLOCK
                    self.framer.send_framed_data(self.buf, 1, self.ser)
                    crc, nr = self.framer.receive_framed_data(self.buf, self.ser)
                    if crc != 1 or not (self.buf[0] == RPC.ACK_GET_BLOCK_MORE or self.buf[0] == RPC.ACK_GET_BLOCK_LAST):
                        raise TransportError(f"Transport RX Error on GET_BLOCK {crc} {nr} {self.buf[0]}")
                    reply = reply + self.buf[1:nr]

                    if self.buf[0] == RPC.ACK_GET_BLOCK_LAST:
                        break

                return reply

            except TransportError:
                self.read_error = self.read_error + 1
                self.ser.reset_output_buffer()
                self.ser.reset_input_buffer()
                if throw_on_error:
                    raise
                self.logger.exception("Exception while communicating with device")

            except (serial.SerialTimeoutException, serial.SerialException, TypeError):
                self.write_error += 1
                self.ser = None
                if throw_on_error:
                    raise
                self.logger.exception("Exception while communicating with device")

        return None
