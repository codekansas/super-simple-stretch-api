import array as arr
import copy
import fcntl
import logging
import time
from collections import deque
from dataclasses import dataclass
from typing import Callable, Deque

import serial

from stretch.uart.cobbs_framing import CobbsFraming


class TransportError(Exception):
    pass


class RPC:
    START_NEW_RPC = 100
    ACK_NEW_RPC = 101
    SEND_BLOCK_MORE = 102
    ACK_SEND_BLOCK_MORE = 103
    SEND_BLOCK_LAST = 104
    ACK_SEND_BLOCK_LAST = 105
    GET_BLOCK = 106
    ACK_GET_BLOCK_MORE = 107
    ACK_GET_BLOCK_LAST = 108
    BLOCK_SIZE = 32
    DATA_SIZE = 1024


@dataclass
class Status:
    rate: float
    read_error: int
    write_error: int
    itr: int
    transaction_time_avg: int
    transaction_time_max: int


class Transport:
    def __init__(self, usb: str) -> None:
        self.usb = usb
        self.logger = logging.getLogger(f"transport.{usb}")

        self.payload_out = arr.array("B", [0] * (RPC.DATA_SIZE + 1))
        self.payload_in = arr.array("B", [0] * (RPC.DATA_SIZE + 1))
        self.buf = arr.array("B", [0] * (RPC.BLOCK_SIZE * 2))

        self.write_error = 0
        self.read_error = 0
        self.itr = 0
        self.itr_time = 0.0
        self.tlast = 0.0

        self.rpc_queue: Deque[tuple[arr.array, Callable[[arr.array], None]]] = deque()
        self.rpc_queue2: Deque[tuple[arr.array, Callable[[arr.array], None]]] = deque()

        self.logger.debug("Starting connection")

        try:
            self.ser = serial.Serial(self.usb, write_timeout=1.0)
            if self.ser.isOpen():
                fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)

        except serial.SerialException:
            self.logger.exception("Exception while opening serial port")
            self.ser = None

        except IOError:
            self.logger.exception("Device is busy. Check if another Stretch Body process is already running.")
            self.ser.close()
            self.ser = None

        if self.ser is None:
            self.logger.warning("Unable to open serial port for device.")

        self.framer = CobbsFraming()
        self.status = Status(
            rate=0,
            read_error=0,
            write_error=0,
            itr=0,
            transaction_time_avg=0,
            transaction_time_max=0,
        )

    def startup(self) -> bool:
        try:
            self.ser = serial.Serial(self.usb, write_timeout=1.0)
            if self.ser.isOpen():
                fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)

        except serial.SerialException:
            self.logger.exception("Exception while opening serial port")
            self.ser = None

        except IOError:
            self.logger.exception("Port is busy. Check if another Stretch Body process is already running")
            self.ser.close()
            self.ser = None

        if self.ser is None:
            self.logger.warning("Unable to open serial port for device.")

        return self.ser is not None

    def stop(self) -> None:
        if self.ser is not None:
            self.logger.debug("Shutting down")
            self.ser.close()
            self.ser = None

    def queue_rpc(self, n: int, reply_callback: Callable[[arr.array], None]) -> None:
        if self.ser is not None:
            self.rpc_queue.append((copy.copy(self.payload_out[:n]), reply_callback))

    def queue_rpc2(self, n: int, reply_callback: Callable[[arr.array], None]) -> None:
        if self.ser is not None:
            self.rpc_queue2.append((copy.copy(self.payload_out[:n]), reply_callback))

    def step_rpc(self, rpc: arr.array, rpc_callback: Callable[[arr.array], None], timeout: float = 0.2) -> None:
        """Handles a single RPC transaction.

        Args:
            rpc: The RPC to send
            rpc_callback: The callback to call when the RPC is complete
            timeout: The timeout for the RPC
        """

        if self.ser is None:
            self.logger.debug("Transport serial not present")
            return

        try:
            time.time()
            self.buf[0] = RPC.START_NEW_RPC
            self.framer.send_framed_data(self.buf, 1, self.ser)
            crc, nr = self.framer.receive_framed_data(self.buf, self.ser, timeout=timeout)

            if crc != 1 or self.buf[0] != RPC.ACK_NEW_RPC:
                raise TransportError(f"Transport RX Error on ACK_NEW_RPC {crc} {nr} {self.buf[0]}")
            ntx = 0

            while ntx < len(rpc):
                nb = min(len(rpc) - ntx, RPC.BLOCK_SIZE)  # Number of bytes to send
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
            rpc_callback(reply)

        except TransportError:
            self.logger.exception("Exception while communicating with device")
            self.read_error = self.read_error + 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()

        except (serial.SerialTimeoutException, serial.SerialException, TypeError):
            self.logger.exception("Exception while communicating with device")
            self.write_error += 1
            self.ser = None

    def step(self, exiting: bool = False) -> None:
        if not self.ser:
            return

        if exiting:
            time.sleep(0.1)
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()

        try:
            self.itr += 1
            self.itr_time = time.time() - self.tlast
            self.tlast = time.time()

            while len(self.rpc_queue):
                rpc, reply_callback = self.rpc_queue.popleft()
                self.step_rpc(rpc, reply_callback)

        except serial.SerialTimeoutException:
            self.logger.exception("Exception while communicating with device")
            self.write_error += 1

        except IOError:
            self.logger.exception("Exception while communicating with device")
            self.read_error += 1

        # Update status
        if self.itr_time != 0:
            self.status.rate = 1 / self.itr_time
        else:
            self.status.rate = 0.0
        self.status.read_error = self.read_error
        self.status.write_error = self.write_error
        self.status.itr = self.itr

    def step2(self, exiting: bool = False) -> None:
        if not self.ser:
            return

        if exiting:
            time.sleep(0.1)
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()

        try:
            while self.rpc_queue2:
                rpc, reply_callback = self.rpc_queue2.popleft()
                self.step_rpc(rpc, reply_callback)

        except serial.SerialTimeoutException:
            self.logger.exception("Exception while communicating with device")
            self.write_error += 1

        except IOError:
            self.logger.exception("Exception while communicating with device")
            self.read_error += 1
