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

logger = logging.getLogger(__name__)


class TransportError(Exception):
    pass


RPC_START_NEW_RPC = 100
RPC_ACK_NEW_RPC = 101

RPC_SEND_BLOCK_MORE = 102
RPC_ACK_SEND_BLOCK_MORE = 103
RPC_SEND_BLOCK_LAST = 104
RPC_ACK_SEND_BLOCK_LAST = 105

RPC_GET_BLOCK = 106
RPC_ACK_GET_BLOCK_MORE = 107
RPC_ACK_GET_BLOCK_LAST = 108

RPC_BLOCK_SIZE = 32
RPC_DATA_SIZE = 1024


@dataclass
class Status:
    rate: float
    read_error: int
    write_error: int
    itr: int
    transaction_time_avg: int
    transaction_time_max: int


class Transport:
    def __init__(self, usb: str | None = None) -> None:
        self.usb = usb

        self.payload_out = arr.array("B", [0] * (RPC_DATA_SIZE + 1))
        self.payload_in = arr.array("B", [0] * (RPC_DATA_SIZE + 1))
        self.buf = arr.array("B", [0] * (RPC_BLOCK_SIZE * 2))

        self.write_error = 0
        self.read_error = 0
        self.itr = 0
        self.itr_time = 0.0
        self.tlast = 0.0

        self.rpc_queue: Deque[tuple[arr.array, Callable[[arr.array], None]]] = deque()
        self.rpc_queue2: Deque[tuple[arr.array, Callable[[arr.array], None]]] = deque()

        logger.debug("Starting TransportConnection on: %s", self.usb)

        try:
            self.ser = serial.Serial(self.usb, write_timeout=1.0)
            if self.ser.isOpen():
                try:
                    fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except IOError:
                    logger.error("Port %s is busy. Check if another Stretch Body process is already running" % self.usb)
                    self.ser.close()
                    self.ser = None

        except serial.SerialException as e:
            logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))
            self.ser = None

        if self.ser is None:
            logger.warning("Unable to open serial port for device %s" % self.usb)

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
                try:
                    fcntl.flock(self.ser.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except IOError:
                    logger.error("Port %s is busy. Check if another Stretch Body process is already running" % self.usb)
                    self.ser.close()
                    self.ser = None

        except serial.SerialException:
            logger.exception("Exception for device %s", self.usb)
            self.ser = None

        if self.ser is None:
            logger.warning("Unable to open serial port for device %s", self.usb)

        return self.ser is not None

    def stop(self) -> None:
        if self.ser is not None:
            logger.debug("Shutting down TransportConnection on: %s", self.usb)
            self.ser.close()
            self.ser = None

    def queue_rpc(self, n: int, reply_callback: Callable[[arr.array], None]) -> None:
        if self.ser is not None:
            self.rpc_queue.append((copy.copy(self.payload_out[:n]), reply_callback))

    def queue_rpc2(self, n: int, reply_callback: Callable[[arr.array], None]) -> None:
        if self.ser is not None:
            self.rpc_queue2.append((copy.copy(self.payload_out[:n]), reply_callback))

    def step_rpc(self, rpc: arr.array, rpc_callback: Callable[[arr.array], None]) -> None:
        """Handles a single RPC transaction.

        Args:
            rpc: The RPC to send
            rpc_callback: The callback to call when the RPC is complete
        """

        if self.ser is None:
            logger.debug("Transport Serial not present for: %s" % self.usb)
            return

        try:
            time.time()
            self.buf[0] = RPC_START_NEW_RPC
            self.framer.send_framed_data(self.buf, 1, self.ser)
            crc, nr = self.framer.receive_framed_data(self.buf, self.ser)

            if crc != 1 or self.buf[0] != RPC_ACK_NEW_RPC:
                logger.error("Transport RX Error on RPC_ACK_NEW_RPC %s %s %s", crc, nr, self.buf[0])
                raise TransportError
            ntx = 0

            while ntx < len(rpc):
                nb = min(len(rpc) - ntx, RPC_BLOCK_SIZE)  # Num bytes to send
                b = rpc[ntx : ntx + nb]
                ntx = ntx + nb
                if ntx == len(rpc):  # Last block
                    self.buf[0] = RPC_SEND_BLOCK_LAST
                    self.buf[1 : len(b) + 1] = b
                    self.framer.send_framed_data(self.buf, nb + 1, self.ser)
                    crc, nr = self.framer.receive_framed_data(self.buf, self.ser)
                    if crc != 1 or self.buf[0] != RPC_ACK_SEND_BLOCK_LAST:
                        logger.error("Transport RX Error on RPC_ACK_SEND_BLOCK_LAST %s %s %s", crc, nr, self.buf[0])
                        raise TransportError
                else:
                    self.buf[0] = RPC_SEND_BLOCK_MORE
                    self.buf[1 : len(b) + 1] = b
                    self.framer.send_framed_data(self.buf, nb + 1, self.ser)
                    crc, nr = self.framer.receive_framed_data(self.buf, self.ser)
                    if crc != 1 or self.buf[0] != RPC_ACK_SEND_BLOCK_MORE:
                        logger.error("Transport RX Error on RPC_ACK_SEND_BLOCK_MORE %s %s %s", crc, nr, self.buf[0])
                        raise TransportError

            reply = arr.array("B")
            while True:
                self.buf[0] = RPC_GET_BLOCK
                self.framer.send_framed_data(self.buf, 1, self.ser)
                crc, nr = self.framer.receive_framed_data(self.buf, self.ser)
                if crc != 1 or not (self.buf[0] == RPC_ACK_GET_BLOCK_MORE or self.buf[0] == RPC_ACK_GET_BLOCK_LAST):
                    logger.error("Transport RX Error on RPC_GET_BLOCK %s %s %s", crc, nr, self.buf[0])
                    raise TransportError
                reply = reply + self.buf[1:nr]

                if self.buf[0] == RPC_ACK_GET_BLOCK_LAST:
                    break
            rpc_callback(reply)

        except TransportError:
            logger.exception("Exception for device %s", self.usb)
            self.read_error = self.read_error + 1
            self.ser.reset_output_buffer()
            self.ser.reset_input_buffer()

        except (serial.SerialTimeoutException, serial.SerialException, TypeError):
            logger.exception("Exception for device %s", self.usb)
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
            logger.exception("Exception for device %s", self.usb)
            self.write_error += 1

        except IOError:
            logger.exception("Exception for device %s", self.usb)
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
            logger.exception("Exception for device %s", self.usb)
            self.write_error += 1

        except IOError:
            logger.exception("Exception for device %s", self.usb)
            self.read_error += 1
