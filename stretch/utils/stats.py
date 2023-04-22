import logging
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque

import numpy as np


@dataclass
class Status:
    execution_time_s: float
    curr_rate_hz: float
    avg_rate_hz: float
    supportable_rate_hz: float
    min_rate_hz: float
    max_rate_hz: float
    std_rate_hz: float
    missed_loops: int
    num_loops: int


class LoopStats:
    def __init__(self, loop_name: str, target_loop_rate: float, *, n_history: int = 100, debug_freq: int = 50) -> None:
        """Track timing statistics for control loops.

        Args:
            loop_name: Name of the loop for logging purposes
            target_loop_rate: Target loop rate in Hz
            n_history: Number of loop cycles to use for calculating
                average loop rate
            debug_freq: Frequency at which to log timing statistics
        """

        self.loop_name = loop_name
        self.target_loop_rate = target_loop_rate
        self.n_history = n_history
        self.debug_freq = debug_freq

        self.ts_loop_start: float | None = None
        self.ts_loop_end: float | None = None
        self.last_ts_loop_end: float | None = None
        self.sleep_time_s = 0.0

        self.status = Status(
            execution_time_s=0.0,
            curr_rate_hz=0.0,
            avg_rate_hz=0.0,
            supportable_rate_hz=0.0,
            min_rate_hz=float("inf"),
            max_rate_hz=0.0,
            std_rate_hz=0.0,
            missed_loops=0,
            num_loops=0,
        )

        self.logger = logging.getLogger(self.loop_name)
        self.curr_rate_history: Deque[float] = deque()
        self.supportable_rate_history: Deque[float] = deque()

    def mark_loop_start(self) -> None:
        self.ts_loop_start = time.time()

    def mark_loop_end(self) -> None:
        self.status.num_loops += 1

        # First two cycles initialize vars / log
        if self.ts_loop_start is None:
            return

        if self.ts_loop_end is None:
            self.ts_loop_end = time.time()
            return

        if self.last_ts_loop_end is None:
            self.last_ts_loop_end = self.ts_loop_end
            self.ts_loop_end = time.time()
            self.status.execution_time_s = self.ts_loop_end - self.ts_loop_start
            self.status.curr_rate_hz = 1.0 / (self.ts_loop_end - self.last_ts_loop_end)
            return

        # Calculate average and supportable loop rate **must be done before marking loop end.
        if len(self.curr_rate_history) >= self.n_history:
            self.curr_rate_history.popleft()
        self.curr_rate_history.append(self.status.curr_rate_hz)

        curr_rate_history = np.array(self.curr_rate_history)
        self.status.avg_rate_hz, self.status.std_rate_hz = curr_rate_history.mean(), curr_rate_history.std()

        if len(self.supportable_rate_history) >= self.n_history:
            self.supportable_rate_history.popleft()
        self.supportable_rate_history.append(1.0 / self.status.execution_time_s)
        supportable_rate_history = np.array(self.supportable_rate_history)
        self.status.supportable_rate_hz = supportable_rate_history.mean()

        # Log timing stats **must be done before marking loop end.
        if self.status.num_loops % self.debug_freq == 0:
            self.logger.debug("--------- TimingStats %s %d -----------", self.loop_name, self.status.num_loops)
            self.logger.debug("Target rate: %f", self.target_loop_rate)
            self.logger.debug("Current rate (Hz): %f", self.status.curr_rate_hz)
            self.logger.debug("Average rate (Hz): %f", self.status.avg_rate_hz)
            self.logger.debug("Standard deviation of rate history (Hz): %f", self.status.std_rate_hz)
            self.logger.debug("Min rate (Hz): %f", self.status.min_rate_hz)
            self.logger.debug("Max rate (Hz): %f", self.status.max_rate_hz)
            self.logger.debug("Supportable rate (Hz): %f", self.status.supportable_rate_hz)
            self.logger.debug("Standard deviation of supportable rate history (Hz): %f", supportable_rate_history.std())
            self.logger.debug("Warnings: %d out of %d", self.status.missed_loops, self.status.num_loops)
            self.logger.debug("Sleep time (s): %f", self.sleep_time_s)

        # Calculate current loop rate & execution time.
        self.last_ts_loop_end = self.ts_loop_end
        self.ts_loop_end = time.time()
        self.status.execution_time_s = self.ts_loop_end - self.ts_loop_start
        self.status.curr_rate_hz = 1.0 / (self.ts_loop_end - self.last_ts_loop_end)
        self.status.min_rate_hz = min(self.status.curr_rate_hz, self.status.min_rate_hz)
        self.status.max_rate_hz = max(self.status.curr_rate_hz, self.status.max_rate_hz)

        # Calculate sleep time to achieve desired loop rate.
        self.sleep_time_s = (1 / self.target_loop_rate) - self.status.execution_time_s
        if self.sleep_time_s < 0.0:
            self.status.missed_loops += 1
            if self.status.missed_loops == 1:
                self.logger.debug(
                    "Missed target loop rate of %.2f Hz for %s. Currently %.2f Hz",
                    self.target_loop_rate,
                    self.loop_name,
                    self.status.curr_rate_hz,
                )

    def get_loop_sleep_time(self) -> float:
        return max(0.0, self.sleep_time_s)
