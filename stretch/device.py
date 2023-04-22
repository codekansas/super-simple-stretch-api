import logging
import threading
import time

from stretch.utils.stats import LoopStats


class DeviceTimestamp:
    def __init__(self) -> None:
        self.timestamp_first: float | None = None
        self.timestamp_last: float | None = None
        self.timestamp_base: float = 0.0
        self.ts_start = time.time()

    def reset(self) -> None:
        self.timestamp_first = None
        self.timestamp_last = None
        self.timestamp_base = 0.0
        self.ts_start = time.time()

    def set(self, ts: float) -> float:
        """Take a timestamp from a uC in uS and put in terms of system clock.

        Args:
            ts: The timestamp from the uC in uS

        Returns:
            The timestamp in seconds
        """

        if self.timestamp_first is None or self.timestamp_last is None:
            self.timestamp_last = ts
            self.timestamp_first = ts
        if ts - self.timestamp_last < 0:  # Rollover
            self.timestamp_base = self.timestamp_base + 0xFFFFFFFF
        self.timestamp_last = ts
        s = (self.timestamp_base + ts - self.timestamp_first) / 1000000.0
        return self.ts_start + s


class Device:
    def __init__(self, name: str | None = None, timeout: float | None = None) -> None:
        self.name = name
        self.timeout = timeout
        self.logger = logging.getLogger("device" if self.name is None else f"device.{self.name}")

        self.timestamp = DeviceTimestamp()
        self.thread_rate_hz = 25.0
        self.thread_stats: LoopStats | None = None
        self.thread: threading.Thread | None = None
        self.thread_shutdown_flag = threading.Event()

    def startup(self, threaded: bool = False) -> bool:
        if threaded:
            if self.thread is not None:
                self.thread_shutdown_flag.set()
                self.thread.join(1)
            self.thread_stats = LoopStats(
                loop_name="{0}_thread".format(self.name),
                target_loop_rate=self.thread_rate_hz,
            )
            self.thread = threading.Thread(target=self._thread_target)
            self.thread_shutdown_flag.clear()
            self.thread.start()
        return True

    def stop(self) -> None:
        if self.thread is not None:
            self.thread_shutdown_flag.set()
            self.thread.join(self.timeout)
        self.thread = None

    def push_command(self) -> None:
        pass

    def pull_status(self) -> None:
        pass

    def _thread_loop(self) -> None:
        self.pull_status()

    def _thread_target(self) -> None:
        if (thread_stats := self.thread_stats) is None:
            return

        self.logger.debug("Starting %s", thread_stats.loop_name)

        while not self.thread_shutdown_flag.is_set():
            thread_stats.mark_loop_start()
            self._thread_loop()
            thread_stats.mark_loop_end()
            if not self.thread_shutdown_flag.is_set():
                time.sleep(thread_stats.get_loop_sleep_time())

        self.logger.debug("Shutting down %s", thread_stats.loop_name)
