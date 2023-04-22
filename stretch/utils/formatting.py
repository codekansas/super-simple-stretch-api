import datetime
import time


def create_time_string(*, time_format: str = "%Y%m%d%H%M%S") -> str:
    """Creates a time string.

    Args:
        time_format: Format of the time string.

    Returns:
        Time string.
    """

    return time.strftime(time_format)


def format_time(dt: float) -> str:
    maybe_plural = lambda n, s: f"{n} {s}" if n != 1 else f"{n} {s[:-1]}"

    delta = datetime.timedelta(seconds=dt)
    total_seconds = int(round(delta.total_seconds()))
    parts: list[str] = [maybe_plural(total_seconds % 60, "seconds")]
    if total_seconds >= 60:
        parts += [maybe_plural((total_seconds // 60) % 60, "minutes")]
    if total_seconds >= 60 * 60:
        parts += [maybe_plural((total_seconds // (60 * 60)) % 24, "hours")]
    if total_seconds >= 60 * 60 * 24:
        parts += [maybe_plural(total_seconds // (60 * 60 * 24), "days")]
    return ", ".join(reversed(parts))
