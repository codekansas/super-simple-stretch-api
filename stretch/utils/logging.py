import logging
import sys

from stretch.utils.colors import Color, colorize, get_colorize_parts
from stretch.utils.env import is_debugging


class ColoredFormatter(logging.Formatter):
    """Defines a custom formatter for displaying logs."""

    RESET_SEQ = "\033[0m"
    COLOR_SEQ = "\033[1;%dm"
    BOLD_SEQ = "\033[1m"

    COLORS: dict[str, Color] = {
        "WARNING": "yellow",
        "INFOALL": "magenta",
        "INFO": "cyan",
        "DEBUG": "grey",
        "CRITICAL": "yellow",
        "FATAL": "red",
        "ERROR": "red",
    }

    def __init__(self, *, prefix: str | None = None, use_color: bool = True) -> None:
        asc_start, asc_end = get_colorize_parts("grey")
        message = "{levelname:^19s} " + asc_start + "{asctime}" + asc_end + " [{name}] {message}"
        if prefix is not None:
            message = colorize(prefix, "white") + " " + message
        super().__init__(message, style="{", datefmt="%Y-%m-%d %H:%M:%S")

        self.use_color = use_color

    def format(self, record: logging.LogRecord) -> str:
        levelname = record.levelname

        if levelname == "DEBUG":
            record.levelname = ""
        else:
            if self.use_color and levelname in self.COLORS:
                record.levelname = colorize(levelname, self.COLORS[levelname], bold=True)
        return logging.Formatter.format(self, record)


def configure_logging(*, prefix: str | None = None) -> None:
    """Instantiates print logging, to either stdout or tqdm.

    Args:
        prefix: An optional prefix to add to the logger
    """

    root_logger = logging.getLogger()
    while root_logger.hasHandlers():
        root_logger.removeHandler(root_logger.handlers[0])
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(ColoredFormatter(prefix=prefix))
    root_logger.addHandler(handler)
    root_logger.setLevel(logging.DEBUG if is_debugging() else logging.INFO)

    # Avoid junk logs from other libraries.
    logging.getLogger("matplotlib").setLevel(logging.WARNING)
    logging.getLogger("PIL").setLevel(logging.WARNING)
