"""Defines any core environment variables used by the Stretch.

In order to keep all environment variables in one place, so that they can be
easily referenced, don't use `os.environ` or `os.getenv` outside of this file.
Instead, add a new accessor function to this file.
"""

import os
from pathlib import Path


class _StrEnvVar:
    def __init__(self, key: str) -> None:
        self.key = key

    def get(self) -> str:
        value = self.maybe_get()
        assert value is not None, f"Value for {self.key} environment variable is not set"
        return value

    def maybe_get(self) -> str | None:
        return os.environ.get(self.key)

    def set(self, value: str) -> None:
        os.environ[self.key] = value


class _StrEnvVarWithDefault(_StrEnvVar):
    def __init__(self, key: str, default: str) -> None:
        super().__init__(key)

        self.default = default

    def get(self) -> str:
        return self.maybe_get()

    def maybe_get(self) -> str:
        return os.environ.get(self.key, self.default)


class _BoolEnvVar:
    def __init__(self, key: str) -> None:
        self.key = key

    def get(self) -> bool:
        value = self.maybe_get()
        assert value is not None, f"Value for {self.key} environment variable is not set"
        return value

    def maybe_get(self) -> bool | None:
        value = os.environ.get(self.key)
        if value is None:
            return None
        return value.lower() in ["true", "1"]

    def set(self, value: bool) -> None:
        os.environ[self.key] = "1" if value else "0"


class _BoolEnvVarWithDefault(_BoolEnvVar):
    def __init__(self, key: str, default: bool) -> None:
        super().__init__(key)

        self.default = default

    def get(self) -> bool:
        return self.maybe_get()

    def maybe_get(self) -> bool:
        value = os.environ.get(self.key)
        if value is None:
            return self.default
        return value.lower() in ["true", "1"]


class _PathEnvVar:
    def __init__(self, key: str, *, suffix: str | None = None) -> None:
        self.key = key
        self.suffix = suffix

    def get(self) -> Path:
        value = self.maybe_get()
        assert value is not None, f"Value for {self.key} environment variable is not set"
        return value

    def maybe_get(self) -> Path | None:
        value = Path(os.environ[self.key]).resolve() if self.key in os.environ else None
        if value is not None and self.suffix is not None:
            value = value / self.suffix
        return value

    def set(self, value: Path) -> None:
        os.environ[self.key] = str(value.resolve())


class _PathEnvVarWithDefault(_PathEnvVar):
    def __init__(self, key: str, default: Path, *, suffix: str | None = None) -> None:
        super().__init__(key, suffix=suffix)

        self.default = default

    def get(self) -> Path:
        return self.maybe_get()

    def maybe_get(self) -> Path:
        return Path(os.environ[self.key]).resolve() if self.key in os.environ else self.default


# Enable or disable debugging mode.
Debugging = _BoolEnvVarWithDefault("DEBUGGING", False)
is_debugging = Debugging.get
set_debugging = Debugging.set

# The Unix DISPLAY environment variable.
Display = _StrEnvVar("DISPLAY")
get_display = Display.maybe_get
set_display = Display.set

# The fleet ID for the current robot.
FleetID = _StrEnvVarWithDefault("HELLO_FLEET_ID", "stretch-re1-001")
get_fleet_id = FleetID.get
set_fleet_id = FleetID.set

# The directory containing the fleet configuration files.
FleetPath = _PathEnvVarWithDefault("HELLO_FLEET_PATH", default=Path("/tmp/"))
get_fleet_path = FleetPath.get
set_fleet_path = FleetPath.set
