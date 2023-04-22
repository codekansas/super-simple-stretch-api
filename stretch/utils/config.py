import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, cast

from omegaconf import DictConfig, OmegaConf

from stretch.utils.env import get_fleet_path

logger = logging.getLogger(__name__)

FieldType = Any


def get_stretch_directory(sub_directory: str | Path | None = None) -> Path:
    base_path = get_fleet_path()
    full_path = base_path if sub_directory is None else base_path / sub_directory
    return full_path


@dataclass
class LoggingConfig:
    level: str = field(default="INFO", metadata={"help": "Logging level."})


@dataclass
class Config:
    logging: LoggingConfig = field(default=LoggingConfig(), metadata={"help": "Logging configuration"})

    @classmethod
    def load(cls, file_path: str | Path | None = None) -> "Config":
        if file_path is None:
            file_path = get_fleet_path() / "stretch_config.yaml"
        config = cast(DictConfig, OmegaConf.load(file_path))
        return OmegaConf.structured(cls(**config))

    def save(self, file_path: str | Path) -> None:
        OmegaConf.save(OmegaConf.to_container(self), file_path)
