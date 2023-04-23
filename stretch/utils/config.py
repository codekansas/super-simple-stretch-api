import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, cast

from omegaconf import MISSING, DictConfig, OmegaConf

from stretch.utils.env import get_device_config_path

logger = logging.getLogger(__name__)

FieldType = Any


@dataclass
class GainsConfig:
    effort_LPF: float = field(default=MISSING)
    iMax_neg: float = field(default=MISSING)
    iMax_pos: float = field(default=MISSING)
    i_contact_neg: float = field(default=MISSING)
    i_contact_pos: float = field(default=MISSING)
    i_safety_feedforward: float = field(default=MISSING)
    pKd_d: float = field(default=MISSING)
    pKi_d: float = field(default=MISSING)
    pKi_limit: float = field(default=MISSING)
    pKp_d: float = field(default=MISSING)
    pLPF: float = field(default=MISSING)
    phase_advance_d: float = field(default=MISSING)
    pos_near_setpoint_d: float = field(default=MISSING)
    safety_stiffness: float = field(default=MISSING)
    vKd_d: float = field(default=0.0)
    vKi_d: float = field(default=MISSING)
    vKp_d: float = field(default=MISSING)
    vKi_limit: float = field(default=MISSING)
    vLPF: float = field(default=MISSING)
    vTe_d: float = field(default=MISSING)
    vel_near_setpoint_d: float = field(default=MISSING)
    vel_status_LPF: float = field(default=MISSING)
    safety_hold: bool = field(default=False)
    enable_guarded_mode: bool = field(default=False)
    enable_runstop: bool = field(default=True)
    enable_sync_mode: bool = field(default=True)
    enable_vel_watchdog: bool = field(default=True)
    flip_effort_polarity: bool = field(default=True)
    flip_encoder_polarity: bool = field(default=True)


@dataclass
class MotionConfig:
    vel: float = field(default=MISSING)
    acc: float = field(default=MISSING)
    limit_pos: float = field(default=0.0)
    limit_neg: float = field(default=0.0)


@dataclass
class ChainConfig:
    sprocket_teeth: int = field(default=MISSING)
    pitch: float = field(default=MISSING)
    gr_spur: float = field(default=MISSING)


@dataclass
class CalibrationConfig:
    range_bounds: tuple[float, float] = field(default=MISSING)
    contact_thresh_calibration_margin: float = field(default=MISSING)
    contact_thresh_max: tuple[float, float] = field(default=MISSING)


@dataclass
class StepperConfig:
    usb: str = field(default=MISSING)
    gains: GainsConfig = field(default=GainsConfig())
    motion: MotionConfig = field(default=MotionConfig())
    chain: ChainConfig = field(default=ChainConfig())
    calibration: CalibrationConfig = field(default=CalibrationConfig())
    rated_current: float = field(default=MISSING)
    holding_torque: float = field(default=MISSING)


@dataclass
class LoggingConfig:
    level: str = field(default="INFO")


@dataclass
class Config:
    logging: LoggingConfig = field(default=LoggingConfig())
    stepper: dict[str, StepperConfig] = field(default_factory=dict)

    @classmethod
    def load(cls, *, file_path: str | Path | None = None) -> "Config":
        file_path = get_device_config_path() if file_path is None else Path(file_path)
        if not file_path.exists():
            raise ValueError(f"Config file not found: '{file_path}'")
        return cls(**cast(DictConfig, OmegaConf.merge(OmegaConf.structured(cls), OmegaConf.load(file_path))))

    @classmethod
    def save(cls, cfg: "Config", *, file_path: str | Path | None = None) -> "Config":
        file_path = get_device_config_path() if file_path is None else Path(file_path)
        OmegaConf.save(cfg, file_path, resolve=True)
        return cls.load(file_path=file_path)
