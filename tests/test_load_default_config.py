from pathlib import Path

import pytest
from omegaconf import OmegaConf

from stretch.utils.config import Config


def test_load_default_config() -> None:
    Config.load()  # This should fail if there is an error.


def test_load_bad_config(tmpdir: Path) -> None:
    cfg_path = tmpdir / "config.yaml"
    cfg = OmegaConf.create({"test": 1})
    with open(cfg_path, "w", encoding="utf-8") as f:
        OmegaConf.save(cfg, f)
    with pytest.raises(Exception):
        Config.load(file_path=cfg_path)
