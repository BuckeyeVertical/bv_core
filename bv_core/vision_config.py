"""Helpers for loading shared vision configuration."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def load_vision_config() -> dict:
    """Load ``vision_params.yaml`` from the installed package share directory."""
    vision_yaml = os.path.join(
        get_package_share_directory("bv_core"),
        "config",
        "vision_params.yaml",
    )

    with open(vision_yaml, "r") as handle:
        return yaml.safe_load(handle) or {}
