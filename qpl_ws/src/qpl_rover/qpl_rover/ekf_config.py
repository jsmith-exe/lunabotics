"""Loader for the merged EKF config files.

ekf_local_params.yaml and ekf_global_params.yaml each hold three sections:
`common`, `sim` and `rover`. The environment section is layered over `common`,
so a key set in `sim` or `rover` wins and everything else is shared.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory


def load_ekf_params(filename: str, use_sim_time: bool) -> dict:
    """Return the merged parameter dict for one EKF node."""
    path = os.path.join(
        get_package_share_directory("qpl_rover"), "config", filename
    )
    with open(path) as handle:
        config = yaml.safe_load(handle)

    environment = "sim" if use_sim_time else "rover"
    for section in ("common", environment):
        if section not in config:
            raise RuntimeError(f"{filename}: missing '{section}' section")

    params = {**config["common"], **config[environment]}
    params["use_sim_time"] = use_sim_time

    print(f"[ekf] {filename}: '{environment}' profile, {len(params)} parameters")

    return params
