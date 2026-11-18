#!/usr/bin/env python3
"""
policy_io.py - load the authors' TD3 policies.

Two of the four policy files (td3_gazebo_custom_policy2.zip, ..._backw.zip) were saved with numpy 1.24.4, ours has
1.23.5, so the pickled action_space cannot be deserialised ("__generator_ctor() takes from 0 to 1 positional arguments").
Stable-Baselines3 suggests `custom_objects`; we pass freshly built spaces that match the environment
(depth image 1x64x64 + 7 scalars; action = [linear 0..1, angular -1..1]).
"""
import numpy as np


def explicit_spaces():
    from gymnasium import spaces
    return {
        "observation_space": spaces.Dict({
            "image": spaces.Box(0.0, 1.0, (1, 64, 64), np.float32),
            "scalars": spaces.Box(-np.inf, np.inf, (7,), np.float32)}),
        "action_space": spaces.Box(np.array([0.0, -1.0], np.float32), np.array([1.0, 1.0], np.float32),
                                   dtype=np.float32),
    }


def load_policy(path, device="cpu"):
    from stable_baselines3 import TD3
    try:
        return TD3.load(path, device=device)
    except Exception as e:                       # noqa: BLE001  (version mismatch of pickled spaces)
        print(f"load_policy: plain load of {path} failed ({type(e).__name__}); "
              f"retrying with explicit spaces", flush=True)
        return TD3.load(path, device=device, custom_objects=explicit_spaces())
