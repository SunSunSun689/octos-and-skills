"""Per-robot manifest loader for the arm skill pack.

A manifest is a small JSON file describing ONE arm's grasp geometry (host-agnostic
robot description — NOT deployment paths). The skill code reads values through
this loader so adding a new arm of this class is just: write a manifest +
a `skills/<robot>/SKILL.md` descriptor. No skill/agent/bridge code changes.

Resolution precedence for each key:  explicit env (UPPER_CASE)  >  manifest value
(lower_case key)  >  built-in default. The env override keeps the launchers and
the old UR5e env-only flow working unchanged.

Point at a manifest with:  ROBOT_MANIFEST=/path/to/manifests/rebot.json
Deployment paths (MODEL_NAME, ARM_BRIDGE_URL, BALL_URL) stay in env, not the
manifest, so manifests are portable across machines.
"""
from __future__ import annotations

import json
import os

import numpy as np

_DATA = None


def _data() -> dict:
    global _DATA
    if _DATA is None:
        p = os.environ.get("ROBOT_MANIFEST")
        _DATA = json.load(open(p)) if p else {}
    return _DATA


def _raw(key: str, default):
    env = os.environ.get(key.upper())
    if env is not None:
        return env  # string from env
    d = _data()
    return d[key] if key in d else default


def s(key: str, default) -> str:
    return str(_raw(key, default))


def f(key: str, default) -> float:
    return float(_raw(key, default))


def vec(key: str, default) -> np.ndarray:
    v = _raw(key, default)
    if isinstance(v, str):
        return np.array([float(x) for x in v.split(",")])
    return np.array(v, dtype=float)


def b(key: str, default: bool) -> bool:
    v = _raw(key, default)
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "yes", "on")
    return bool(v)
