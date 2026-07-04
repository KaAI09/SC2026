"""Driving-profile loader: the single offline -> online contract.

A profile YAML captures the combination the offline evaluation selected for a
given track: which perception mode + parameters and which controller + gains.
The online perception_node and driving_node load this one file instead of
carrying dozens of separately-tuned ROS parameters, so "what offline picked" is
exactly "what the car runs".

    profile = load_profile('profiles/track2025.yaml')
    p_over = section(profile, 'perception')   # {mode, roi_top_frac, ...}
    c_over = section(profile, 'control')       # {controller, kp, kd, ...}

Schema (all keys optional; unspecified -> preset defaults):
    name: <str>
    perception: { mode: <str>, <lane_core.Cfg field>: <value>, ... }
    control:    { controller: <str>, <control_core.CtrlCfg field>: <value>, ... }
"""
import yaml


def load_profile(path):
    """Read a profile YAML. Returns {} for an empty file."""
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f) or {}


def section(profile, key):
    """Return a shallow copy of profile[key] as a plain dict ({} if absent)."""
    d = (profile or {}).get(key) or {}
    return dict(d)
