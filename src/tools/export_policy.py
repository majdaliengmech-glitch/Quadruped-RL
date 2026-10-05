#!/usr/bin/env python3
"""Export a Brax PPO params pickle (Joystick_model/joystick_params.pkl) to the
NumPy .npz read by policy_node. Run in the training venv (needs jax/brax to unpickle).
usage: python export_policy.py <params.pkl> <out.npz>"""
import pickle, sys
import numpy as np

src, out = sys.argv[1], sys.argv[2]
d = pickle.load(open(src, "rb"))
norm, pol = (d["params"] if isinstance(d, dict) else d)[:2]
layers = pol["params"]
w = {"obs_mean": np.asarray(norm.mean["state"]), "obs_std": np.asarray(norm.std["state"])}
for i in range(4):
    w[f"W{i}"] = np.asarray(layers[f"hidden_{i}"]["kernel"])
    w[f"b{i}"] = np.asarray(layers[f"hidden_{i}"]["bias"])
np.savez(out, **w)
print({k: v.shape for k, v in w.items()})
