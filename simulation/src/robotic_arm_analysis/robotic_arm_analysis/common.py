"""Shared helpers of the analysis package: paths, model loading, link point clouds, plot style."""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def _src_dir():
    return os.path.normpath(os.path.join(HERE, "..", ".."))


def _share(pkg):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(pkg)
    except Exception:
        return os.path.join(_src_dir(), pkg)


try:
    import robotic_arm_control  # noqa: F401  (installed workspace)
except ImportError:              # running from the source tree without colcon
    sys.path.insert(0, os.path.join(_src_dir(), "robotic_arm_control"))

from robotic_arm_control.kinematics import ARM_JOINTS, ArmModel  # noqa: E402
from robotic_arm_control import planner  # noqa: E402

D = math.pi / 180.0
JN = ["J1", "J2", "J3", "J4", "J5", "J6"]
JLABEL = ["J1 base", "J2 shoulder", "J3 elbow", "J4 wrist pitch", "J5 wrist roll", "J6 tool"]
ACTUATOR = ["17HS4401 + 1:14"] * 4 + ["DS3218MG"] * 2
URDF = os.path.join(_share("robotic_arm_description"), "urdf", "robotic_arm.urdf")
MESH_DIR = os.path.join(_share("robotic_arm_description"), "meshes", "visual")
TASK = os.path.join(_share("robotic_arm_control"), "config", "task.yaml")
CONTROLLERS = os.path.join(_share("robotic_arm_control"), "config", "controllers.yaml")
POWER_LIMIT_W = 9.4

# figure style: 0.85 x A4 text width (A4, 25 mm margins -> 160 mm text width -> 136 mm)
FIG_W = 0.85 * 160 / 25.4
COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]   # fixed order J1..J6
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#dddcd6"
STATE_SHADE = {"HOMING": "#f0efec", "HOME": "#ffffff", "APPROACH": "#e8f0fb", "DESCEND": "#d5e4f7", "GRASP": "#fbe6dc",
               "LIFT": "#d5e4f7", "CLASSIFY": "#fdf1d3", "MOVE_TO_BIN": "#dff3ea", "RELEASE": "#fbe6dc", "RETURN": "#eeeafa"}


def load():
    model = ArmModel(URDF)
    task = planner.load_task(TASK)
    return model, task, planner.Poses(model, task)


def setup_style():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.size": 9, "axes.labelsize": 9.5, "xtick.labelsize": 9, "ytick.labelsize": 9, "legend.fontsize": 9,
        "font.family": "DejaVu Sans", "axes.edgecolor": MUTED, "axes.labelcolor": INK, "text.color": INK,
        "xtick.color": MUTED, "ytick.color": MUTED, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
        "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.5, "legend.frameon": False,
        "figure.facecolor": "white", "axes.facecolor": "white", "savefig.dpi": 200, "pdf.fonttype": 42,
        "axes.axisbelow": True,
    })
    return plt


def save(fig, out_dir, name):
    os.makedirs(out_dir, exist_ok=True)
    fig.savefig(os.path.join(out_dir, name + ".pdf"), bbox_inches="tight", pad_inches=0.02)
    fig.savefig(os.path.join(out_dir, name + ".png"), dpi=200, bbox_inches="tight", pad_inches=0.02)
    import matplotlib.pyplot as plt
    plt.close(fig)
    print("  wrote", name + ".pdf/.png")


def read_stl_vertices(path):
    with open(path, "rb") as f:
        f.seek(80)
        n = int(np.frombuffer(f.read(4), dtype="<u4")[0])
        rec = np.frombuffer(f.read(n * 50), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    return rec["v"].reshape(-1, 3).astype(float)


_PTS = {}


def link_points(model, n=3000, seed=0):
    if _PTS:
        return _PTS
    """random subsample of the ORIGINAL (visual) mesh vertices of every arm link, in the link frame.
    Used for the clearance check (true geometry, not the convex hulls)."""
    rng = np.random.default_rng(seed)
    pts = {}
    for ln in ["base_link"] + model.link_order:
        v = np.unique(np.round(read_stl_vertices(os.path.join(MESH_DIR, ln + ".STL")), 5), axis=0)
        pts[ln] = v[rng.choice(len(v), size=min(n, len(v)), replace=False)]
    _PTS.update(pts)
    return pts


def link_transforms(model, q):
    """{link: 4x4} for base_link and the six moving links."""
    Ts, _, Tt = model.chain(q)
    out = {"base_link": model.joints["world_to_base"]["T"]}
    out.update(dict(zip(model.link_order, Ts)))
    out["tcp"] = Tt
    return out


def read_run(run_dir, name="cycle.csv"):
    """reads a log written by offline_sim.py or by the Gazebo logger node. returns dict of numpy arrays."""
    import csv
    path = os.path.join(run_dir, name)
    with open(path) as f:
        rows = list(csv.DictReader(f))
    out = {"t": np.array([float(r["t"]) for r in rows]), "state": [r["state"] for r in rows]}
    for key in ("pos", "vel", "eff", "des"):
        if f"{key}_J1" in rows[0]:
            out[key] = np.array([[float(r[f"{key}_J{i}"]) for i in range(1, 7)] for r in rows])
    for key in ("cycle", "payload", "jaw"):
        if key in rows[0]:
            out[key] = np.array([float(r[key]) for r in rows])
    meta = os.path.join(run_dir, "meta.json")
    out["meta"] = json.load(open(meta)) if os.path.exists(meta) else {"source": "unknown"}
    return out


def state_spans(t, state):
    spans, s0, i0 = [], state[0], 0
    for i in range(1, len(state)):
        if state[i] != s0:
            spans.append((t[i0], t[i], s0))
            s0, i0 = state[i], i
    spans.append((t[i0], t[-1], s0))
    return spans
