#!/usr/bin/env python3
"""Clearance along the simulated path: arm to table, arm (and carried cube) to pick stand / bins, arm to itself.

Geometry = subsampled vertices of the original CAD meshes (not the convex collision hulls) plus the jaws and the
carried cube; resolution about 3 mm.  Self-distance is taken between links that are not neighbours in the chain.
Two pairs are left out because they are 16 to 34 mm apart by construction in every pose (they are joined through a
short link and cannot collide): forearm / wrist_roll and wrist_pitch / tool_flange.
"""
import numpy as np

from .common import COLORS, FIG_W, STATE_SHADE, link_points, link_transforms, load, planner, read_run, save, setup_style, state_spans

try:
    from scipy.spatial import cKDTree
except ImportError:                                   # slower fallback without scipy
    cKDTree = None


def _box_points(size, center, n=6):
    s = np.asarray(size) / 2
    g = np.linspace(-1, 1, n)
    pts = []
    for ax in range(3):
        for sign in (-1, 1):
            a, b = np.meshgrid(g, g)
            p = np.zeros((n * n, 3))
            p[:, ax] = sign
            p[:, (ax + 1) % 3], p[:, (ax + 2) % 3] = a.ravel(), b.ravel()
            pts.append(p * s + center)
    return np.vstack(pts)


def _min_dist(A, B):
    if cKDTree is not None:
        return float(cKDTree(B).query(A)[0].min())
    return float(min(np.sqrt(((B[None, :, :] - a[None, None, :]) ** 2).sum(-1)).min() for a in A[::4]))


def _box_dist(P, lo, hi):
    """signed distance of points to an axis-aligned box (negative inside)."""
    d = np.maximum(np.maximum(lo - P, P - hi), 0)
    outside = np.linalg.norm(d, axis=1)
    inside = np.max(np.maximum(lo - P, P - hi), axis=1)
    return np.where(outside > 0, outside, inside)


def compute(run_dir, hz=10.0):
    model, task, poses = load()
    run = read_run(run_dir)
    pts = link_points(model, 4000)
    tool = model.link_order[-1]
    structural = {("forearm_link", "wrist_roll_link"), ("wrist_pitch_link", "tool_flange_link")}
    tcp_local = model.joints["tcp_joint"]["T"][:3, 3]
    jaw = model.joints["left_jaw_joint"]["T"][:3, 3]
    jaws = np.vstack([_box_points((0.006, 0.045, 0.02), (sx * 0.038, jaw[1], 0.0)) for sx in (1, -1)])
    cube = _box_points([task["payload"]["size"]] * 3, tcp_local)
    st = task["stations"]
    boxes = {}
    x, y = planner.station_xy(st["pick"])
    h = st["pick"]["stand_size"] / 2
    boxes["pick stand"] = (np.array([x - h, y - h, 0]), np.array([x + h, y + h, st["pick"]["surface_z"]]))
    for b in ("bin_red", "bin_blue"):
        x, y = planner.station_xy(st[b])
        h = st[b]["outer"] / 2
        boxes[b.replace("_", " ")] = (np.array([x - h, y - h, 0]), np.array([x + h, y + h, st[b]["rim_z"]]))   # solid up to the rim (conservative)
    links = ["base_link"] + model.link_order
    step = max(1, int(round(1.0 / hz / (run["t"][1] - run["t"][0]))))
    idx = np.arange(0, len(run["t"]), step)
    out = dict(t=run["t"][idx], state=[run["state"][i] for i in idx], table=[], stations=[], self=[], detail=[])
    for i in idx:
        q = run["pos"][i]
        T = link_transforms(model, q)
        W = {ln: pts[ln] @ T[ln][:3, :3].T + T[ln][:3, 3] for ln in links}
        extra = [jaws] + ([cube] if run["payload"][i] > 0.5 else [])
        # the cube rests on the stand during the grasp and is above the bin during release: it is checked
        # against the stations only while it is carried at the travel height
        W_tool_env = np.vstack([W[tool]] + [e @ T[tool][:3, :3].T + T[tool][:3, 3] for e in [jaws]])
        W[tool] = np.vstack([W[tool]] + [e @ T[tool][:3, :3].T + T[tool][:3, 3] for e in extra])
        moving = links[2:]
        d_table = min(W[ln][:, 2].min() for ln in moving)
        d_st, who = 9.0, ""
        for ln in moving:
            P = W_tool_env if ln == tool else W[ln]
            for name, (lo, hi) in boxes.items():
                d = _box_dist(P, lo, hi).min()
                if d < d_st:
                    d_st, who = d, f"{ln} / {name}"
        d_self, who_s = 9.0, ""
        for a in range(len(links)):
            for b in range(a + 2, len(links)):
                if (links[a], links[b]) in structural:
                    continue
                d = _min_dist(W[links[a]], W[links[b]])
                if d < d_self:
                    d_self, who_s = d, f"{links[a]} / {links[b]}"
        out["table"].append(d_table)
        out["stations"].append(d_st)
        out["self"].append(d_self)
        out["detail"].append((who, who_s))
    for k in ("table", "stations", "self"):
        out[k] = np.array(out[k])
    cyc = np.array([s != "HOMING" for s in out["state"]])
    out["summary"] = {}
    for k in ("table", "stations", "self"):
        j = int(np.argmin(np.where(cyc, out[k], 9.0)))
        out["summary"][k] = dict(min_mm=float(out[k][j] * 1000), t=float(out["t"][j]), state=out["state"][j],
                                 pair=out["detail"][j][0 if k == "stations" else 1] if k != "table" else "")
    out["collision"] = bool(min(out["summary"][k]["min_mm"] for k in out["summary"]) <= 0.0)
    return out


def figure(res, out_dir):
    plt = setup_style()
    fig, ax = plt.subplots(figsize=(FIG_W, 2.6))
    for t0, t1, s in state_spans(res["t"], res["state"]):
        ax.axvspan(t0, t1, color=STATE_SHADE.get(s, "#ffffff"), lw=0)
    for k, lab, c in (("table", "arm to table", COLORS[0]), ("stations", "arm to pick stand and bins", COLORS[1]), ("self", "arm to itself (non-adjacent links)", COLORS[2])):
        ax.plot(res["t"], res[k] * 1000, color=c, label=lab)
    ax.axhline(0, color="#d03b3b", lw=1.0, ls="--")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("minimum distance (mm)")
    ax.set_xlim(res["t"][0], res["t"][-1])
    ax.set_ylim(bottom=-5)
    ax.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=3, columnspacing=1.0, handlelength=1.5)
    save(fig, out_dir, "sim_clearance")
