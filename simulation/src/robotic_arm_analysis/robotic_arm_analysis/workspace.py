#!/usr/bin/env python3
"""Reachable workspace of the tcp (side and top view) with the pick location and the bins."""
import math

import numpy as np

from .common import COLORS, D, FIG_W, INK, MUTED, load, planner, save, setup_style


def tool_down_grid(model, task, seed):
    """grid of targets with the task tool orientation in the arm plane: reachable?  static torque rule met (x1.5)?"""
    mp = task["payload"]["mass"]
    tool = task.get("tool", {})
    seeds = [seed, seed * np.array([1, -1, -1, 0.5, 1, 1])]
    out = []
    for r in np.arange(0.06, 0.521, 0.03):
        for z in np.arange(0.0, 0.331, 0.03):
            best = None
            for s in seeds:
                q, ok = model.ik_tool((r, 0.0, z), 0.0, tool.get("jaw_yaw_deg", 0.0) * D, tool.get("tilt_deg", 0.0) * D, s)
                if ok:
                    u = np.max(1.5 * np.abs(model.gravity_torques(q, mp)) / model.effort)
                    best = u if best is None else min(best, u)
            out.append((r, z, best))
    return out


def compute(n=30000, seed=2):
    model, task, poses = load()
    rng = np.random.default_rng(seed)
    Q = rng.uniform(model.lower, model.upper, size=(n, 6))
    P = np.array([model.tcp(q)[:3, 3] for q in Q])
    grid = tool_down_grid(model, task, poses.q["above_pick"])
    ok = [(r, z) for r, z, u in grid if u is not None and u <= 1.0]
    return dict(model=model, task=task, poses=poses, P=P, grid=grid,
                reach_max=float(np.max(np.linalg.norm(P[:, :2], axis=1))), z_max=float(P[:, 2].max()),
                tool_down_ok_r=(min(r for r, _ in ok), max(r for r, _ in ok)) if ok else None,
                tool_down_ok_n=len(ok), tool_down_n=sum(1 for g in grid if g[2] is not None))


def figure(res, out_dir):
    plt = setup_style()
    task, P = res["task"], res["P"]
    fig, (a, b) = plt.subplots(1, 2, figsize=(FIG_W, 3.4), gridspec_kw={"width_ratios": [0.66, 1]})
    rr = np.linalg.norm(P[:, :2], axis=1) * 1000
    a.scatter(rr, P[:, 2] * 1000, s=1, c="#c9c8c2", alpha=0.5, linewidths=0, rasterized=True, label="tcp reachable (any orientation)")
    g = res["grid"]
    gr = [(r * 1000, z * 1000) for r, z, u in g if u is not None and u > 1.0]
    go = [(r * 1000, z * 1000) for r, z, u in g if u is not None and u <= 1.0]
    if gr:
        a.scatter(*zip(*gr), s=14, marker="x", c=COLORS[1], linewidths=1.0, label="task tool orientation, torque rule not met")
    if go:
        a.scatter(*zip(*go), s=14, marker="o", c=COLORS[2], linewidths=0, label="task tool orientation, torque rule met")
    st = task["stations"]
    a.plot([st["pick"]["r"] * 1000], [task["heights"]["grasp"] * 1000], marker="*", ms=11, c=INK, ls="none", label="pick")
    a.plot([st["bin_red"]["r"] * 1000], [task["heights"]["release"] * 1000], marker="s", ms=6, c=INK, mfc="white", ls="none", label="bin release")
    a.axhline(0, color=MUTED, lw=1.0)
    a.set_xlabel("radial distance from the J1 axis (mm)")
    a.set_ylabel("height above the table (mm)")
    a.set_xlim(0, 900)
    a.set_ylim(-450, 950)
    a.set_aspect("equal", adjustable="box")
    b.scatter(P[:, 0] * 1000, P[:, 1] * 1000, s=1, c="#c9c8c2", alpha=0.5, linewidths=0, rasterized=True)
    th = np.linspace(0, 2 * math.pi, 200)
    b.plot(125 * np.cos(th), 125 * np.sin(th), color=MUTED, lw=1.0)
    b.annotate("base", (0, 0), ha="center", va="center", color=MUTED, fontsize=9)
    if res["tool_down_ok_r"]:
        for r in res["tool_down_ok_r"]:
            b.plot(r * 1000 * np.cos(th), r * 1000 * np.sin(th), color=COLORS[2], lw=1.2, ls="--")
    px, py = planner.station_xy(st["pick"])
    b.plot([px * 1000], [py * 1000], marker="*", ms=11, c=INK, ls="none")
    b.annotate("pick", (px * 1000, py * 1000), xytext=(8, 6), textcoords="offset points", fontsize=9)
    for name, col in (("bin_red", "#d03b3b"), ("bin_blue", COLORS[0])):
        x, y = planner.station_xy(st[name])
        b.plot([x * 1000], [y * 1000], marker="s", ms=8, c=col, ls="none")
        b.annotate(name.replace("_", " "), (x * 1000, y * 1000), xytext=(8, 6 if y > 0 else -12), textcoords="offset points", fontsize=9)
    b.set_xlabel("x (mm)")
    b.set_ylabel("y (mm)")
    b.set_xlim(-900, 900)
    b.set_ylim(-900, 900)
    b.set_aspect("equal", adjustable="box")
    fig.legend(*a.get_legend_handles_labels(), loc="lower center", ncol=2, bbox_to_anchor=(0.5, 0.98), markerscale=1.0, handletextpad=0.4)
    fig.tight_layout()
    save(fig, out_dir, "sim_workspace")


if __name__ == "__main__":
    import sys
    figure(compute(), sys.argv[1] if len(sys.argv) > 1 else "results/figures")
