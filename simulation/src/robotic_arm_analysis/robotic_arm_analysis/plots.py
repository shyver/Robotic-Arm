"""Time-series figures from a run log (Gazebo logger or offline simulation)."""
import csv
import os

import numpy as np

from .common import COLORS, D, FIG_W, INK, JLABEL, JN, MUTED, POWER_LIMIT_W, STATE_SHADE, load, read_run, save, setup_style, state_spans


def _shade(ax, t, state, labels=False):
    for t0, t1, s in state_spans(t, state):
        ax.axvspan(t0, t1, color=STATE_SHADE.get(s, "#ffffff"), lw=0)
        if labels and t1 - t0 > 0.0:
            ax.text((t0 + t1) / 2, 1.02, s.replace("_", " ").title().replace("To", "to"), transform=ax.get_xaxis_transform(),
                    rotation=90, ha="center", va="bottom", fontsize=9, color=MUTED)


def _cycle(run, n):
    c = run["cycle"]
    n = n if (c == n).any() else int(c.max())
    i = np.where(c == n)[0]
    return slice(i[0], i[-1] + 1), n


def joint_angles(run, out_dir, cycle=2):
    plt = setup_style()
    sl, n = _cycle(run, cycle)
    t = run["t"][sl] - run["t"][sl][0]
    fig, ax = plt.subplots(figsize=(FIG_W, 3.3))
    _shade(ax, t, run["state"][sl], labels=True)
    for j in range(6):
        ax.plot(t, run["pos"][sl][:, j] / D, color=COLORS[j], label=JLABEL[j])
    ax.set_xlabel("time in the cycle (s)")
    ax.set_ylabel("joint angle (deg)")
    ax.set_xlim(t[0], t[-1])
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.2), ncol=3)
    save(fig, out_dir, "sim_joint_angles")
    return dict(cycle=n, cycle_time=float(t[-1]))


def joint_torques(run, out_dir, model):
    plt = setup_style()
    i = run["cycle"] >= 1
    t = run["t"][i] - run["t"][i][0]
    st = [s for s, k in zip(run["state"], i) if k]
    fig, axes = plt.subplots(3, 2, figsize=(FIG_W, 4.6), sharex=True)
    for j, ax in enumerate(axes.T.ravel() if False else axes.ravel()):
        _shade(ax, t, st)
        lim = model.effort[j]
        ax.plot(t, run["eff"][i][:, j], color=COLORS[j], lw=1.2)
        ax.axhline(lim, color=INK, lw=1.0, ls="--")
        ax.axhline(-lim, color=INK, lw=1.0, ls="--")
        ax.set_ylim(-1.15 * lim, 1.15 * lim)
        ax.set_ylim(-1.3 * lim, 1.6 * lim)
        ax.text(0.02, 0.96, f"{JLABEL[j]}, available ±{lim:g} N·m", transform=ax.transAxes, va="top", ha="left", fontsize=9,
                bbox=dict(facecolor="white", edgecolor="none", alpha=0.8, pad=1.0))
        ax.set_xlim(t[0], t[-1])
        if j % 2 == 0:
            ax.set_ylabel("torque (N·m)")
        if j >= 4:
            ax.set_xlabel("time (s)")
    fig.text(0.99, 1.0, "dashed: available torque of the actuator", ha="right", va="bottom", fontsize=9, color=MUTED)
    fig.tight_layout(h_pad=0.6, w_pad=0.8)
    save(fig, out_dir, "sim_joint_torques")
    pk = np.abs(run["eff"][i]).max(axis=0)
    rms = np.sqrt((run["eff"][i] ** 2).mean(axis=0))
    return dict(peak=pk.tolist(), rms=rms.tolist(), peak_ratio=(pk / model.effort).tolist(), duration=float(t[-1]))


def power(run, out_dir):
    plt = setup_style()
    i = run["cycle"] >= 1
    t = run["t"][i] - run["t"][i][0]
    st = [s for s, k in zip(run["state"], i) if k]
    P = np.abs(run["eff"][i] * run["vel"][i])
    fig, (a, b) = plt.subplots(2, 1, figsize=(FIG_W, 3.6), sharex=True, gridspec_kw={"height_ratios": [1, 1.5]})
    for ax in (a, b):
        _shade(ax, t, st)
        for j in range(4):
            ax.plot(t, P[:, j], color=COLORS[j], lw=1.2, label=JLABEL[j])
        ax.set_xlim(t[0], t[-1])
        ax.set_ylabel("power (W)")
    a.axhline(POWER_LIMIT_W, color=INK, lw=1.0, ls="--")
    a.text(t[-1], POWER_LIMIT_W, f"budget assumption {POWER_LIMIT_W} W ", ha="right", va="bottom", fontsize=9)
    a.set_ylim(0, POWER_LIMIT_W * 1.2)
    b.set_ylim(0, max(1e-3, P[:, :4].max()) * 1.15)
    b.set_xlabel("time (s)")
    b.text(0.01, 0.95, "same data, enlarged scale", transform=b.transAxes, va="top", fontsize=9,
           bbox=dict(facecolor="white", edgecolor="none", alpha=0.75, pad=1.0))
    a.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=4, columnspacing=1.0)
    fig.tight_layout(h_pad=0.5)
    save(fig, out_dir, "sim_power")
    return dict(peak=P.max(axis=0).tolist(), mean=P.mean(axis=0).tolist())


def homing(run, run_dir, out_dir, task, poses):
    plt = setup_style()
    i = np.array([s == "HOMING" for s in run["state"]])
    t = run["t"][i]
    ev = []
    path = os.path.join(run_dir, "events.csv")
    if os.path.exists(path):
        for r in csv.DictReader(open(path)):
            if "triggered" in r["event"]:
                ev.append((float(r["t"]), int(r["event"].split("J")[1][0]) - 1, float(r["event"].split("at ")[1].split(" deg")[0])))
    fig, axes = plt.subplots(4, 1, figsize=(FIG_W, 4.6), sharex=True)
    out = []
    for j, ax in enumerate(axes):
        sw = (poses.start[j] / D + task["homing"]["switch_offset_deg"][j])
        ax.plot(t, run["pos"][i][:, j] / D, color=COLORS[j])
        ax.axhline(sw, color=INK, lw=1.0, ls="--")
        for te, je, ang in ev:
            if je == j:
                ax.plot([te], [ang], marker="v", ms=8, color=INK, ls="none")
                ax.annotate(f"switch at {te:.2f} s", (te, ang), xytext=(8, 4), textcoords="offset points", fontsize=9)
                out.append(dict(joint=JN[j], t=te, angle=ang, switch=sw, overshoot=abs(ang - sw)))
        ax.set_ylabel(f"{JN[j]} (deg)")
        ax.margins(y=0.25)
    axes[-1].set_xlabel("time (s)")
    axes[-1].set_xlim(t[0], t[-1])
    fig.text(0.99, 0.995, "dashed: virtual limit switch,  ▼ switch event", ha="right", va="top", fontsize=9, color=MUTED)
    fig.tight_layout(h_pad=0.4, rect=(0, 0, 1, 0.97))
    save(fig, out_dir, "sim_homing")
    return dict(events=out, duration=float(t[-1] - t[0]))


def estop(run_dir, out_dir):
    plt = setup_style()
    if not os.path.exists(os.path.join(run_dir, "estop.csv")):
        print("  no estop.csv in the run directory: sim_estop skipped")
        return None
    e = read_run(run_dir, "estop.csv")
    if "ESTOP" not in e["state"]:
        print("  estop.csv has no ESTOP rows: sim_estop skipped")
        return None
    k = next(i for i, s in enumerate(e["state"]) if s != "HOLD")
    t = e["t"] - e["t"][k]
    fig, ax = plt.subplots(figsize=(FIG_W, 3.0))
    for j in range(6):
        ax.plot(t, e["pos"][:, j] / D, color=COLORS[j], label=JLABEL[j])
    ax.axvline(0, color=INK, lw=1.0, ls="--")
    ax.annotate("E-stop: J1–J4 disabled", (0, 1), xycoords=("data", "axes fraction"), xytext=(-4, 4), textcoords="offset points", va="bottom", ha="right", fontsize=9)
    res = dict(start=(e["pos"][k] / D).tolist(), end=(e["pos"][-1] / D).tolist(), drop=((e["pos"][-1] - e["pos"][k]) / D).tolist(), contact=None)
    if "CONTACT" in e["state"]:
        c = e["state"].index("CONTACT")
        ax.axvline(t[c], color="#d03b3b", lw=1.0, ls="--")
        ax.annotate("tool reaches the pick stand", (t[c], 1), xycoords=("data", "axes fraction"), xytext=(4, 4), textcoords="offset points", va="bottom", fontsize=9)
        res["contact"] = float(t[c])
    ax.set_xlabel("time after the E-stop (s)")
    ax.set_ylabel("joint angle (deg)")
    ax.set_xlim(t[0], t[-1])
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=3)
    save(fig, out_dir, "sim_estop")
    return res
