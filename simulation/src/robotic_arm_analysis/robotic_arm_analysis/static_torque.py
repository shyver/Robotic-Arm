#!/usr/bin/env python3
"""Static torque check computed directly from the URDF (no simulator).

For every joint: gravity torque of everything beyond the joint, payload at the tcp included,
 (a) at the worst pose inside the joint ranges (book rule; for J2 this is the arm stretched out horizontally),
 (b) at the worst pose of the pick-and-place task (the poses actually used).
Required = 1.5 x static (acceleration allowance of the design book), margin = available / required.
"""
import numpy as np

from .common import ACTUATOR, D, JLABEL, load

FACTOR = 1.5


def worst_poses(model, payload, n=40000, seed=1):
    rng = np.random.default_rng(seed)
    Q = rng.uniform(model.lower, model.upper, size=(n, 6))
    Q[: n // 4, 4] = rng.choice([-41.4, 138.6, 48.6, -131.4], n // 4) * D     # tool in / across the arm plane
    tau = np.array([model.gravity_torques(q, payload) for q in Q])
    out = []
    for j in range(6):
        q = Q[np.argmax(np.abs(tau[:, j]))].copy()
        best = abs(model.gravity_torques(q, payload)[j])
        step = 5 * D
        while step > 0.02 * D:                      # coordinate ascent refinement
            improved = False
            for k in range(1, 6):
                for s in (step, -step):
                    c = q.copy()
                    c[k] = np.clip(c[k] + s, model.lower[k], model.upper[k])
                    v = abs(model.gravity_torques(c, payload)[j])
                    if v > best + 1e-9:
                        q, best, improved = c, v, True
            if not improved:
                step /= 2
        out.append((best, q))
    return out


def compute():
    model, task, poses = load()
    mp = task["payload"]["mass"]
    worst = worst_poses(model, mp)
    task_tau = np.max(np.abs([model.gravity_torques(q, mp) for q in poses.q.values()]), axis=0)
    rows = []
    for j in range(1, 6):
        st = worst[j][0]
        rows.append(dict(joint=JLABEL[j], actuator=ACTUATOR[j], available=model.effort[j], static=st,
                         required=FACTOR * st, margin=model.effort[j] / (FACTOR * st),
                         task_static=task_tau[j], task_required=FACTOR * task_tau[j],
                         task_margin=model.effort[j] / (FACTOR * task_tau[j]) if task_tau[j] > 1e-3 else float("inf"),
                         worst_pose_deg=[round(float(x) / D, 1) for x in worst[j][1]]))
    g = task["gripper"]
    f_n = mp * 9.81 / (2 * g["mu"])                 # normal force per jaw so that friction carries the weight
    t_g = f_n * g["pinion_radius"]                  # rack-and-pinion: both racks on one pinion, T = F * r
    rows.append(dict(joint="Gripper", actuator="MG996R", available=1.08, static=t_g, required=FACTOR * t_g,
                     margin=1.08 / (FACTOR * t_g), task_static=t_g, task_required=FACTOR * t_g,
                     task_margin=1.08 / (FACTOR * t_g), worst_pose_deg=None, grip_force=f_n))
    # what the full-range rule would need
    need = dict(j2_ratio=14 * FACTOR * worst[1][0] / model.effort[1], j3_ratio=14 * FACTOR * worst[2][0] / model.effort[2])
    # largest horizontal reach of the tcp at which J2 still meets the rule with the arm stretched (for the text)
    return dict(rows=rows, factor=FACTOR, payload=mp, mu=g["mu"], pinion_radius=g["pinion_radius"], need=need,
                moving_mass=sum(m for m, _ in model._c_mass[1:]) + mp)


def table_text(res):
    lines = ["Joint            Actuator          Avail.  | worst pose in range: static  x1.5  margin | task poses: static  x1.5  margin"]
    for r in res["rows"]:
        tm = "   inf" if r["task_margin"] == float("inf") else f"{r['task_margin']:6.2f}"
        lines.append(f"{r['joint']:16s} {r['actuator']:16s} {r['available']:5.2f}   |   {r['static']:8.3f} {r['required']:7.3f} {r['margin']:7.2f}"
                     f"       |   {r['task_static']:8.3f} {r['task_required']:7.3f} {tm}")
    return "\n".join(lines)


def main():
    res = compute()
    print(table_text(res))
    for r in res["rows"]:
        if r["worst_pose_deg"]:
            print(f"  worst pose {r['joint']:15s} q = {r['worst_pose_deg']} deg")
    print("  reduction ratio needed for margin 1 over the whole range: J2 1:%.0f, J3 1:%.0f" % (res["need"]["j2_ratio"], res["need"]["j3_ratio"]))


if __name__ == "__main__":
    main()
