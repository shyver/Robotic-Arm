#!/usr/bin/env python3
"""Offline rigid-body simulation of the same URDF, controller gains and motion program as the Gazebo demo.

It needs no ROS and no Gazebo (only numpy, pyyaml and pinocchio: `pip install pin`), and writes the same
log files as the Gazebo logger node, so make_figures.py can plot either source:

    <run_dir>/cycle.csv    homing + pick-and-place cycles (100 Hz): t, state, cycle, pos/vel/eff/des per joint
    <run_dir>/estop.csv    E-stop test: torque of J1..J4 removed while holding the payload
    <run_dir>/events.csv   state changes, homing switch events, classification results
    <run_dir>/meta.json    source = "offline"

Model: articulated-body dynamics (pinocchio CRBA/RNEA) of the URDF masses and inertias, joint viscous
damping and Coulomb friction from the URDF, PID effort control with the gains of controllers.yaml at 1 kHz,
torque saturation at the URDF effort limits, 1 ms time step, payload added to the tool while attached.
"""
import argparse
import csv
import json
import os

import numpy as np
import yaml

from .common import CONTROLLERS, D, URDF, link_points, link_transforms, load, planner
from .common import ARM_JOINTS

DT = 0.001
CTRL_EVERY = 1          # 1 kHz controller (controller_manager update_rate)
LOG_EVERY = 10          # 100 Hz log
V_STICK = 0.01          # rad/s, width of the smoothed Coulomb friction


def build_pin(payload_mass=0.0, payload_size=0.03, tcp_in_tool=None):
    import pinocchio as pin
    full = pin.buildModelFromUrdf(URDF)
    lock = [full.getJointId(n) for n in ("left_jaw_joint", "right_jaw_joint")]
    m = pin.buildReducedModel(full, lock, pin.neutral(full))
    if payload_mass > 0:
        I = payload_mass * payload_size ** 2 / 6.0
        m.appendBodyToJoint(m.getJointId("tool_rotation_joint"),
                            pin.Inertia(payload_mass, np.asarray(tcp_in_tool, dtype=float), np.eye(3) * I), pin.SE3.Identity())
    return m


def load_gains():
    c = yaml.safe_load(open(CONTROLLERS))
    g = {}
    for ctrl in ("stepper_controller", "servo_controller"):
        g.update(c[ctrl]["ros__parameters"]["gains"])
    return (np.array([g[n]["p"] for n in ARM_JOINTS]), np.array([g[n]["i"] for n in ARM_JOINTS]),
            np.array([g[n]["d"] for n in ARM_JOINTS]), np.array([g[n]["i_clamp"] for n in ARM_JOINTS]))


class Sim:
    def __init__(self, model, task, poses):
        import pinocchio as pin
        self.pin = pin
        self.model, self.task, self.poses = model, task, poses
        tcp_in_tool = model.joints["tcp_joint"]["T"][:3, 3]
        self.pm = {False: build_pin(), True: build_pin(task["payload"]["mass"], task["payload"]["size"], tcp_in_tool)}
        self.pd = {k: m.createData() for k, m in self.pm.items()}
        self.kp, self.ki, self.kd, self.iclamp = load_gains()
        self.damp = np.array([model.joints[n]["damping"] for n in ARM_JOINTS])
        self.fric = np.array([model.joints[n]["friction"] for n in ARM_JOINTS])
        self.limit = model.effort.copy()
        self.q = poses.start.copy()
        self.v = np.zeros(6)
        self.integ = np.zeros(6)
        self.des, self.des_v = self.q.copy(), np.zeros(6)
        self.tau = np.zeros(6)
        self.t, self.k = 0.0, 0
        self.state, self.cycle, self.payload, self.jaw = "HOMING", 0, False, 0.0
        self.enabled = np.ones(6, dtype=bool)        # False = driver disabled (E-stop)
        self.rows, self.events = [], []
        self.limit_spring = False

    # ---------------------------------------------------------------- physics
    def step(self):
        pin = self.pin
        if self.k % CTRL_EVERY == 0:
            e = self.des - self.q
            self.integ = np.clip(self.integ + self.ki * e * DT * CTRL_EVERY, -self.iclamp, self.iclamp)
            tau = self.kp * e + self.integ + self.kd * (self.des_v - self.v)
            self.tau = np.where(self.enabled, np.clip(tau, -self.limit, self.limit), 0.0)
        pm, pd = self.pm[self.payload], self.pd[self.payload]
        M = pin.crba(pm, pd, self.q)
        M = np.triu(M) + np.triu(M, 1).T
        h = pin.nonLinearEffects(pm, pd, self.q, self.v)
        th = np.tanh(self.v / V_STICK)
        tau_f = self.fric * th
        c_eff = self.damp + self.fric * (1 - th ** 2) / V_STICK
        ext = np.zeros(6)
        if self.limit_spring:                       # joint end stops (only needed for the E-stop drop)
            over = np.minimum(self.q - self.model.lower, 0) + np.maximum(self.q - self.model.upper, 0)
            ext = -2000.0 * over - 20.0 * self.v * (over != 0)
        rhs = M @ self.v + DT * (self.tau + ext - h - tau_f + (c_eff - self.damp) * self.v)
        self.v = np.linalg.solve(M + DT * np.diag(c_eff), rhs)       # friction/damping implicit
        self.q = self.q + DT * self.v
        self.t += DT
        self.k += 1
        if self.k % LOG_EVERY == 0:
            self.rows.append([round(self.t, 4), self.state, self.cycle] + list(self.q) + list(self.v) + list(self.tau)
                             + list(self.des) + [self.jaw, int(self.payload)])

    def event(self, text):
        self.events.append((round(self.t, 3), self.state, text))
        print(f"[{self.t:8.3f} s] {self.state:12s} {text}")

    # ---------------------------------------------------------------- command interpreter
    def run(self, prog):
        vmax, amax = planner.motion_limits(self.task)
        settle = self.task["motion"]["settle_s"]
        reply = None
        while True:
            try:
                cmd = prog.send(reply)
            except StopIteration:
                break
            reply = None
            op = cmd[0]
            if op == "state":
                self.state = cmd[1]
                if cmd[1] == "APPROACH":
                    self.cycle += 1
                self.event("state")
            elif op == "log":
                self.event(cmd[1])
            elif op == "move":
                q0, q1 = self.des.copy(), np.asarray(cmd[1])
                T = planner.sync_duration(q0, q1, vmax, amax)
                t0 = self.t
                while self.t - t0 < T + settle:
                    self.des, self.des_v, _ = planner.sync_sample(q0, q1, T, self.t - t0)
                    self.step()
                self.des_v = np.zeros(6)
            elif op == "seek":
                _, j, speed, switch = cmd
                while (self.q[j] - switch) * np.sign(speed) < 0:
                    self.des[j] += speed * DT
                    self.des_v[j] = speed
                    self.step()
                reply = float(self.q[j])
                self.event(f"limit switch J{j+1} triggered at {self.q[j]/D:.3f} deg")
                self.des[j], self.des_v[j] = self.q[j], 0.0
                self.hold(0.3)
            elif op == "gripper":
                self.jaw = cmd[1]
                self.hold(0.3)
            elif op == "dwell":
                self.hold(cmd[1])
            elif op == "attach":
                self.payload = True
                self.event(f"attach {cmd[1]}")
            elif op == "detach":
                self.payload = False
                self.event(f"detach {cmd[1]}")
            elif op == "classify":
                reply = next(c["color"] for c in self.task["cubes"] if c["name"] == cmd[1])
            elif op == "feed":
                self.event(f"feed {cmd[1]} to the pick location")

    def hold(self, seconds):
        t0 = self.t
        while self.t - t0 < seconds:
            self.step()

    def write(self, path):
        hdr = ["t", "state", "cycle"] + [f"{k}_J{i}" for k in ("pos", "vel", "eff", "des") for i in range(1, 7)] + ["jaw", "payload"]
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(hdr)
            for r in self.rows:
                w.writerow([f"{x:.6g}" if isinstance(x, float) else x for x in r])


def run_cycles(run_dir, cycles=None):
    model, task, poses = load()
    sim = Sim(model, task, poses)
    sim.run(planner.program(poses, task, cycles=cycles))
    sim.write(os.path.join(run_dir, "cycle.csv"))
    with open(os.path.join(run_dir, "events.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "state", "event"])
        w.writerows(sim.events)
    return sim


def run_estop(run_dir):
    """hold the payload at the travel height above the pick stand, then disable J1..J4."""
    model, task, poses = load()
    sim = Sim(model, task, poses)
    sim.q = poses.q["above_pick"].copy()
    sim.des = sim.q.copy()
    sim.payload, sim.state, sim.limit_spring = True, "HOLD", True
    sim.integ = np.clip(model.gravity_torques(sim.q, task["payload"]["mass"]), -sim.iclamp, sim.iclamp)
    sim.hold(1.0)
    sim.state = "ESTOP"
    sim.enabled[:4] = False
    sim.event("E-stop: J1..J4 drivers disabled")
    pts = link_points(model)
    st = task["stations"]["pick"]
    stand_xy, stand_half, stand_z = np.array(planner.station_xy(st)), st["stand_size"] / 2, st["surface_z"]
    contact = None
    t_end = sim.t + task["estop_test"]["duration_s"]
    while sim.t < t_end:
        sim.step()
        if sim.k % 5 == 0:
            T = link_transforms(model, sim.q)
            for ln in model.link_order[1:]:
                p = pts[ln] @ T[ln][:3, :3].T + T[ln][:3, 3]
                if p[:, 2].min() < 0.0:
                    contact = (sim.t, ln, "the table")
                    break
                if (np.all(np.abs(p[:, :2] - stand_xy) < stand_half, axis=1) & (p[:, 2] < stand_z)).any():
                    contact = (sim.t, ln, "the pick stand")
                    break
            if contact:
                break
    if contact:
        sim.event(f"{contact[1]} reaches {contact[2]} {contact[0]-1.0:.2f} s after the E-stop")
        sim.state = "CONTACT"
        sim.v[:] = 0
        for _ in range(30):
            sim.rows.append([round(sim.t, 4), sim.state, 0] + list(sim.q) + [0.0] * 6 + list(sim.tau) + list(sim.des) + [sim.jaw, 1])
            sim.t += 0.01
    sim.write(os.path.join(run_dir, "estop.csv"))
    with open(os.path.join(run_dir, "estop_events.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "state", "event"])
        w.writerows(sim.events)
    return sim, contact


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--run-dir", default="results/offline_run")
    ap.add_argument("--cycles", type=int, default=None)
    a = ap.parse_args()
    os.makedirs(a.run_dir, exist_ok=True)
    run_cycles(a.run_dir, a.cycles)
    run_estop(a.run_dir)
    json.dump({"source": "offline", "description": "offline rigid-body simulation (pinocchio), same URDF, gains and program as the Gazebo demo"},
              open(os.path.join(a.run_dir, "meta.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
