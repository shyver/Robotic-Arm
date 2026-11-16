"""Motion program of the cell (homing, pick-and-place state machine), independent of ROS.

`program()` is a generator of commands.  It is interpreted by the Gazebo node
(pick_place_node.py) and by the offline simulation (robotic_arm_analysis/offline_sim.py),
so the sequence that is plotted is the sequence that runs.

Commands:
  ("state", name)                      state-machine state (logged)
  ("move", q[6])                       synchronised trapezoidal joint move to q, then settle
  ("seek", joint, speed, switch)       move one joint at constant speed until it passes `switch` (limit switch)
  ("gripper", jaw_position)
  ("feed", cube) / ("attach", cube) / ("detach", cube)
  ("classify", cube)                   interpreter must send() back the colour ("red" / "blue")
  ("dwell", seconds), ("log", text)
"""
import math
import os

import numpy as np
import yaml

from .kinematics import ArmModel

D = math.pi / 180.0
STATES = ["HOMING", "HOME", "APPROACH", "DESCEND", "GRASP", "LIFT", "CLASSIFY", "MOVE_TO_BIN", "RELEASE", "RETURN"]


def load_task(path=None):
    if path is None:
        path = os.path.join(os.path.dirname(__file__), "..", "config", "task.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


def station_xy(st):
    a = st["azimuth_deg"] * D
    return st["r"] * math.cos(a), st["r"] * math.sin(a)


def balanced_pose(model, q0=None):
    """pose near the CAD zero pose where the gravity torques of J2..J4 vanish: the unpowered arm
    stays there (joint friction), so the simulation can start before the controllers are active."""
    q = np.zeros(6) if q0 is None else np.array(q0, dtype=float)
    for _ in range(60):
        g = model.gravity_torques(q)[1:4]
        if np.abs(g).max() < 1e-6:
            break
        Jg = np.zeros((3, 3))
        for k in range(3):
            dq = q.copy()
            dq[1 + k] += 1e-5
            Jg[:, k] = (model.gravity_torques(dq)[1:4] - g) / 1e-5
        step = np.linalg.solve(Jg, -g)
        q[1:4] += np.clip(step, -0.2, 0.2)
    return q


class Poses:
    """joint targets of the task, solved once by IK.  One arm configuration (the one of ik_seed_deg) and one
    tool orientation (task["tool"]) are used for the whole cycle: every pose is seeded from above_pick, so
    the elbow never flips; moving to a bin is lift -> rotate J1 at the travel height -> descend."""

    def __init__(self, model, task):
        self.model, self.task = model, task
        h = task["heights"]
        seed = np.array(task["ik_seed_deg"]) * D
        self.start = balanced_pose(model) if task["start_pose_deg"] == "auto" else np.array(task["start_pose_deg"]) * D
        self.q, self.xyz = {}, {}
        px, py = station_xy(task["stations"]["pick"])
        paz = task["stations"]["pick"]["azimuth_deg"] * D
        self._solve("above_pick", (px, py, h["travel"]), paz, seed)
        self._solve("pick", (px, py, h["grasp"]), paz, self.q["above_pick"])
        self.q["home"] = self.q["above_pick"]
        self.xyz["home"] = self.xyz["above_pick"]
        for color in ("red", "blue"):
            st = task["stations"]["bin_" + color]
            bx, by = station_xy(st)
            s = self.q["above_pick"].copy()
            s[0] += (st["azimuth_deg"] - task["stations"]["pick"]["azimuth_deg"]) * D
            self._solve("above_bin_" + color, (bx, by, h["travel"]), st["azimuth_deg"] * D, s)
            self._solve("bin_" + color, (bx, by, h["release"]), st["azimuth_deg"] * D, self.q["above_bin_" + color])

    def _solve(self, name, xyz, yaw, seed):
        tool = self.task.get("tool", {})
        q, ok = self.model.ik_tool(xyz, yaw, tool.get("jaw_yaw_deg", 0.0) * D, tool.get("tilt_deg", 0.0) * D, seed)
        if not ok:
            raise RuntimeError(f"IK failed for '{name}' at {xyz}: target outside the reachable workspace")
        self.q[name], self.xyz[name] = q, xyz


def sync_duration(q0, q1, vmax, amax):
    """duration of a synchronised trapezoidal move (accel 25 %, cruise 50 %, decel 25 % of the time)."""
    d = np.abs(np.asarray(q1) - np.asarray(q0))
    T = max(np.max(d / (0.75 * vmax)), np.max(np.sqrt(d / (0.1875 * amax))))
    return max(T, 0.2)


def sync_sample(q0, q1, T, t):
    """position, velocity, acceleration at time t of the synchronised move."""
    q0, q1 = np.asarray(q0), np.asarray(q1)
    ta = 0.25 * T
    vp = 1.0 / (0.75 * T)
    a = vp / ta
    t = min(max(t, 0.0), T)
    if t < ta:
        s, sd, sdd = 0.5 * a * t * t, a * t, a
    elif t <= T - ta:
        s, sd, sdd = 0.5 * a * ta * ta + vp * (t - ta), vp, 0.0
    else:
        r = T - t
        s, sd, sdd = 1.0 - 0.5 * a * r * r, a * r, -a
    return q0 + (q1 - q0) * s, (q1 - q0) * sd, (q1 - q0) * sdd


def motion_limits(task):
    m = task["motion"]
    v = np.array([m["stepper_speed_deg_s"]] * 4 + [m["servo_speed_deg_s"]] * 2) * D
    a = np.array([m["stepper_accel_deg_s2"]] * 4 + [m["servo_accel_deg_s2"]] * 2) * D
    return v, a


def homing_program(poses, task):
    hm = task["homing"]
    q = poses.start.copy()
    yield ("state", "HOMING")
    for j in hm["order"]:
        switch = poses.start[j] + hm["switch_offset_deg"][j] * D
        direction = -1.0 if hm["switch_offset_deg"][j] < 0 else 1.0
        yield ("log", f"homing J{j+1}: seeking the limit switch at {switch/D:.2f} deg")
        hit = yield ("seek", j, direction * hm["seek_speed_deg_s"] * D, switch)
        hit = switch if hit is None else hit
        q[j] = switch - direction * hm["backoff_deg"] * D
        yield ("log", f"homing J{j+1}: switch hit at {hit/D:.3f} deg (overshoot {abs(hit-switch)/D:.3f} deg), "
                      f"back off {hm['backoff_deg']} deg, zero set")
        yield ("move", q.copy())
    yield ("log", "homing done")


def cycle_program(poses, task, i):
    cube = task["cubes"][i]
    m = task["motion"]
    g = task["gripper"]
    yield ("feed", cube["name"])
    yield ("state", "APPROACH")
    yield ("gripper", g["open"])
    yield ("move", poses.q["above_pick"])
    yield ("state", "DESCEND")
    yield ("move", poses.q["pick"])
    yield ("state", "GRASP")
    yield ("gripper", g["closed"])
    yield ("dwell", m["grasp_dwell_s"])
    yield ("attach", cube["name"])
    yield ("state", "LIFT")
    yield ("move", poses.q["above_pick"])
    yield ("state", "CLASSIFY")
    color = yield ("classify", cube["name"])
    color = color or cube["color"]
    yield ("dwell", m["classify_dwell_s"])
    yield ("log", f"cycle {i+1}: {cube['name']} classified {color} -> bin_{color}")
    yield ("state", "MOVE_TO_BIN")
    yield ("move", poses.q["above_bin_" + color])
    yield ("move", poses.q["bin_" + color])
    yield ("state", "RELEASE")
    yield ("detach", cube["name"])
    yield ("gripper", g["open"])
    yield ("dwell", m["grasp_dwell_s"])
    yield ("state", "RETURN")
    yield ("move", poses.q["above_bin_" + color])
    yield ("move", poses.q["home"])


def program(poses, task, cycles=None, homing=True):
    if homing:
        yield from homing_program(poses, task)
    yield ("state", "HOME")
    yield ("move", poses.q["home"])
    n = len(task["cubes"]) if cycles is None else cycles
    for i in range(n):
        yield from cycle_program(poses, task, i % len(task["cubes"]))
        yield ("state", "HOME")
        yield ("dwell", 0.3)
    yield ("log", "program finished")


def default_urdf():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("robotic_arm_description"), "urdf", "robotic_arm.urdf")
    except Exception:
        here = os.path.dirname(os.path.abspath(__file__))
        return os.path.normpath(os.path.join(here, "..", "..", "robotic_arm_description", "urdf", "robotic_arm.urdf"))


def default_task():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("robotic_arm_control"), "config", "task.yaml")
    except Exception:
        return os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "config", "task.yaml"))


def load_all(urdf=None, task=None):
    model = ArmModel(urdf or default_urdf())
    t = load_task(task or default_task())
    return model, t, Poses(model, t)
