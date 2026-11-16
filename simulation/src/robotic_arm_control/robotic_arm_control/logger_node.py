#!/usr/bin/env python3
"""Logs joint positions, velocities and efforts at 100 Hz (simulation time) to CSV files that
robotic_arm_analysis/make_figures.py reads:

    ~/ras_arm_logs/<date_time>/cycle.csv     homing + pick-and-place cycles
    ~/ras_arm_logs/<date_time>/estop.csv     rows logged during the E-stop test (states HOLD / ESTOP)
    ~/ras_arm_logs/<date_time>/events.csv    state changes, limit-switch events
    ~/ras_arm_logs/latest -> newest run

Effort = torque commanded by the joint trajectory controllers (controller_state output), i.e. the actuator torque.
If that field is not published by the installed ros2_controllers version, the effort of /joint_states is used.
"""
import csv
import datetime
import json
import os

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .kinematics import ARM_JOINTS, JAW_JOINTS


class Logger(Node):
    def __init__(self):
        super().__init__("arm_logger")
        self.declare_parameter("log_root", os.path.expanduser("~/ras_arm_logs"))
        root = self.get_parameter("log_root").value
        self.dir = os.path.join(root, datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.dir, exist_ok=True)
        latest = os.path.join(root, "latest")
        if os.path.islink(latest):
            os.unlink(latest)
        if not os.path.exists(latest):
            os.symlink(self.dir, latest)
        hdr = ["t", "state", "cycle"] + [f"{k}_J{i}" for k in ("pos", "vel", "eff", "des") for i in range(1, 7)] + ["jaw", "payload"]
        self.files = {}
        for name in ("cycle", "estop"):
            f = open(os.path.join(self.dir, name + ".csv"), "w", newline="")
            w = csv.writer(f)
            w.writerow(hdr)
            self.files[name] = (f, w)
        self.ev_f = open(os.path.join(self.dir, "events.csv"), "w", newline="")
        self.ev = csv.writer(self.ev_f)
        self.ev.writerow(["t", "state", "event"])
        self.pos, self.vel, self.eff_js = {}, {}, {}
        self.eff_cmd, self.des = {}, {}
        self.status = ("INIT", 0, 0)
        self.t0 = None
        self.used_cmd = False
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.create_subscription(String, "/arm_status", self.on_status, 10)
        self.create_subscription(String, "/arm_events", self.on_event, 50)
        for c in ("stepper_controller", "servo_controller"):
            self.create_subscription(JointTrajectoryControllerState, f"/{c}/controller_state", self.on_ctrl, 10)
            self.create_subscription(JointTrajectoryControllerState, f"/{c}/state", self.on_ctrl, 10)
        self.create_timer(0.01, self.tick)
        self.get_logger().info(f"logging to {self.dir}")

    def on_js(self, m):
        for i, n in enumerate(m.name):
            self.pos[n] = m.position[i]
            if len(m.velocity) > i:
                self.vel[n] = m.velocity[i]
            if len(m.effort) > i:
                self.eff_js[n] = m.effort[i]

    def on_ctrl(self, m):
        out = getattr(m, "output", None)
        ref = getattr(m, "reference", None)
        if ref is None or len(ref.positions) == 0:
            ref = getattr(m, "desired", None)
        for i, n in enumerate(m.joint_names):
            if out is not None and len(out.effort) > i:
                self.eff_cmd[n] = out.effort[i]
            if ref is not None and len(ref.positions) > i:
                self.des[n] = ref.positions[i]

    def on_status(self, m):
        s, c, p = m.data.split("|")
        self.status = (s, int(c), int(p))

    def on_event(self, m):
        t, s, text = m.data.split("|", 2)
        if self.t0 is not None:
            self.ev.writerow([f"{float(t) - self.t0:.3f}", s, text])
            self.ev_f.flush()

    def tick(self):
        state, cycle, payload = self.status
        if state in ("INIT", "DONE") or not all(n in self.pos for n in ARM_JOINTS):
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        if state == "ESTOP":                    # the stepper controller is off: no commanded torque on J1..J4
            for n in ARM_JOINTS[:4]:
                self.eff_cmd[n] = 0.0
        eff = []
        for n in ARM_JOINTS:
            if n in self.eff_cmd:
                eff.append(self.eff_cmd[n])
                self.used_cmd = True
            else:
                eff.append(self.eff_js.get(n, 0.0))
        row = [f"{now - self.t0:.4f}", state, cycle] + [f"{self.pos[n]:.6g}" for n in ARM_JOINTS] + \
              [f"{self.vel.get(n, 0.0):.6g}" for n in ARM_JOINTS] + [f"{e:.6g}" for e in eff] + \
              [f"{self.des.get(n, self.pos[n]):.6g}" for n in ARM_JOINTS] + [f"{self.pos.get(JAW_JOINTS[0], 0.0):.5g}", payload]
        self.files["estop" if state in ("HOLD", "ESTOP") else "cycle"][1].writerow(row)

    def close(self):
        for f, _ in self.files.values():
            f.close()
        self.ev_f.close()
        if os.path.getsize(os.path.join(self.dir, "estop.csv")) < 400:
            os.remove(os.path.join(self.dir, "estop.csv"))
        json.dump({"source": "gazebo", "effort_source": "controller_state output.effort" if self.used_cmd else "joint_states effort"},
                  open(os.path.join(self.dir, "meta.json"), "w"), indent=1)


def main():
    rclpy.init()
    node = Logger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
