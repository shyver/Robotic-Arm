#!/usr/bin/env python3
"""Runs the motion program (homing + pick-and-place state machine) on the Gazebo arm.

It interprets the same command stream as the offline simulation (planner.program):
joint moves are sent as synchronised trapezoidal trajectories to the joint trajectory controllers,
homing watches /joint_states for the virtual limit switches, grasping uses the DetachableJoint topics.

Parameters: cycles (int, default 4), homing (bool, default true), estop_test (bool, default false: when true the
arm picks the first cube, holds it at the travel height, then calls /estop and watches the drop).
"""
import math
import subprocess

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, String
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from . import planner
from .kinematics import ARM_JOINTS, JAW_JOINTS

D = math.pi / 180.0
WORLD = "sorting_cell"


def dur(t):
    return Duration(sec=int(t), nanosec=int((t - int(t)) * 1e9))


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place")
        self.declare_parameter("cycles", 4)
        self.declare_parameter("homing", True)
        self.declare_parameter("estop_test", False)
        self.model, self.task, self.poses = planner.load_all()
        self.q = None
        self.jaw = 0.0
        self.des = None
        self.state, self.cycle, self.payload = "INIT", 0, 0
        self.create_subscription(JointState, "/joint_states", self.on_js, 50)
        self.pub_step = self.create_publisher(JointTrajectory, "/stepper_controller/joint_trajectory", 10)
        self.pub_servo = self.create_publisher(JointTrajectory, "/servo_controller/joint_trajectory", 10)
        self.pub_grip = self.create_publisher(JointTrajectory, "/gripper_controller/joint_trajectory", 10)
        self.pub_status = self.create_publisher(String, "/arm_status", 10)
        self.pub_event = self.create_publisher(String, "/arm_events", 50)
        self.attach = {c["name"]: self.create_publisher(Empty, f"/grasp/{c['name']}/attach", 10) for c in self.task["cubes"]}
        self.detach = {c["name"]: self.create_publisher(Empty, f"/grasp/{c['name']}/detach", 10) for c in self.task["cubes"]}
        self.estop = self.create_client(SetBool, "/estop")
        self.create_timer(0.02, self.publish_status)

    # ------------------------------------------------------------------ helpers
    def on_js(self, msg):
        pos = dict(zip(msg.name, msg.position))
        if all(n in pos for n in ARM_JOINTS):
            self.q = np.array([pos[n] for n in ARM_JOINTS])
        if JAW_JOINTS[0] in pos:
            self.jaw = pos[JAW_JOINTS[0]]

    def publish_status(self):
        self.pub_status.publish(String(data=f"{self.state}|{self.cycle}|{self.payload}"))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def sleep(self, seconds):
        """sleep in simulation time while processing callbacks."""
        t0 = self.now()
        while rclpy.ok() and self.now() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.005)

    def event(self, text):
        self.get_logger().info(f"[{self.state}] {text}")
        self.pub_event.publish(String(data=f"{self.now():.3f}|{self.state}|{text}"))

    def send(self, points):
        """points: list of (time_from_start, q[6], qd[6] or None) -> both arm controllers."""
        for pub, idx in ((self.pub_step, [0, 1, 2, 3]), (self.pub_servo, [4, 5])):
            msg = JointTrajectory()
            msg.joint_names = [ARM_JOINTS[i] for i in idx]
            for t, q, qd in points:
                p = JointTrajectoryPoint()
                p.positions = [float(q[i]) for i in idx]
                if qd is not None:
                    p.velocities = [float(qd[i]) for i in idx]
                p.time_from_start = dur(t)
                msg.points.append(p)
            pub.publish(msg)

    def move(self, q1):
        vmax, amax = planner.motion_limits(self.task)
        q0 = self.des.copy()
        T = planner.sync_duration(q0, q1, vmax, amax)
        n = max(2, int(math.ceil(T / 0.05)))
        pts = []
        for k in range(1, n + 1):
            q, qd, _ = planner.sync_sample(q0, q1, T, T * k / n)
            pts.append((T * k / n, q, qd))
        self.send(pts)
        self.des = np.array(q1, dtype=float)
        self.sleep(T + self.task["motion"]["settle_s"])

    def seek(self, j, speed, switch):
        """constant-speed move of joint j until the measured position passes the virtual limit switch."""
        target = self.des.copy()
        target[j] = switch + math.copysign(10 * D, speed)          # aim beyond the switch
        T = abs(target[j] - self.des[j]) / abs(speed)
        self.send([(T, target, None)])
        t0 = self.now()
        while rclpy.ok() and (self.q[j] - switch) * math.copysign(1.0, speed) < 0 and self.now() - t0 < T + 2.0:
            rclpy.spin_once(self, timeout_sec=0.002)
        hit = float(self.q[j])
        self.des[j] = hit
        self.send([(0.15, self.des, np.zeros(6))])                 # stop: hold where the switch triggered
        self.event(f"limit switch J{j+1} triggered at {hit/D:.3f} deg")
        self.sleep(0.3)
        return hit

    def gripper(self, pos):
        msg = JointTrajectory()
        msg.joint_names = list(JAW_JOINTS)
        p = JointTrajectoryPoint()
        p.positions = [float(pos), float(pos)]
        p.time_from_start = dur(0.3)
        msg.points.append(p)
        self.pub_grip.publish(msg)
        self.sleep(0.3)

    def feed(self, cube):
        """teleport the next cube onto the pick stand (the first one is already there)."""
        st = self.task["stations"]["pick"]
        x, y = planner.station_xy(st)
        z = st["surface_z"] + self.task["payload"]["size"] / 2 + 0.002
        yaw = (st["azimuth_deg"] + self.task.get("tool", {}).get("jaw_yaw_deg", 0.0)) * D     # cube faces parallel to the jaws
        req = (f'name: "{cube}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}, '
               f'orientation: {{z: {math.sin(yaw / 2):.5f}, w: {math.cos(yaw / 2):.5f}}}')
        try:
            subprocess.run(["ign", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "ignition.msgs.Pose",
                            "--reptype", "ignition.msgs.Boolean", "--timeout", "2000", "--req", req],
                           check=True, capture_output=True, timeout=5)
            self.event(f"feed {cube} to the pick location")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"could not move {cube} to the pick location ({e}); is 'ign' on the PATH?")
        self.sleep(0.5)

    # ------------------------------------------------------------------ main program
    def run(self):
        self.get_logger().info("waiting for /joint_states and the trajectory controllers ...")
        while rclpy.ok() and (self.q is None or self.pub_step.get_subscription_count() == 0
                              or self.pub_servo.get_subscription_count() == 0 or self.now() < 0.5):
            rclpy.spin_once(self, timeout_sec=0.1)
        for c in self.detach.values():           # safety for Fortress versions without <initial_attach>
            c.publish(Empty())
        self.des = self.q.copy()
        estop_test = self.get_parameter("estop_test").value
        cycles = 1 if estop_test else self.get_parameter("cycles").value
        prog = planner.program(self.poses, self.task, cycles=cycles, homing=self.get_parameter("homing").value)
        reply = None
        while rclpy.ok():
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
                if estop_test and cmd[1] == "CLASSIFY":
                    self.run_estop_test()
                    return
            elif op == "log":
                self.event(cmd[1])
            elif op == "move":
                self.move(cmd[1])
            elif op == "seek":
                reply = self.seek(cmd[1], cmd[2], cmd[3])
            elif op == "gripper":
                self.gripper(cmd[1])
            elif op == "dwell":
                self.sleep(cmd[1])
            elif op == "attach":
                self.attach[cmd[1]].publish(Empty())
                self.payload = 1
                self.event(f"attach {cmd[1]}")
                self.sleep(0.2)
            elif op == "detach":
                self.detach[cmd[1]].publish(Empty())
                self.payload = 0
                self.event(f"detach {cmd[1]}")
                self.sleep(0.2)
            elif op == "classify":
                reply = next(c["color"] for c in self.task["cubes"] if c["name"] == cmd[1])   # colour of the spawned object
            elif op == "feed":
                if cmd[1] != self.task["cubes"][0]["name"] or self.cycle > 0:
                    self.feed(cmd[1])
        self.state = "DONE"
        self.sleep(0.5)

    def run_estop_test(self):
        self.state = "HOLD"
        self.event("holding the payload at the travel height")
        self.sleep(1.0)
        self.state = "ESTOP"
        self.event("E-stop: J1..J4 drivers disabled")
        if self.estop.wait_for_service(timeout_sec=3.0):
            self.estop.call_async(SetBool.Request(data=True))
        else:
            self.get_logger().error("/estop service not available (is estop_node running?)")
        self.sleep(float(self.task["estop_test"]["duration_s"]))
        self.event("E-stop test finished; reset with: ros2 service call /estop std_srvs/srv/SetBool '{data: false}'")
        self.state = "DONE"
        self.sleep(0.5)


def main():
    rclpy.init()
    node = PickPlace()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
