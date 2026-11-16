#!/usr/bin/env python3
"""Screenshots and clip from the fixed cell camera (/cell_camera/image).

  ros2 run robotic_arm_control camera_capture --ros-args -p out_dir:=$HOME/ras_arm_logs/media -p video_seconds:=50.0

Writes gazebo_world.png (first frame), gazebo_arm.png (first frame of the first GRASP state) and, if
video_seconds > 0, pick_place.mp4 starting at the first APPROACH state (needs python3-opencv).
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class Capture(Node):
    def __init__(self):
        super().__init__("camera_capture")
        self.declare_parameter("out_dir", os.path.expanduser("~/ras_arm_logs/media"))
        self.declare_parameter("video_seconds", 50.0)
        self.declare_parameter("fps", 30.0)
        self.out = self.get_parameter("out_dir").value
        os.makedirs(self.out, exist_ok=True)
        self.seconds = float(self.get_parameter("video_seconds").value)
        self.fps = float(self.get_parameter("fps").value)
        self.state = "INIT"
        self.have_world = self.have_arm = False
        self.writer, self.frames, self.done = None, 0, False
        self.create_subscription(Image, "/cell_camera/image", self.on_image, 10)
        self.create_subscription(String, "/arm_status", lambda m: setattr(self, "state", m.data.split("|")[0]), 10)

    def on_image(self, m):
        import cv2
        img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step // 3 if m.step else m.width, 3)[:, :m.width]
        bgr = img[:, :, ::-1] if m.encoding.lower().startswith("rgb") else img
        if not self.have_world:
            cv2.imwrite(os.path.join(self.out, "gazebo_world.png"), bgr)
            self.have_world = True
            self.get_logger().info("saved gazebo_world.png")
        if not self.have_arm and self.state == "GRASP":
            cv2.imwrite(os.path.join(self.out, "gazebo_arm.png"), bgr)
            self.have_arm = True
            self.get_logger().info("saved gazebo_arm.png")
        if self.seconds > 0 and not self.done and (self.writer is not None or self.state == "APPROACH"):
            if self.writer is None:
                self.writer = cv2.VideoWriter(os.path.join(self.out, "pick_place.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                                              self.fps, (m.width, m.height))
                self.get_logger().info("recording pick_place.mp4 ...")
            self.writer.write(np.ascontiguousarray(bgr))
            self.frames += 1
            if self.frames >= self.seconds * self.fps:
                self.writer.release()
                self.done = True
                self.get_logger().info(f"saved pick_place.mp4 ({self.frames} frames) in {self.out}")


def main():
    rclpy.init()
    node = Capture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    if node.writer is not None and not node.done:
        node.writer.release()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
