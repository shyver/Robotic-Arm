#!/usr/bin/env python3
"""E-stop of the stepper joints J1..J4, modelled like the hardware: the drivers are disabled, so the joints
have no holding torque (the servos J5, J6 and the gripper stay powered).

  ros2 service call /estop std_srvs/srv/SetBool "{data: true}"     -> press  (torque of J1..J4 = 0)
  ros2 service call /estop std_srvs/srv/SetBool "{data: false}"    -> release (controller holds the current pose)
  ros2 topic pub -1 /estop_trigger std_msgs/msg/Bool "{data: true}"   (same, as a topic)

Implementation: stepper_controller (joint trajectory controller) is deactivated and estop_effort_controller
(forward effort controller) is activated and fed with zeros.
"""
import rclpy
from controller_manager_msgs.srv import SwitchController
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import SetBool


class EStop(Node):
    def __init__(self):
        super().__init__("estop")
        self.active = False
        self.switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.zero = self.create_publisher(Float64MultiArray, "/estop_effort_controller/commands", 10)
        self.state = self.create_publisher(Bool, "/estop_active", 10)
        self.create_service(SetBool, "/estop", self.on_srv)
        self.create_subscription(Bool, "/estop_trigger", lambda m: self.set(m.data), 10)
        self.create_timer(0.02, self.tick)

    def tick(self):
        self.state.publish(Bool(data=self.active))
        if self.active:
            self.zero.publish(Float64MultiArray(data=[0.0, 0.0, 0.0, 0.0]))

    def set(self, press):
        req = SwitchController.Request()
        if press:
            req.deactivate_controllers, req.activate_controllers = ["stepper_controller"], ["estop_effort_controller"]
        else:
            req.deactivate_controllers, req.activate_controllers = ["estop_effort_controller"], ["stepper_controller"]
        req.strictness = SwitchController.Request.BEST_EFFORT
        req.activate_asap = True
        if not self.switch.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("controller_manager not available")
            return False
        self.switch.call_async(req)
        self.active = bool(press)
        self.get_logger().warn("E-STOP PRESSED: J1..J4 torque removed" if press else "E-stop released: J1..J4 hold the current pose")
        return True

    def on_srv(self, req, res):
        res.success = self.set(req.data)
        res.message = "estop active" if self.active else "estop released"
        return res


def main():
    rclpy.init()
    node = EStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
