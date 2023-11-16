#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from simple_pid import PID

class CrazyfliePositionController(Node):

    def __init__(self):
        super().__init__("crazyflie_motion_control")

        # subscriber listens to aruco pose detection topic
        self.create_subscription(
            msg_type = Pose,
            topic = "aruco_pose",
            qos_profile = 1,
            callback = self.update_setpoints
            )
        
        # publisher for commanding 
        
        # pid controllers
        self.pid_x = PID(Kp = 1, Ki = 0, Kd = 0, setpoint = 2)
    
    def update_setpoints(self, msg:Twist) -> None:
        pass
        

    def pid_control(self) -> None:
        pass
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()