#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from simple_pid import PID
from queue import Queue

class CrazyfliePositionController(Node):

    def __init__(self):
        super().__init__("crazyflie_motion_control")

        # subscriber listens to aruco pose detection topic
        self.create_subscription(
            msg_type    = Pose,
            topic       = "aruco_pose",
            qos_profile = 1,
            callback    = self.recieve_relative_tag_pose
            )
        
        # publisher for sending velocity commands to Crazyflie drone
        self.cmd_vel_pub = self.create_publisher(
            msg_type    = Twist,
            topic       = "cmd_vel",
            qos_profile = 1
        )

        # timer for regularly publishing velocity command msgs
        self.create_timer(
            timer_period_sec = 20/1000,
            callback         = self.send_velocity_command
        )
        
        # pid controllers
        self.pid_x = PID(Kp = -1, Ki = -0.1, Kd = -0.001, setpoint = 2)

        # current state
        self.tag_pose_relative:Pose = None
        self.filter_size = 100
        self.x_filter = list() # for implementing rolling average noise filter
    
    def recieve_relative_tag_pose(self, msg:Pose) -> None:
        self.tag_pose_relative = msg
        while len(self.x_filter) < self.filter_size + 1:
            self.x_filter.append(msg.position.z) # z because how aruco detector publishes forard position as z
        self.x_filter.pop(0)

    def send_velocity_command(self) -> None:
        # if current tag pose is not None
        if self.tag_pose_relative:
            cmd_vel_msg = Twist() # instantiate empty Twist msg

            # populate Twist msg with PID computed control velocities
            cmd_vel_msg.linear.x = self.pid_x(sum(self.x_filter)/self.filter_size)

            # send Twist msg
            self.cmd_vel_pub.publish(cmd_vel_msg)
        

def main(args=None):
    rclpy.init(args=args)
    motion_controller = CrazyfliePositionController()
    rclpy.spin(motion_controller)
    rclpy.shutdown()

if __name__ == '__main__':
    main()