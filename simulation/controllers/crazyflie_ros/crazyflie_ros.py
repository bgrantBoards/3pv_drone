# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

# Webots import
from controller import Robot, Camera

# Other imports
from pid_controller import pid_velocity_fixed_height_controller
from math import cos, sin
import numpy as np
from cv_bridge import CvBridge
import cv2


class CrazyflieVelocityControllerWebots(Node):
    """ PID velocity controller for Crazylie in webots simulation """

    def __init__(self, altitude: float = 2.0):

        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Initialize motors
        self.m1 = self.robot.getDevice("m1_motor")
        self.m1.setPosition(float('inf'))
        self.m1.setVelocity(-1)
        self.m2 = self.robot.getDevice("m2_motor")
        self.m2.setPosition(float('inf'))
        self.m2.setVelocity(1)
        self.m3 = self.robot.getDevice("m3_motor")
        self.m3.setPosition(float('inf'))
        self.m3.setVelocity(-1)
        self.m4 = self.robot.getDevice("m4_motor")
        self.m4.setPosition(float('inf'))
        self.m4.setVelocity(1)

        # Initialize Sensors
        self.imu = self.robot.getDevice("inertial_unit")  # imu
        self.imu.enable(self.timestep)
        self.gps = self.robot.getDevice("gps")           # gps
        self.gps.enable(self.timestep)
        self.gyro = self.robot.getDevice("gyro")         # gyro
        self.gyro.enable(self.timestep)
        # self.camera = self.robot.getDevice("camera")
        self.camera:Camera = self.robot.getDevice("camera")     # camera
        self.camera.enable(self.timestep)

        # Initialize state variables
        self.past_x_global = self.gps.getValues()[0]
        self.past_y_global = self.gps.getValues()[1]
        self.past_time = self.robot.getTime()

        # Crazyflie velocity PID controller
        self.PID = pid_velocity_fixed_height_controller()
        self.PID_update_last_time = self.robot.getTime()
        self.sensor_read_last_time = self.robot.getTime()

        # PID controller setpoints
        self.cmd_vel_x = 0
        self.cmd_vel_y = 0
        self.cmd_ang_w = 0
        self.height_desired = altitude

        # initialize ROS Node superclass
        super().__init__('crazyflie_controller_webots')

        # object for converting cv2 image to ROS img msg
        self.br = CvBridge()

        # create ROS subscriber for reading in velocity commands
        self.create_subscription(
            msg_type=Twist,
            topic="cmd_vel",
            callback=self.update_setpoints,
            qos_profile=10
        )

        # create ROS publisher for camera images
        self.camera_publisher = self.create_publisher(
            msg_type=Image,
            topic="image_rect",
            qos_profile=10
        )

        # create timer to run the controller step by step
        self.create_timer(self.timestep/1000, self.step)

    def step(self) -> None:
        """ Run control step """

        if self.robot.step(self.timestep) != -1:
            data = self.camera.getImage()
            img = np.frombuffer(data, dtype=np.uint8).reshape(
                (self.camera.getHeight(), self.camera.getWidth(), 4))
            # roi = img[:, :, 0:3]
            # print(np.shape(roi))
            
            # gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            # camera_data = self.camera.getImageArray()
            
            # # image = np.frombuffer(camera_data, np.uint8).reshape(
            # #     (self.camera.getHeight(), self.camera.getWidth(), 4))
            # cv2.imshow("preview", img)
            # cv2.waitKey(self.timestep)
            # print(image)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self.camera_publisher.publish(self.br.cv2_to_imgmsg(img, encoding="rgb8"))

            dt = self.robot.getTime() - self.past_time
            actual_state = {}

            # Get sensor data
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            yaw_rate = self.gyro.getValues()[2]
            x_global, y_global, altitude = self.gps.getValues()
            v_x_global = (x_global - self.past_x_global)/dt
            v_y_global = (y_global - self.past_y_global)/dt

            # Get body fixed velocities
            v_x = v_x_global * cos(yaw) + v_y_global * sin(yaw)
            v_y = - v_x_global * sin(yaw) + v_y_global * cos(yaw)

            # set motor thrust via PID velocity controller with fixed height
            motor_power = self.PID.pid(dt, self.cmd_vel_x, self.cmd_vel_y,
                                        self.cmd_ang_w, self.height_desired,
                                        roll, pitch, yaw_rate,
                                        altitude, v_x, v_y)

            if not np.nan in motor_power:
                self.m1.setVelocity(-motor_power[0])
                self.m2.setVelocity(motor_power[1])
                self.m3.setVelocity(-motor_power[2])
                self.m4.setVelocity(motor_power[3])

            # update time and state variables
            self.past_time = self.robot.getTime()
            self.past_x_global = x_global
            self.past_y_global = y_global

    def update_setpoints(self, cmd: Twist) -> None:
        """
        ROS \cmd_vel subscription callback:
        update controller PID setpoints from Twist message
        """
        self.cmd_vel_x = float(cmd.linear.x)
        self.cmd_vel_y = float(cmd.linear.y)
        self.cmd_ang_w = float(cmd.angular.z)


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    # TODO: add arg for setting calibration_yaml
    crazyflie = CrazyflieVelocityControllerWebots(altitude=2.0)

    # Spin the node so the callback function is called.
    rclpy.spin(crazyflie)

    # Destroy the node explicitly
    crazyflie.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
  main()
