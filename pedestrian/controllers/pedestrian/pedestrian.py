# export WEBOTS_HOME=/usr/local/webots

"""Pedestrian class container."""
from controller import Supervisor, Keyboard
from geometry_msgs.msg import Twist, Vector3, Pose2D
import math
import optparse


class Pedestrian (Supervisor):
    """Control a Pedestrian PROTO."""

    def __init__(self):
        """Constructor: initialize constants."""
        Supervisor.__init__(self) # superclass constructor
        # setup
        self.BODY_PARTS_NUMBER = 13
        self.WALK_SEQUENCES_NUMBER = 8
        self.ROOT_HEIGHT = 1.27
        self.CYCLE_TO_DISTANCE_RATIO = 0.22
        self.current_height_offset = 0
        self.joints_position_field = []
        self.joint_names = [
            "leftArmAngle", "leftLowerArmAngle", "leftHandAngle",
            "rightArmAngle", "rightLowerArmAngle", "rightHandAngle",
            "leftLegAngle", "leftLowerLegAngle", "leftFootAngle",
            "rightLegAngle", "rightLowerLegAngle", "rightFootAngle",
            "headAngle"
        ]
        self.height_offsets = [  # those coefficients are empirical coefficients which result in a realistic walking gait
            -0.02, 0.04, 0.08, -0.03, -0.02, 0.04, 0.08, -0.03
        ]
        self.angles = [  # those coefficients are empirical coefficients which result in a realistic walking gait
            [-0.52, -0.15, 0.58, 0.7, 0.52, 0.17, -0.36, -0.74],  # left arm
            [0.0, -0.16, -0.7, -0.38, -0.47, -0.3, -0.58, -0.21],  # left lower arm
            [0.12, 0.0, 0.12, 0.2, 0.0, -0.17, -0.25, 0.0],  # left hand
            [0.52, 0.17, -0.36, -0.74, -0.52, -0.15, 0.58, 0.7],  # right arm
            [-0.47, -0.3, -0.58, -0.21, 0.0, -0.16, -0.7, -0.38],  # right lower arm
            [0.0, -0.17, -0.25, 0.0, 0.12, 0.0, 0.12, 0.2],  # right hand
            [-0.55, -0.85, -1.14, -0.7, -0.56, 0.12, 0.24, 0.4],  # left leg
            [1.4, 1.58, 1.71, 0.49, 0.84, 0.0, 0.14, 0.26],  # left lower leg
            [0.07, 0.07, -0.07, -0.36, 0.0, 0.0, 0.32, -0.07],  # left foot
            [-0.56, 0.12, 0.24, 0.4, -0.55, -0.85, -1.14, -0.7],  # right leg
            [0.84, 0.0, 0.14, 0.26, 1.4, 1.58, 1.71, 0.49],  # right lower leg
            [0.0, 0.0, 0.42, -0.07, 0.07, 0.07, -0.07, -0.36],  # right foot
            [0.18, 0.09, 0.0, 0.09, 0.18, 0.09, 0.0, 0.09]  # head
        ]

        self.root_node_ref = self.getSelf()
        self.root_translation_field = self.root_node_ref.getField(
            "translation")
        self.root_rotation_field = self.root_node_ref.getField("rotation")
        for i in range(0, self.BODY_PARTS_NUMBER):
            self.joints_position_field.append(
                self.root_node_ref.getField(self.joint_names[i]))
            
        # conrtoller timestep
        self.timestep = 10  # ms

        # motion members
        self.pose = Pose2D()
        self.pose.x = self.root_translation_field.value[0]
        self.pose.y = self.root_translation_field.value[1]
        self.pose.theta = self.root_rotation_field.value[3]
        self.speed = 2.0
        self.current_twist = Twist(linear=Vector3(x=self.speed))  # current lin and rot velocities
        self.walk_cycle_progress = 0

        # Create a keyboard object
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.timestep)

 
    def run(self):
        """get control from keyboard and move the character accordingly with joint animations"""
        # call controller step method
        while self.step(self.timestep) != -1:
            self.keyboard_control()
            self.update_pose(self.timestep)
            self.update_joints(self.timestep)
    
    def update_pose(self, timestep:float) -> None:
        """ update character's global pose via current twist command """
        # compute global pose via adding deltas (velocity * timestep)
        timestep = timestep/ 1000 # convert to sec
        self.pose.x += math.cos(self.pose.theta) * self.current_twist.linear.x * timestep
        self.pose.y += math.sin(self.pose.theta) * self.current_twist.linear.x * timestep
        self.pose.theta += self.current_twist.angular.z * timestep

        # set global pose
        root_translation = [self.pose.x, self.pose.y,
                            self.ROOT_HEIGHT + self.current_height_offset]
        self.root_translation_field.setSFVec3f(root_translation)
        rotation = [0, 0, 1, self.pose.theta]
        self.root_rotation_field.setSFRotation(rotation)
        
    def update_joints(self, timestep:float) -> None:
        """ adjust joints based on walk cycle progress (linear interpoaltion between key poses) """
        timestep = timestep / 1000  # convert to sec
        # compute current progress through walk cycle
        x_delta = self.current_twist.linear.x * timestep
        self.walk_cycle_progress = (self.walk_cycle_progress + 2*x_delta * self.CYCLE_TO_DISTANCE_RATIO) % 1

        # get current walk sequence number
        current_sequence = int(self.walk_cycle_progress *
                               self.WALK_SEQUENCES_NUMBER)
        sequence_progress = (self.walk_cycle_progress *
                             self.WALK_SEQUENCES_NUMBER) % 1

        # # compute the ratio 'distance already covered between way-point(X) and way-point(X+1)'
        # # / 'total distance between way-point(X) and way-point(X+1)'
        # ratio = (self.time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO - \
        #     int(((self.time * self.speed) / self.CYCLE_TO_DISTANCE_RATIO))

        # loop through each joint and update its angle according to the defined sequence
        for i in range(0, self.BODY_PARTS_NUMBER):
            angle_1 = self.angles[i][current_sequence]
            angle_2 = self.angles[i][(current_sequence + 1)%self.WALK_SEQUENCES_NUMBER]
            current_angle = angle_1 + (angle_2-angle_1) * sequence_progress
            self.joints_position_field[i].setSFFloat(current_angle)

        # adjust height
        h1 = self.height_offsets[current_sequence]
        h2 = self.height_offsets[(current_sequence + 1)%self.WALK_SEQUENCES_NUMBER]
        self.current_height_offset = h1 + (h2 - h1) * sequence_progress

    def keyboard_control(self):
        """ use keyboard to control character's motion """
        self.current_twist = Twist()  # start with a blank twist

        key = self.keyboard.get_key()  # Read input from keyboard
        
        # repeat keyboard detection until all pressed keys are acted on
        while key in ["up", "down", "left", "right"]:
            if key == "up":
                print("up")
                self.current_twist.linear.x = self.speed
            elif key == "down":
                print("down")
                self.current_twist.linear.x = -self.speed
            elif key == "right":
                self.current_twist.angular.z = -self.speed
            elif key == "left":  
                print("left")
                self.current_twist.angular.z = self.speed

            key = self.keyboard.get_key()  # reread keyboard


controller = Pedestrian()
controller.run()
