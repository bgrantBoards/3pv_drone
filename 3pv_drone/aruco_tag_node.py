# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import cv2.aruco as aruco # aruco tag module of OpenCV library
from utils import read_camera_params, estimatePoseSingleMarkers
import numpy as np
from geometry_msgs.msg import Pose
import time
 
class ArucoDetector(Node):
	"""
	Create an ImageSubscriber class, which is a subclass of the Node class.
	"""
	def __init__(self, calibration_yaml:str):
		"""
		Class constructor to set up the node
		"""
		# Initiate the Node class's constructor and give it a name
		super().__init__('aruco_detector')
		
		# Create the subscriber. This subscriber will receive an Image
		# from the video_frames topic. The queue size is 10 messages.
		self.subscription = self.create_subscription( 
			Image, 
			'image_rect', 
			self.detect_tags, 
			10)
		
		# image publisher (for tag detection overlay)
		self.img_publisher = self.create_publisher(Image, "aruco_image", 10)

		# tag pose publisher
		self.tag_pose_publisher = self.create_publisher(Pose, "aruco_pose", 1)
			
		# Used to convert between ROS and OpenCV images
		self.br = CvBridge()

		# camera calbration params (read from yaml file)
		(self.camera_matrix, self.dist_coeffs) = read_camera_params(calibration_yaml)

		# init aruco tag detector
		self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)  # Use 4x4 dictionary to find markers
		parameters = aruco.DetectorParameters()  # Marker detection parameters
		self.aruco_detector = aruco.ArucoDetector(dictionary=self.aruco_dict, detectorParams=parameters)
		self.tag_pose:PoseStamped = None

	def detect_tags(self, data):
		# Display status message on the console
		# self.get_logger().info('Receiving video frame')
 
		# Convert ROS Image message to OpenCV image
		frame = self.br.imgmsg_to_cv2(data)

		# operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		# lists of ids and the corners beloning to each id
		corners, ids, rejected_img_points = self.aruco_detector.detectMarkers(gray)	
		
		if np.all(ids is not None):  # If there are markers found by detector
			for id in ids: # for each detected tag id``
					# Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
					# board = aruco.Board(corners, self.aruco_dict, np.array(ids))
					# object_points, image_points = board.matchImagePoints(corners, id)
					rvec, tvec, markerPoints = estimatePoseSingleMarkers(corners, 0.165, self.camera_matrix, self.dist_coeffs)
					(rvec - tvec).any()  # get rid of that nasty numpy value array error
					aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
					try:
						cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)  # Draw Axis
						self._publish_pose(rvec, tvec)
					except:
						pass
		
		# Display image)
		self.img_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding="rgb8"))
	
	def _publish_pose(self, rvec:np.ndarray, tvec:np.ndarray):
		tvec = tvec.reshape(3,1)
		tvec = [tvec[0][0], tvec[1][0], tvec[2][0]]
		rvec = rvec.reshape(3,1)
		rvec = [rvec[0][0], rvec[1][0], rvec[2][0]]

		# Create a PoseStamped message
		pose = Pose()
		pose.position.x = tvec[0]
		pose.position.y = tvec[1]
		pose.position.z = tvec[2]
		pose.orientation.w = 0.
		pose.orientation.x = rvec[0]
		pose.orientation.y = rvec[1]
		pose.orientation.z = rvec[2]

		print("detected tag\n"+\
		f"x:{pose.orientation.x} "+\
		f"y:{pose.orientation.y} "+\
		f"z:{pose.orientation.z}\n"
		)

		# Publish the pose message
		self.tag_pose_publisher.publish(pose)

def main(args=None):
	
	# Initialize the rclpy library
	rclpy.init(args=args)
	
	# Create the node
	image_subscriber = ArucoDetector(calibration_yaml="dell_webcam_calibration.yaml")
	
	# Spin the node so the callback function is called.
	rclpy.spin(image_subscriber)
	
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	image_subscriber.destroy_node()
	
	# Shutdown the ROS client library for Python
	rclpy.shutdown()
	
if __name__ == '__main__':
	main()