# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import numpy as np
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from utils import read_camera_params, get_camera_id

class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self, calibration_yaml:str, cam_name_contains:str):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.raw_publisher = self.create_publisher(Image, 'image_raw', 10)
    self.rect_publisher = self.create_publisher(Image, 'image_rect', 10)
      
    # We will publish a message every 0.1 seconds
    fps = 20
    timer_period = 1/fps  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
    # self.cap = cv2.VideoCapture(get_camera_id(name_contains=cam_name_contains)) # TODO: uncomment 
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # camera calbration params (read from yaml file)
    (self.camera_matrix, self.dist_coeffs) = read_camera_params(calibration_yaml)
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    ret, frame = self.cap.read()
          
    if ret == True:
      # change color encoding
      frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

      # Publish the raw image.
      self.raw_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding='rgb8'))

      # rectify the image
      frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

      # Publish the rectified image.
      self.rect_publisher.publish(self.br.cv2_to_imgmsg(frame, encoding='rgb8'))

    # Display status message on the console
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  # TODO: add arg for setting calibration_yaml
  image_publisher = ImagePublisher(calibration_yaml="fpv_cam_calibration.yaml", cam_name_contains="USB2.0")
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()