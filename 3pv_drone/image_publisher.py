# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
import numpy as np
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
from v4l2py.device import Device # For detecting camera port id

# function for finding id number of camera based on name
def get_camera_id(name_contains:str, port_range:int=10):
    """
    Find the first dev/ port number (id) of a usb camera whose card
    info contains the provided string.

    Args:
        name_contains (str, optional): identifying string in camera's v4l2 card info. Defaults to "USB".
        range (int): number of ports to check. Function will check ports 0, 1, 2... up to this number.

    Returns:
        int: id number (port number i guess)
    """
    cam_cards = {}
    match_id = None
    # loop through possible ids, starting at 0
    for id in range(port_range + 1):
        cam = Device.from_id(id)
        try:
            cam.open()
            cam_cards[id] = cam.info.card
            if name_contains in cam.info.card.lower() and not match_id:
                match_id = id
        except:
            pass
    
    print(f'searching for camera whose name contains "{name_contains}"')
    print("    available cameras:")
    for id, card in cam_cards.items():
        print(f"        id {id}:", card)
    
    if match_id is not None:
        print(f"search success:\n    id {match_id}: {cam_cards[match_id]}")
        return match_id
    else: # no match found
        raise ValueError(f'no camera found with name containing "{name_contains}"')


class ImagePublisher(Node):
  """
  Create an ImagePublisher class, which is a subclass of the Node class.
  """
  def __init__(self, cam_name_contains:str):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_publisher')
      
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'image_rect', 10)
      
    # We will publish a message every 0.1 seconds
    fps = 20
    timer_period = 1/fps  # seconds
      
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
         
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(get_camera_id(name_contains=cam_name_contains))
         
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()

    # camera calbration params (copy from calibration yaml file)
    self.camera_matrix = np.array([[209.41852, 0.       , 324.61798],
                                   [0.       , 210.7007 , 245.44306],
                                   [0.       , 0.       ,   1.     ]])
    self.dist_coeffs = np.array([-0.001051, -0.013015, -0.000785, 0.000690, 0.000000])
   
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.1 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
          
    if ret == True:
      # rectify the image
      frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
      # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

      # Publish the image.
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame, encoding='bgr8'))
 
    # Display the message on the console
    self.get_logger().info('Publishing video frame')
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_publisher = ImagePublisher()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_publisher)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_publisher.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()