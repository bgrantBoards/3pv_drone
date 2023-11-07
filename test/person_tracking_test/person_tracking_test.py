# import cv2
# import imutils
  
# # Initializing the HOG person
# # detector
# hog = cv2.HOGDescriptor()
# hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
  
# # Reading the Image
# # image = cv2.imread('img.png')
# image = cv2.imread("3pv_drone/test/person_tracking_test/test4.png")
# image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert to gryscale
# image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) # convert back to color ?

# # Resizing the Image
# image = imutils.resize(image,
#                        width=min(400, image.shape[1]))
  
# # Detecting all the regions in the 
# # Image that has a pedestrians inside it
# (regions, _) = hog.detectMultiScale(image, 
#                                     winStride=(4, 4),
#                                     padding=(4, 4),
#                                     scale=1.05)
  
# # Drawing the regions in the Image
# for (x, y, w, h) in regions:
#     cv2.rectangle(image, (x, y), 
#                   (x + w, y + h), 
#                   (0, 0, 255), 2)
 
# # Showing the output Image
# cv2.imshow("Image", image)
# cv2.waitKey(0)
  
# cv2.destroyAllWindows()

# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import imutils
 
class PersonTracker(Node):
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('person_detector')
            
        # Create the publisher. This publisher will publish an Image
        # to the video_frames topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(Image, 'person_detection', 10)

        # create a subscriber object that listens for new images and runs person detection process on them
        # self.subscriber_ = self.create_subscription(Image, 'screen_record', self.detect_person, 10)
        self.subscriber_ = self.create_subscription(Image, 'video_raw', self.detect_person, 10)
                
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Initializing the HOG person
        # detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
   
    def detect_person(self, data):
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        image = self.br.imgmsg_to_cv2(data)

        # image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # convert to grayscale
        # image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) # convert back to color

        # Resizing the Image
        # image = imutils.resize(current_frame,
        #                     width=min(400, current_frame.shape[1]))
        
        # Detecting all the regions in the 
        # Image that has a pedestrians inside it
        (regions, _) = self.hog.detectMultiScale(image, 
                                            winStride=(4, 4),
                                            padding=(4, 4),
                                            scale=1.05)
        
        # Drawing the regions in the Image
        for (x, y, w, h) in regions:
            cv2.rectangle(image, (x, y), 
                        (x + w, y + h), 
                        (0, 0, 255), 2)

        processed_frame = image
        
        # publish processed image
        self.publisher_.publish(self.br.cv2_to_imgmsg(processed_frame,  encoding='bgr8'))
        # cv2.imshow("camera", current_frame)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  tracker = PersonTracker()
  
  # Spin the node so the callback function is called.
  rclpy.spin(tracker)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  tracker.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()