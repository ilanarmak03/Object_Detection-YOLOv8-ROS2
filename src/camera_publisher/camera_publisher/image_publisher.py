# Libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# The program

class CameraPublisher(Node):
# Other Functions:
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ =self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_frame)
        self.cap = cv2.VideoCapture(0)
        self.bridge = CvBridge()
    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame )
            self.publisher_.publish(msg)
    def __del__(self):
        self.cap.release
# Main:
def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
        
if __name__=='__main__':
    main() 