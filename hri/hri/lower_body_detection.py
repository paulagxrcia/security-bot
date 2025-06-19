import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class BodyDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('body_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  
            self.image_callback,
            10)
        self.body_publisher = self.create_publisher(
            Image,
            '/detected_body/body',  
            10)
        self.bridge = CvBridge()
        data_folder = os.path.join(get_package_share_directory('hri'), 'data')
        body_cascade_path = os.path.join(data_folder, 'haarcascade_lowerbody.xml')
       
        if not os.path.isfile(body_cascade_path):
            self.get_logger().error("Cascade xml file not found")
            return

        self.body_cascade = cv2.CascadeClassifier(body_cascade_path)


    def image_callback(self, msg: Image) -> None:
        """ Detects bodies, drawing blue rectangles around them"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        bodies = self.body_cascade.detectMultiScale(gray_image, scaleFactor=1.3, minNeighbors=5)
        if len(bodies) > 0:
            self.get_logger().info("Worker detected")
        for (x, y, w, h) in bodies:
            # draw blue box surrounding detected face
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(cv_image, 'Worker', (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)


        self.body_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        

def main(args=None) -> None:
    rclpy.init(args=args)
    body_detector_node = BodyDetectorNode()
    rclpy.spin(body_detector_node)
    body_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()