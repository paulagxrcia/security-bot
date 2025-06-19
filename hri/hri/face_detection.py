import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ament_index_python.packages import get_package_share_directory

class FaceDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__('face_detector_node')
        self.subscription = self.create_subscription(
            Image,
            '/deepmind_robot1/deepmind_robot1_camera/image_raw',  
            self.image_callback,
            10)
        self.face_publisher = self.create_publisher(
            Image,
            '/detected_faces/faces',  
            10)
        self.bridge = CvBridge()
        data_folder = os.path.join(get_package_share_directory('hri'), 'data')
        face_cascade_path = os.path.join(data_folder, 'haarcascade_frontalface_default.xml')
       
        if not os.path.isfile(face_cascade_path):
            self.get_logger().error("Cascade xml file not found")
            return

        self.face_cascade = cv2.CascadeClassifier(face_cascade_path)


    def image_callback(self, msg: Image) -> None:
        """ Detects faces, drawing blue rectangles around them"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray_image, scaleFactor=1.3, minNeighbors=5)
        if len(faces) > 0:
            self.get_logger().info("Face detected")
        for (x, y, w, h) in faces:
            # draw blue box surrounding detected face
            cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

        self.face_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        

def main(args=None) -> None:
    rclpy.init(args=args)
    face_detector_node = FaceDetectorNode()
    rclpy.spin(face_detector_node)
    face_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
