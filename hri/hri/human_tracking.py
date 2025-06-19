import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from geometry_msgs.msg import Twist

# Reemplazo de non_max_suppression de imutils
def non_max_suppression_fast(boxes, overlapThresh=0.65):
    if len(boxes) == 0:
        return []

    boxes = boxes.astype("float")
    pick = []

    x1 = boxes[:,0]
    y1 = boxes[:,1]
    x2 = boxes[:,2]
    y2 = boxes[:,3]

    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    idxs = np.argsort(y2)

    while len(idxs) > 0:
        last = idxs[-1]
        i = idxs[-1]
        pick.append(i)

        xx1 = np.maximum(x1[i], x1[idxs[:-1]])
        yy1 = np.maximum(y1[i], y1[idxs[:-1]])
        xx2 = np.minimum(x2[i], x2[idxs[:-1]])
        yy2 = np.minimum(y2[i], y2[idxs[:-1]])

        w = np.maximum(0, xx2 - xx1 + 1)
        h = np.maximum(0, yy2 - yy1 + 1)

        overlap = (w * h) / area[idxs[:-1]]

        idxs = np.delete(
            idxs,
            np.concatenate(([last], np.where(overlap > overlapThresh)[0]))
        )

    return boxes[pick].astype("int")

class HumanTrackerNode(Node):
    def __init__(self) -> None:
        super().__init__('human_tracker')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.human_publisher = self.create_publisher(
            Image,
            '/detected_human',
            10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def image_callback(self, msg: None) -> None:
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        (rects, _) = self.hog.detectMultiScale(cv_image, winStride=(8, 8),
                                               padding=(8, 8), scale=1.05)

        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression_fast(rects, overlapThresh=0.65)
        self.get_logger().info(f'Detected {len(pick)} humans')

        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(cv_image, (xA, yA), (xB, yB), (0, 255, 0), 2)

        image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.human_publisher.publish(image_msg)

        if len(pick) > 0:
            (xA, yA, xB, yB) = pick[0]
            human_center_x = (xA + xB) / 2
            image_center_x = cv_image.shape[1] / 2
            offset_x = human_center_x - image_center_x
            gain = 0.001 
            angular_velocity = -offset_x * gain
            self.get_logger().info(f'Computed angular velocity: {angular_velocity}')

            twist_msg = Twist()
            twist_msg.angular.z = angular_velocity
            self.cmd_vel_publisher.publish(twist_msg)

def main(args=None) -> None:
    rclpy.init(args=args)
    human_tracker_node = HumanTrackerNode()
    rclpy.spin(human_tracker_node)
    human_tracker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
