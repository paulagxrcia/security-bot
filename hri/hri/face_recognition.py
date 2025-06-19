import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import face_recognition
import os
from ament_index_python.packages import get_package_share_directory
from typing import Dict, List, Any

class FaceRecognitionNode(Node):
    def __init__(self) -> None:
        super().__init__('face_recognition_node')
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

        # Load reference database
        self.reference_database = self.load_reference_database()

    def load_reference_database(self) -> Dict[str, List[Any]]:
        """ Populate a database with facial encodings from a predefined set of reference images """
        reference_database = {}
        reference_images_directories = [
            os.path.join(get_package_share_directory('hri'), 'data', 'person1'),
            os.path.join(get_package_share_directory('hri'), 'data', 'mona_lisa')
        ]

        for directory in reference_images_directories:
            # Retrieving person name by accessing last component of the 'directory' path
            person_name = os.path.basename(directory)
            reference_database[person_name] = []

            # Iterate over each file in specified directory
            for filename in os.listdir(directory):
                # searches for images files
                if filename.endswith('.png') or filename.endswith('.jpg'):
                    image_path = os.path.join(directory, filename)
                    # load image file and convert it into an image format compatible with the library
                    reference_image = face_recognition.load_image_file(image_path)
                    # compute the face encodings
                    reference_encoding = face_recognition.face_encodings(reference_image)
                    # checks if at least one face is detected in the image
                    if len(reference_encoding) > 0:
                        reference_database[person_name].append(reference_encoding[0])
                        self.get_logger().info(f"Face encoding found in {filename}! Let's add it to reference database for {person_name}")
                    else:
                        self.get_logger().warning(f"No face detected in {os.path.basename(directory)}/{filename}. Skipping...")

        self.get_logger().info(f"Finished loading the reference database")
        return reference_database

    def image_callback(self, msg: Image) -> None:
        """ Processes Image data to perform face detection and recognition """
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # Converts the BGR image to grayscale, as face detection usually requires grayscale images
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Face detection using preloaded Haar Cascade classifier
        faces = self.face_cascade.detectMultiScale(gray_image, scaleFactor=1.3, minNeighbors=5)
        # Check if any faces were detected
        if len(faces) > 0:
             # Process each detected face
            for (x, y, w, h) in faces:
                # Extract face ROI
                face_image = cv_image[y:y+h, x:x+w]
                # Convert to RGB (face_recognition uses RGB format)
                rgb_face_image = cv2.cvtColor(face_image, cv2.COLOR_BGR2RGB)
                # Generate a face encoding for the extracted face region
                face_encodings = face_recognition.face_encodings(rgb_face_image)

                 # Check if at least one face encoding was generated
                if len(face_encodings) > 0:
                    # Compare with reference encodings
                    for person_name, reference_encodings in self.reference_database.items():
                        for reference_encoding in reference_encodings:
                            # Compare the detected face encoding with each reference encoding
                            match = face_recognition.compare_faces([reference_encoding], face_encodings[0])
                            if match[0]:
                                # If match found, draw rectangle around detected face
                                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
                                # Draw label with person's name
                                cv2.putText(cv_image, person_name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                                self.get_logger().info(f"Recognized {person_name}!")
                                break
                else:
                    self.get_logger().warning("No face encoding detected.")

        else:
            self.get_logger().error("No faces detected yet ...")

        self.face_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))


def main(args=None) -> None:
    rclpy.init(args=args)
    face_recognition_node = FaceRecognitionNode()
    rclpy.spin(face_recognition_node)
    face_recognition_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
