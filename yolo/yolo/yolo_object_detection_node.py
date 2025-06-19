#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_msgs.msg import InferenceResult, Yolo11Inference


"""Este código utiliza un modelo de YOLO11 junto con un nodo ROS2
para la detección de objetos en tiempo real. Se suscribe a las imágenes
proporcionadas por la cámara del robot, procesa las imágenes para detectar
objetos, y publica los resultados."""

bridge = CvBridge()

class YoloObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__('object_detection')

        #importamos el modelo preentrenado de YOLO11 
        self.model = YOLO('/home/paula/security_bot_ws/src/yolo/weights/best.pt')
        self.yolo11_inference = Yolo11Inference()
        
        self.subscription = self.create_subscription(
            Image, 
            '/camera/color/image_raw',
            self.camera_callback,
            10
        )

        self.yolo11_pub = self.create_publisher(Yolo11Inference, "/Yolo11_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, msg: Image) -> None:

        img = bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(img)

        self.yolo11_inference.header.frame_id = "inference"
        self.yolo11_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inf_result = InferenceResult()
    
                #obtener las coordenadas de la caja en el formato (arriba, izquierda, abajo, derecha)
                b = box.xyxy[0].to('cpu').detach().numpy().copy()

                """ box.xyxy contiene las coordenadas de la caja delimitadora en el formato
                (xmin, ymin, xmax, ymax), donde (xmin, ymin) es la coordenada de la 
                esquina superior izquierda de la caja,  y (xmax, ymax) la de la esquina
                inferior derecha.
                to('cpu').detach().numpy().copy() aseguran que el tensor que contiene las 
                coordenadas de la caja delimitadora se mueva a la memoria de la CPU, se 
                detache de cualquier gráfico de calculo, se convierta en un array de NumPy
                y luego se copie para asegurarse de que sea un objeto independiente en memoria.
                """

                c = box.cls

                """box.cls contiene el identificador de la clase del objeto detectado. Se utiliza
                para obtener el nombre de la clase del objeto detectado."""

                self.inf_result.class_name = self.model.names[int(c)]
                self.inf_result.left = int(b[0])
                self.inf_result.top = int(b[1])
                self.inf_result.right = int(b[2])
                self.inf_result.bottom = int(b[3])
                self.inf_result.box_width = (self.inf_result.right - self.inf_result.left)
                self.inf_result.box_height = (self.inf_result.bottom - self.inf_result.top)
                self.inf_result.x = self.inf_result.left + (self.inf_result.box_width/2.0)
                self.inf_result.y = self.inf_result.top + (self.inf_result.box_height/2.0)
                self.yolo11_inference.yolo11_inference.append(self.inf_result)

            annotated_frame = results[0].plot()
            img_msg = bridge.cv2_to_imgmsg(annotated_frame)
            self.img_pub.publish(img_msg)

            self.yolo11_pub.publish(self.yolo11_inference)
            self.yolo11_inference.yolo11_inference.clear()

def main(args = None) -> None:
    rclpy.init(args=args)
    object_detection = YoloObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

