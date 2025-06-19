import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AlarmPublisher(Node):
    def __init__(self):
        super().__init__('alarm_publisher')
        self.publisher_ = self.create_publisher(String, '/alarm', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):   
        msg = String()
        msg.data = "ALARM: Anomaly Detected!"
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AlarmPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()