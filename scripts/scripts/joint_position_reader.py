import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointPositionReader(Node):
    def __init__(self):
        super().__init__('joint_position_reader')

        self.subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        try:
            index = msg.name.index('fixed_pole_link_to_extendable_pole_link_joint')
            position = msg.position[index]
            self.get_logger().info(f'Posición actual: {position:.4f}')
        except ValueError:
            self.get_logger().warn('Joint no encontrado en /joint_states')

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionReader()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()