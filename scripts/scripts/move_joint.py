import rclpy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration

def main():
    rclpy.init()
    node = rclpy.create_node('move_joint_node')
    pub = node.create_publisher(JointTrajectory, '/extendable_mount_controller/joint_trajectory', 10)

    # Crear el mensaje de trayectoria
    trajectory = JointTrajectory()
    trajectory.header = Header(stamp=node.get_clock().now().to_msg(), frame_id='base_link')
    trajectory.joint_names = ['fixed_pole_link_to_extendable_pole_link_joint']

    # Primer punto de la trayectoria
    point1 = JointTrajectoryPoint()
    point1.positions = [0.1]  # Cambia la posición inicial
    point1.velocities = [0.1]  # Velocidad intermedia
    point1.time_from_start = Duration(sec=1)

    # Segundo punto de la trayectoria (final)
    point2 = JointTrajectoryPoint()
    point2.positions = [0.2]  # Posición final
    point2.velocities = [0.0]  # Velocidad final debe ser cero
    point2.time_from_start = Duration(sec=2)

    # Asignamos los puntos a la trayectoria
    trajectory.points = [point1, point2]

    # Publicar la trayectoria
    pub.publish(trajectory)
    rclpy.spin_once(node)

    # Finalizar el nodo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
