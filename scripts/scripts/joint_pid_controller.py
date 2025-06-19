import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float64
import time
import csv
import curses
import threading
import math

class JointEffortPIDController(Node):
    def __init__(self):
        super().__init__('joint_effort_pid_controller')

        # PID parámetros ajustados
        self.Kp = 25.0
        self.Ki = 15.0
        self.Kd = 8.0

        # Estado del PID
        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now().nanoseconds / 1e9

        # Posición deseada
        self.target_position = 0.0

        # Límites
        self.max_effort = 10.0
        self.min_effort = -10.0
        self.max_integral = 5.0
        self.min_integral = -5.0

        # Variables de logging
        self.start_time = time.time()
        self.last_log_time = self.start_time

        # Publicadores y suscriptores
        self.create_subscription(Float64,
                                 '/camera_joint_target_position',
                                 self.target_position_callback, 10)
        self.publisher = self.create_publisher(Float64MultiArray,
                                               '/extendable_mount_controller/commands', 10)
        self.create_subscription(JointState,
                                 '/joint_states',
                                 self.joint_state_callback, 10)

        # Hilo de teclado
        t = threading.Thread(target=self.keyboard_input_thread, daemon=True)
        t.start()

        self.get_logger().info('Nodo joint_effort_pid_controller iniciado')

    def target_position_callback(self, msg):
        self.target_position = msg.data
        self.error_sum = 0.0
        self.last_error = 0.0
        # Solo un log al cambiar target
        self.get_logger().info(f'Target actualizado: {self.target_position:.2f} m')

    def joint_state_callback(self, msg):
        try:
            idx = msg.name.index('fixed_pole_link_to_extendable_pole_link_joint')
            current_pos = msg.position[idx]
        except ValueError:
            return

        now_ros = self.get_clock().now().nanoseconds / 1e9
        dt = now_ros - self.last_time
        if dt <= 0.0:
            return

        # PID
        error = self.target_position - current_pos
        self.error_sum = max(min(self.error_sum + error * dt, self.max_integral), self.min_integral)
        d_error = (error - self.last_error) / dt
        raw = self.Kp * error + self.Ki * self.error_sum + self.Kd * d_error

        # Aplicar break-away mínimo
        min_breakaway = 5.0
        if abs(raw) > 0.01:
            effort = math.copysign(max(abs(raw), min_breakaway), raw)
        else:
            effort = 0.0

        # Limitar esfuerzo
        effort = max(min(effort, self.max_effort), self.min_effort)

        # Publicar esfuerzo
        out = Float64MultiArray()
        out.data = [effort]
        self.publisher.publish(out)

        # Log solo cada 1 segundo
        now = time.time()
        if now - self.last_log_time >= 1.0:
            t = now - self.start_time
            self.get_logger().info(f't={t:.1f}s | pos={current_pos:.3f} | eff={effort:.2f}')
            self.last_log_time = now

        self.last_error = error
        self.last_time = now_ros

    def destroy_node(self):
        # Enviar esfuerzo cero al cerrar
        zero_msg = Float64MultiArray()
        zero_msg.data = [0.0]
        self.publisher.publish(zero_msg)
        self.get_logger().info('Esfuerzo puesto a 0, nodo detenido')
        super().destroy_node()

    def keyboard_input_thread(self):
        stdscr = curses.initscr()
        curses.cbreak()
        stdscr.keypad(True)
        stdscr.timeout(100)
        try:
            while True:
                key = stdscr.getch()
                if key == curses.KEY_UP:
                    self.target_position += 0.025
                    # log mínimo
                    self.get_logger().info(f'Target ↑ {self.target_position:.2f} m')
                elif key == curses.KEY_DOWN:
                    self.target_position -= 0.025
                    self.get_logger().info(f'Target ↓ {self.target_position:.2f} m')
                elif key == ord('q'):
                    break
                time.sleep(0.1)
        finally:
            curses.endwin()

def main(args=None):
    rclpy.init(args=args)
    node = JointEffortPIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
