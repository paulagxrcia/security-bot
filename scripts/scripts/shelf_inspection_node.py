import rclpy
from rclpy.node import Node
from rclpy.service import Service
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
import time
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_simple_commander.robot_navigator import TaskResult

class ShelfInspectionNode(Node):
    def __init__(self):
        super().__init__('inspection_service')
        self.srv = self.create_service(Trigger, '/start_inspection', self.inspection_callback)
        self.get_logger().info('Service ready to start shelf inspection')

    def inspection_callback(self, request, response):
        self.get_logger().info('Starting shelf inspection...')
        
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()

        Shelf_A = [[5.78, -0.04], [2, -0.04]]
        Shelf_B = [[2, -2.14], [5.78, -2.14], [2, -2.14]]
        Shelf_C = [[2, -3.85], [5.78, -3.85], [2, -3.85]]
        Shelf_D = [[2, -5.75], [5.78, -5.75], [2, -5.75]]
        Shelf_E = [[2, -7.65], [5.78, -7.65], [2, -7.65]]
        Shelf_F = [[2, -9.8], [5.78, -9.8], [2, -9.8]]
        Shelf_G = [[2, -11.6], [5.78, -11.6],[2, -11.6]]

        start_pose = [2, -0.04]
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        def goTo(coordinates):
            pose.pose.position.x = float(coordinates[0])
            pose.pose.position.y = float(coordinates[1])
            navigator.goToPose(deepcopy(pose))

        goTo(start_pose)

        while not navigator.isTaskComplete():
            time.sleep(0.5)

        result = navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            response.success = False
            response.message = "Could not reach starting pose. Exiting"
            return response

        shelves = [Shelf_A, Shelf_B, Shelf_C, Shelf_D, Shelf_E, Shelf_F, Shelf_G]
        abc = ['A', 'B', 'C', 'D', 'E', 'F', 'G']

        for i, shelf in enumerate(shelves):
            for pt in shelf:
                goTo(pt)
                while not navigator.isTaskComplete():
                    time.sleep(0.5)
                result = navigator.getResult()
                if result != TaskResult.SUCCEEDED:
                    navigator.cancelTask()
                    goTo(start_pose)
                    while not navigator.isTaskComplete():
                        time.sleep(0.5)
                    response.success = False
                    response.message = f"Inspection failed at shelf {abc[i]}"
                    return response

            self.get_logger().info(f'Shelf {abc[i]} inspected.')
        
        response.success = True
        response.message = 'Inspection completed successfully with no anomalies.'
        return response  


def main(args=None):
    rclpy.init(args=args)
    inspection_service = ShelfInspectionNode()
    rclpy.spin(inspection_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()