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
        super().__init__('patrolling_service')
        self.srv = self.create_service(Trigger, '/start_patrolling', self.inspection_callback)
        self.get_logger().info('Service ready to start security patrol')

    def inspection_callback(self, request, response):
        self.get_logger().info('Starting security patrol...')
        
        navigator = BasicNavigator()
        navigator.waitUntilNav2Active()

        security_route = [
            [0.5845286846160889, -11.42951774597168],
            [1.3253132104873657, 4.869967460632324],
            [-3.4749693870544434, 4.810696601867676],
            [-3.4749696254730225, -11.31097412109375]
        ]

        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0

        while not navigator.isTaskComplete():
            time.sleep(0.5)

        route_poses = []
        
        
        for pt in security_route:
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            route_poses.append(deepcopy(pose))

        navigator.goThroughPoses(route_poses)

        while not navigator.isTaskComplete():
            time.sleep(0.5)
        result = navigator.getResult()
        if result != TaskResult.SUCCEEDED:
            navigator.cancelTask()
            
            while not navigator.isTaskComplete():
                time.sleep(0.5)
            response.success = False
                    
            return response

        
        response.success = True
        response.message = 'Route completed with no anomalities.'
        return response  


def main(args=None):
    rclpy.init(args=args)
    inspection_service = ShelfInspectionNode()
    rclpy.spin(inspection_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

