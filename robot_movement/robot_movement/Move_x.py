import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import MoveX
from my_robot_interfaces.srv import Error


class MoveXActionServer(Node):

    def __init__(self):
        super().__init__('move_x_action_server')

        self._action_server = ActionServer(
            self,
            MoveX,
            'move_x',
            self.execute_callback
        )

        self.publisher_ = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.current_x = 0.0
        
        self.srv = self.create_service(
            Error,
            'stop_robot',
            self.stop_robot_callback
        )
        
        self.get_logger().info('MoveX Action Server & Stop Robot Service initialized.')

    def execute_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        self.publisher_.publish(twist)
        self.current_x = 0.0

    def stop_robot_callback(self, request, response):
        if request.stop:
            self.execute_stop()
            self.get_logger().info('Robot stopped and distance counter reset!')
            response.success = True
            response.message = 'Robot stopped successfully and counter reset.'
        else:
            self.get_logger().info('Received stop request as False, doing nothing.')
            response.success = False
            response.message = 'Stop request was False, no action taken.'

        return response

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Moving forward...')
        target_distance = goal_handle.request.target_distance    
        speed = goal_handle.request.speed if goal_handle.request.speed != 0.0 else 0.2
        start_x = self.current_x
        feedback_msg = MoveX.Feedback()

        twist = Twist()
        twist.linear.x = speed

        while rclpy.ok() and (abs(self.current_x - start_x) < target_distance):
            distance_traveled = abs(self.current_x - start_x)

            feedback_msg.current_distance_traveled = distance_traveled
            goal_handle.publish_feedback(feedback_msg)

            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                self.execute_stop()
                result = MoveX.Result()
                result.success = False
                result.final_distance = distance_traveled
                return result

            self.publisher_.publish(twist)
            time.sleep(0.05)

        self.execute_stop()
        goal_handle.succeed()   

        result = MoveX.Result()
        result.success = True
        result.final_distance = abs(self.current_x - start_x)
        self.get_logger().info(f'Target distance reached: {result.final_distance:.2f}m')
        return result


def main():
    rclpy.init()
    move_x_action_server = MoveXActionServer()
    rclpy.spin(move_x_action_server)
    move_x_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
