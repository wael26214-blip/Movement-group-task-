import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from my_robot_interfaces.action import MoveX
from my_robot_interfaces.srv import Error


# Define and initialize the MoveX Action Server node class
class MoveXActionServer(Node):

    # Constructor method to initialize node, ROS 2 communication interfaces, and state variables
    def __init__(self):
        # Initialize the node with the name 'move_x_action_server'
        super().__init__('move_x_action_server')

        # Create the Action Server for handling linear movement goals
        self._action_server = ActionServer(
            self,
            MoveX,
            'move_x',
            self.execute_callback
        )

        # Create Publisher to send velocity commands to the robot
        self.publisher_ = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )

        # Create Subscriber to continuously monitor robot odometry
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Variable to track the robot's current position on the X axis
        self.current_x = 0.0
        
        # Create Service Server to handle manual stop requests
        self.srv = self.create_service(
            Error,
            'stop_robot',
            self.stop_robot_callback
        )
        
        # Log node startup and readiness status
        self.get_logger().info('MoveX Action Server & Stop Robot Service initialized.')

    # Helper function to stop the robot motion immediately and reset position tracking
    def execute_stop(self):
        # Prepare a zero-velocity Twist message
        twist = Twist()
        twist.linear.x = 0.0
        
        # Publish zero velocity to stop the robot
        self.publisher_.publish(twist)
        
        # Reset the current X coordinate tracker
        self.current_x = 0.0

    # Service callback function to process incoming emergency stop requests
    def stop_robot_callback(self, request, response):
        # Check if the incoming request flag is set to True
        if request.stop:
            # Stop the robot and log status message
            self.execute_stop()
            self.get_logger().info('Robot stopped and distance counter reset!')
            
            # Prepare positive service response
            response.success = True
            response.message = 'Robot stopped successfully and counter reset.'
        # If the stop flag is False, take no action
        else:
            self.get_logger().info('Received stop request as False, doing nothing.')
            
            # Prepare negative service response
            response.success = False
            response.message = 'Stop request was False, no action taken.'

        return response

    # Odometry callback function to continuously update current X position
    def odom_callback(self, msg):
        # Store current X position from incoming odometry message
        self.current_x = msg.pose.pose.position.x

    # Main action server callback function to execute linear movement towards target distance
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Moving forward...')
        
        # Extract target distance from goal request
        target_distance = goal_handle.request.target_distance    
        
        # Use requested speed or set default speed if zero
        speed = goal_handle.request.speed if goal_handle.request.speed != 0.0 else 0.2
        
        # Record starting position
        start_x = self.current_x
        
        # Initialize feedback message container
        feedback_msg = MoveX.Feedback()

        # Prepare movement velocity message
        twist = Twist()
        twist.linear.x = speed

        # Control loop: Keep moving while node is active and target distance is not reached
        while rclpy.ok() and (abs(self.current_x - start_x) < target_distance):
            # Calculate total distance traveled so far
            distance_traveled = abs(self.current_x - start_x)

            # Publish current feedback to client
            feedback_msg.current_distance_traveled = distance_traveled
            goal_handle.publish_feedback(feedback_msg)

            # Check if action cancellation was requested by client
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled.')
                
                # Stop robot movement
                self.execute_stop()
                
                # Prepare and return canceled result
                result = MoveX.Result()
                result.success = False
                result.final_distance = distance_traveled
                return result

            # Publish movement velocity command to robot
            self.publisher_.publish(twist)
            
            # Short sleep delay to control publication loop rate
            time.sleep(0.05)

        # Stop robot after reaching target distance
        self.execute_stop()
        
        # Mark goal execution as successful
        goal_handle.succeed()   

        # Prepare and return final result
        result = MoveX.Result()
        result.success = True
        result.final_distance = abs(self.current_x - start_x)
        self.get_logger().info(f'Target distance reached: {result.final_distance:.2f}m')
        return result


# Main function to initialize ROS 2 communication, create node, and spin execution
def main():
    # Initialize ROS 2 communication
    rclpy.init()
    
    # Instantiate the node
    move_x_action_server = MoveXActionServer()
    
    # Keep node alive to process callbacks and requests
    rclpy.spin(move_x_action_server)
    
    # Destroy node explicitly when spinning stops
    move_x_action_server.destroy_node()
    
    # Shutdown ROS 2 communication
    rclpy.shutdown()


# Entry point execution guard
if __name__ == '__main__':
    main()
