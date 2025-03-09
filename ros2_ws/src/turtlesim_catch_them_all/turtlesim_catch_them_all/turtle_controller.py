#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
import math
from my_robot_interfaces.srv import CatchTurtle

class TurtleControllerNode(Node):
    """

    A ROS2 node that controls turtle1 (aka master turtle) to chase and catch spawned turtles.
    """

    def __init__(self):
        super().__init__("turtle_controller")

        # Initialize variables to store turtle1's pose and target turtle's information
        self.turtle1_pose = None 
        self.target_turtle = None 

        ''' Subscriber '''
        # Subscribe to turtle1's pose to track its position
        self.pose_sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10)
        self.get_logger().info(
            "Subscriber to /turtle1/pose has been started.")
        
        # Subscribe to the list of alive turtles to identify targets
        self.alive_sub_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10
        )
        self.get_logger().info(
            "Subscriber to alive_turtles has been started.")
        
        ''' Publisher '''
        # Publisher for controlling turtle1's velocity
        self.publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel",10)
        self.timer_ = self.create_timer(0.5, self.publish_cmd_vel)
        self.get_logger().info(
            "Publisher to /turtle1/cmd_vel has been started.")
        
        ''' Client'''
        # Client to request catching a turtle
        self.catch_ = self.create_client(CatchTurtle, "catch_turtle")
    
    def callback_pose(self, msg: Pose):
        """
        Callback function to update turtle1's pose.
        """
        self.turtle1_pose = msg

    def callback_alive_turtles(self, msg: TurtleArray):
        """
        Callback function to update the target turtle.
        Selects the closest turtle from the list of alive turtles.
        """
        if not msg.turtles:
            return
        
        # Find the closest turtle based on Euclidean distance
        closest_turtle = min(
            msg.turtles, 
            key=lambda t: math.sqrt((self.turtle1_pose.x - t.x_cord) ** 2 + (self.turtle1_pose.y - t.y_cord) ** 2))
        self.target_turtle = closest_turtle

    def publish_cmd_vel(self):
        """
        Publishes velocity commands to make turtle1 move toward the target turtle.
        """
        if self.turtle1_pose is None or self.target_turtle is None:
            return  # Exit if pose or target is not set

        # Extract coordinates 
        x1, y1 = self.turtle1_pose.x, self.turtle1_pose.y
        x2, y2 = self.target_turtle.x_cord, self.target_turtle.y_cord

        # Compute distance between turtle1 and target
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # If within catching range, call service to catch the turtle
        if distance < 0.5:
            self.catch_turtle(self.target_turtle.name)  # Request turtle catch
            self.target_turtle = None  # Reset target after catching
            return  # Stop movement after catching

        # Compute angle to target 
        angle_to_target = math.atan2(y2 - y1, x2 - x1)
        angle_error = angle_to_target - self.turtle1_pose.theta

        # Normalize angle error to be within [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Create Twist message
        msg = Twist()

        # Rotate toward target if needed
        if abs(angle_error) > 0.1:
            msg.angular.z = 2.0 * angle_error  # Adjust angular velocity based on error
        else:
            msg.linear.x = 1.0  # Move forward when facing the target

        # Publish movement command
        self.publisher_.publish(msg)

    def catch_turtle(self, turtle_name):
        """
        Sends a request to the 'catch_turtle' service to remove the target turtle.
        """
        request = CatchTurtle.Request()
        request.name = turtle_name

        # Call the service asynchronously
        future = self.catch_.call_async(request)
        future.add_done_callback(self.catch_turtle_response)

    def catch_turtle_response(self, future):
        """
        Callback function to log success when the turtle is caught.
        """
        self.get_logger().info(f"Turtle caught successfully!")

def main(args=None):
    """
    Entry point for the ROS2 node.
    """
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()