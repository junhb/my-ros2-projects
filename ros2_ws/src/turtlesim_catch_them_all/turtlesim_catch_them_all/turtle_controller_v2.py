#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
import math
from my_robot_interfaces.srv import CatchTurtle
from functools import partial

class TurtleControllerNode(Node):
    """

    A ROS2 node that controls turtle1 (aka master turtle) to chase and catch spawned turtles.
    """

    def __init__(self):
        super().__init__("turtle_controller")

        # Initialize variables to store turtle1's pose and target turtle's information
        self.turtle1_pose = None 
        self.target_turtle = None 

        # Declare parameter
        self.declare_parameter("catch_closest_turtle_first", True)

        self.catch_closest_turtle_first_ = self.get_parameter(
            "catch_closest_turtle_first").value

        ''' Subscriber '''
        # Subscribe to turtle1's pose to track its position
        self.pose_sub_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10)
        # self.get_logger().info(
        #     "Subscriber to /turtle1/pose has been started.")
        
        # Subscribe to the list of alive turtles to identify targets
        self.alive_sub_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10
        )
        
        ''' Publisher '''
        # Publisher for controlling turtle1's velocity
        self.publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel",10)
        
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        # self.get_logger().info(
        #     "Publisher to /turtle1/cmd_vel has been started.")
        
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
        # CORRECTION MADE FROM THE ANSWER
        # Selecting Target Turtle
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = 0

                # Find the closest turtle based on Euclidean distance
                for turtle in msg.turtles:
                    distance = math.sqrt((self.turtle1_pose.x - turtle.x_cord) ** 2 + (self.turtle1_pose.y - turtle.y_cord) ** 2)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.target_turtle = closest_turtle
            else:
                self.target_turtle = msg.turtles[0]

    def control_loop(self):
        if self.turtle1_pose is None or self.target_turtle is None:
            return  # Exit if pose or target is not set
        
        # Extract coordinates 
        x1, y1 = self.turtle1_pose.x, self.turtle1_pose.y
        x2, y2 = self.target_turtle.x_cord, self.target_turtle.y_cord

        # Compute distance between turtle1 and target
        distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

        # Create Twist message
        msg = Twist()

        # If within catching range, call service to catch the turtle
        if distance > 0.5:
            # position (2*distance and 6*diff are supposedly experimental?)
            msg.linear.x = 2*distance

            # orientation (compute angle to target)
            goal_theta = math.atan2(y2-y1, x2-x1)
            diff = goal_theta - self.turtle1_pose.theta

            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            msg.angular.z = 6*diff 

        else:
            # target reached 
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            
            self.catch_turtle(self.target_turtle.name)  # Request turtle catch
            self.target_turtle = None  # Reset target after catching
            

        # Publish movement command
        self.publisher_.publish(msg)

    def catch_turtle(self, turtle_name):
        """
        Sends a request to the 'catch_turtle' service to remove the target turtle.
        """
        while not self.catch_.wait_for_service(1.0):
            self.get_logger.warn("Wating for catch turtle service...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        # Call the service asynchronously
        future = self.catch_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch, turtle_name=turtle_name))

    def callback_call_catch(self, future, turtle_name):
        """
        Callback function to log success when the turtle is caught.
        """
        # Response is not defined for my code
        # Reason to define book: client side doesn't necessarily know 

        self.get_logger().info("Turtle " + turtle_name + " caught successfully!")

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