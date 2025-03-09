#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
import random
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from functools import partial
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawnerNode(Node):
    """
    ROS2 Node that spawns turtles at random positions, 
    keeps track of their status, and allows catching (removal).
    """
    
    def __init__(self):
        super().__init__("turtle_spawner")

        # Create service clients for spawning and killing turtles
        self.spawn_ = self.create_client(Spawn, "spawn")
        self.kill_ = self.create_client(Kill, "kill")
        
        # Store active turtles
        self.turtles_ = []
        
        ''' Publisher: Publishes the list of currently alive turtles '''
        self.alive_turtle_pub_ = self.create_publisher(
            TurtleArray, "alive_turtles", 10)
        
        ''' Service: Handles requests to "catch" (remove) a turtle '''
        self.server = self.create_service(
            CatchTurtle, "catch_turtle", self.callback_catch_turtle
        )

        # Wait for the spawn and kill services to become available
        # Note: This will not likely happen
        while not self.spawn_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim server...")
        while not self.kill_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim server...")    

        self.get_logger().info("Turtle spawner has started.")

        ''' Timer: Spawns a new turtle at random positions every x second '''
        self._timer_ = self.create_timer(1.0, self.call_spawn)

        ''' Timer: Publishes the list of alive turtles every second '''
        self.alive_timer_ = self.create_timer(1, self.publish_alive_turtles)
        

    def call_spawn(self):
        """

        Sends a request to spawn a new turtle at a random position.
        """

        request = Spawn.Request()
        
        # Generate random coordinates within the 11x11 simulation space
        request.x = random.uniform(0, 11)
        request.y = random.uniform(0, 11)
        
         # Call spawn service asynchronously and pass extra arguments via partial
        future = self.spawn_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn, request=request))

    def callback_call_spawn(self, future, request: Spawn.Request):
        """
        Callback function for handling the spawn service response.
        Adds the new turtle to the list of active turtles.
        """

        response: Spawn.Response = future.result() # "Spawn.Response" is optional to write
        self.get_logger().info("Turtle " + str(response.name) + " has been spawned.")
        
        # Store new turtle's details
        turtle = Turtle()
        turtle.name = response.name
        turtle.x_cord = request.x
        turtle.y_cord = request.y

        self.turtles_.append(turtle) # Specify the turtle to remove

    def call_kill(self, turtleName):
        """
        Sends a request to remove a turtle by name.
        """

        request = Kill.Request()
        request.name = turtleName # Specify the turtle to remove
        
        # Call kill service asynchronously
        future = self.kill_.call_async(request)

        self.get_logger().info("Turtle: " + request.name + " has been killed.")

    def publish_alive_turtles(self):
        """
        Publishes the current list of alive turtles.
        """

        msg = TurtleArray()
        
        # Ensure the list only contains valid turtles
        self.turtles_ = [turtle for turtle in self.turtles_ if turtle is not None]

        msg.turtles = self.turtles_
        self.alive_turtle_pub_.publish(msg)

        self.get_logger().info("Publishing list of alive turtles")

    def callback_catch_turtle(self, request: CatchTurtle.Request, 
                              response: CatchTurtle.Response):
        """
        Handles requests to catch (remove) a turtle.
        Removes the turtle from the active list and calls the kill service.
        """
        self.get_logger().info("Turtle " + str(request.name) + 
                               "is catched.")
        
        # Remove the turtle from the list
        self.turtles_ = [turtle for turtle in self.turtles_ if turtle.name != request.name]
    
        # Call the kill service to remove the turtle from simulation
        self.call_kill(request.name)
        
        return response


def main(args=None):
    """
    Initializes the ROS2 node and starts the event loop.
    """

    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
