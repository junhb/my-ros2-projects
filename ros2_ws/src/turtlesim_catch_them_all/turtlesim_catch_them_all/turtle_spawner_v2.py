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
        self.declare_parameter("turtle_name_prefix", "turtle") # Not really needed for my code
        self.declare_parameter("spawn_frequency", 2.0).value 

        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
        self.spawn_frequency = self.get_parameter("spawn_frequency").value

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
        self.get_logger().info("Turtle spawner has started.")

        ''' Timer: Spawns a new turtle at random positions every x second '''
        self._timer_ = self.create_timer(1.0/self.spawn_frequency, self.call_spawn)

        # ''' Timer: Publishes the list of alive turtles every second '''
        # self.alive_timer_ = self.create_timer(1, self.publish_alive_turtles)
        

    def call_spawn(self):
        """

        Sends a request to spawn a new turtle at a random position.
        """
        while not self.spawn_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim server...")

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
        
        # CORRECTION MADE FROM THE ANSWER
        # Checking if the name is not empty 
        if response.name != "":
            self.get_logger().info("New alive turtle: " + str(response.name))
        
            # Store new turtle's details
            turtle = Turtle()
            turtle.name = response.name
            turtle.x_cord = request.x
            turtle.y_cord = request.y

            self.turtles_.append(turtle) # Specify the turtle to remove
            self.publish_alive_turtles() # Publish when the list is updated

    def call_kill(self, turtleName):
        """
        Sends a request to remove a turtle by name.
        """
        while not self.kill_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Turtlesim server...")    

        request = Kill.Request()
        request.name = turtleName # Specify the turtle to remove
        # Call kill service asynchronously
        future = self.kill_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_service, turtleName=turtleName)
        )

    def callback_call_kill_service(self, future, turtleName):    
        for (i, turtle) in enumerate(self.turtles_):
            if turtle.name == turtleName: 
                del self.turtles_[i]
                self.publish_alive_turtles()
                break

    def publish_alive_turtles(self):
        """
        Publishes the current list of alive turtles.
        """

        msg = TurtleArray()
        
        # Ensure the list only contains valid turtles
        # self.turtles_ = [turtle for turtle in self.turtles_ if turtle is not None]

        msg.turtles = self.turtles_
        self.alive_turtle_pub_.publish(msg)

    def callback_catch_turtle(self, request: CatchTurtle.Request, 
                              response: CatchTurtle.Response):
        """
        Calls the kill service.
        """
        self.get_logger().info("Turtle " + str(request.name) + 
                               "is catched.")
        
        # CORRECTION MADE FROM THE ANSWER 
        # Removing turtle from the list is done now from the kill callback
        # self.turtles_ = [turtle for turtle in self.turtles_ if turtle.name != request.name]
    
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
