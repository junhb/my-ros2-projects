# Turtlesim Catch Them All - ROS2 Project

This project demonstrates how to create and control turtles in the `turtlesim` environment using ROS2. The goal of the project is to spawn, control, and "catch" turtles using services, publishers, and subscribers.

## My Approach

To begin with, I started the project in a "hardcore mode," meaning I directly started on my own without following any instructions step by step (let’s call this the hardcore mode). Below are the steps I took during this initial approach, resulting in the creation of the following files:
- `turtle_controller.py`
- `turtle_spawner.py`

Later, after receiving feedback and instructor notes, I made revisions and improvements. The final versions of the files are:
- `turtle_controller_v2.py`
- `turtle_spawner_v2.py`

## Steps Taken to Create This Project

### 1. **Create Package: `turtlesim_catch_them_all`**

The first step was to create a ROS2 package for managing the turtlesim simulation. The package includes nodes for spawning turtles, controlling them, and interacting with them.

### 2. **Create `turtle_spawner` Node**
This node handles spawning and killing turtles in the simulation.

#### Services:
- **/spawn**: Spawns a new turtle at random coordinates between `0.0` and `11.0` for both `x` and `y`. These coordinates are advertised by the `turtlesim` node.
- **/kill**: Removes a turtle from the screen.
- **/catch_turtle**: Handles the action of "catching" a turtle by calling the `/kill` service.

#### Publisher:
- **/alive_turtles**: Publishes the list of turtles that are alive. This is stored as an array in a custom message.

#### Custom Messages:
- **Turtle.msg**: Contains a list of turtle names.
- **TurtleArray.msg**: Contains a list of turtle coordinates.

### 3. **Create `turtle_controller` Node**
The `turtle_controller` node controls the movement of the turtles based on their positions.

#### Main Features:
- A high-rate control loop (timer).
- Subscribes to `/turtle1/pose` for the current position of the turtle.
- Publishes to `/turtle1/cmd_vel` to control the turtle's movement.
- Subscribes to the `/turtle1/cmd_vel` topic to control the turtle's movement dynamically.
- Selects a target turtle from the list of alive turtles:
  1. First turtle in the array.
  2. Closest turtle to the controller.

### 4. **Create Launch File: `turtlesim_catch_them_all.launch.xml`**
This launch file brings everything together by activating the following nodes:
- `turtlesim_node` (runs the turtlesim environment).
- `turtle_controller` (controls the turtles).
- `turtle_spawner` (spawns and kills turtles).

## Commands & Tools

Here are some useful commands and tools you might use during development:

- **List Services**: `ros2 service list`
- **Show Service Type**: `ros2 service type <service_name>`
- **Show Request and Response**: `ros2 interface show <service_type>`
- **Call Service**: `ros2 service call <service_name> <arguments>`

## How to Start the Project

1. **Build the Workspace**:
   In the `ros2_ws` directory, run the following command to build the workspace:
   ```bash
   colcon build
   source install/setup.bash
   ros2 launch my_robot_bringup turtlesim_catch_them_all_v2.launch.xml

## Results

![Spawn and Catch Simulation for the Final Project](turtlesim_project.png)

The image visually demonstrates the functionality of the "Spawn and Catch Simulation" in the final project. It shows multiple turtles being spawned at random positions within the simulation environment. A robot (turtle_controller) is actively chasing these turtles to "catch" them, with the path and movement of the robot being visualized. The image emphasizes key elements like the turtle spawning service, control loop for catching the turtles, and the interaction between different ROS2 nodes, such as the turtle_spawner and turtle_controller nodes. The layout helps to illustrate the dynamics of spawning, controlling, and catching in the simulation environment.

## Project Notes

Here are some useful commands for working with ROS2 services:

- **ros2 service list**: Lists all available services.
- **ros2 service type <service_name>**: Shows the service type for a specific service.
- **ros2 interface show <interface_name>**: Displays the details of a ROS2 interface.
- **ros2 service call <service_name> <arguments>**: Calls a ROS2 service with the specified arguments.

# ROS2 Course Notes

## Goal (for taking the course)
- Explore ROS and recap the Linux environment.

---

## Section 1: Introduction

### ROS 2 Lecture

#### Installing Ubuntu on Oracle VM
- Installed VSCode
- Installed various extensions like CMake
- Installed ROS2

#### Notes:
- `sudo apt update`
- `sudo apt upgrade`
  - Run often, as these change monthly.

---

## Section 2: Install ROS2 and Setup Your Environment

### 9. Set up Your Environment for ROS 2
- You have to source it in every terminal:
  - `source /opt/ros/jazzy/setup.bash`

### 10. Launch a ROS 2 Program!
- `ros2 run demo_nodes_cpp talker`
- `ros2 run demo_nodes_cpp listener`

### 13. Create a ROS 2 Workspace
- `colcon build`

### 14. Create a Python Package
- `ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy`
- `ros2 bag record /number_count`

### 15. Create a C++ Package
- `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp`
- `code .` (to open VSCode)
- `rm -r build/ install/ log`
- `colcon build --packages-select my_cpp_pkg`

### 16. What is a ROS2 Node?
- ROS2 nodes are subprograms responsible for one task, communicating through topics, services, and parameters.
- Benefits:
  - Reduce code complexity
  - Fault tolerance
  - Language agnostic (Python, C++)

### 17. Write a Python Node - Minimal Code
- `touch my_first_node.py`
- `chmod +x my_first_node.py` to run as executable
- `rclpy.spin(node)` keeps the node alive until you press `Ctrl+C`
- "py_node = my_py_pkg.my_first_node:main" inside 'console_script'
- `colcon build --packages-select my_py_pkg` in `ros2_ws`
- `source ~/.bashrc`

### 18. Write a Python Node - With OOP
- Steps: Build → Source → Run

### 20. Write a C++ Node - With OOP
- Successfully created a C++ node with object-oriented programming.

---

## Section 4: Introduction to ROS 2 Tools

### 25. Introspect Your Nodes with `ros2` CLI
- After installing a node, executables are in the install folder of your ROS2 workspace.
- Learned how to start with command-line tool `ros2 run`.
- `ros2 run package-name executable-name`
- `ros2 node list`

### 27. Colcon
- Must build inside the ROS2 workspace:
  - `colcon build` builds all packages.
  - `colcon build --packages-select my_py_pkg --symlink-install`

### 28. `rqt` and `rqt_graph`
- Used for introspecting ROS2.
- Provides a global graphical overview.

### 29. Discover Turtlesim
- Goal: Reproduce `rqt_graph` for turtlesim.
  - Renamed using `--ros-args` for turtlesim as Donatello.
  - Renamed `my_py_pkg` to `custom_py` and `my_cpp_pkg` to `...works`.

---

## Section 5: ROS2 Topics - Make Your Nodes

### 34. What is a ROS 2 Topic?
- A publisher, topic, and subscriber communication system.

### 35. Write a Python Publisher
- Created a new file: `touch robot_news_station.py`
- Made it executable: `chmod +x robot_news_station.py`
- `ros2 interface show example_interfaces/msg/String`
- `ros2 topic echo /robot_news`

### 37. Write a C++ Publisher
- Modified `CMakeLists.txt` and `package.xml`.
  - Included the interface.
  - Created the publisher.

### 39. How to Introspect a Topic
- `ros2 topic hz /robot_news`
- `ros2 topic bw /robot_news`
- `ros2 topic pub -r 5 /robot_news example_interfaces/msg/String "{data: 'Hello from the terminal'}"`

### 40. Remap a Topic at Runtime
- `ros2 run my_py_pkg robot_news_station --ros-args -r __node:=my_station -r robot_news:=abc`

### 41. Monitor Topics with `rqt` and `rqt_graph`
- Can't have 2 nodes with the same name.

### 42. Experiment on Topics with Turtlesim
- `ros2 run turtlesim turtlesim_node`
- `ros2 topic pub -r 2 /turtle1/cmd_vel geometry_msgs/msg/Twist`

### 44. Activity 02 - Solution [1/2]
- Created a publisher in `int64` in Python and subscriber in `int64` in C++.
- Used `ros2 bag record -o test /number_count`.

---

## Section 6: ROS 2 Services

### 49. What is a ROS 2 Service?
- A service is a request/response communication method.

### 50. Write a Python Service Server
- `ros2 interface show example_interfaces/srv/AddTwoInts`
- Example service call: 
  - `ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 3, b: 7}"`

### 56. Introspect Services with the `ros2` CLI
- `ros2 service list`
- `ros2 service type /add_two_ints`

### 57. Remap a Service at Runtime
- `ros2 run my_py_pkg add_two_ints_server --ros-args -r add_two_ints:=abc`

### 58. Experiment on Services with Turtlesim
- `ros2 service call /clear std_srvs/srv/Empty`
- `ros2 service call /kill turtlesim/srv/Kill "{name: 'my_turtle'}"`

---

## Activity 3: Naming Convention for `.hpp` Files in ROS 2

- Service/message names in CamelCase (e.g., SetBool) → Convert to snake_case (e.g., `set_bool.hpp`).

### Activity 4: Battery Node
- Created a Battery Node to simulate battery states (full/empty).
- Communicates with the LED Panel Node via services to power on/off LEDs based on battery status.

---

## Activity 5: YAML Parameter Files

- Created a YAML file: `numbeR_params.yaml`.
- Managed parameters with `ros2 param list` and `ros2 param set`.

---

## Future Work
- Explore ROS2 Level 2, Level 3, and SLAM courses.
- Investigate when exactly to build and source—currently done repeatedly for testing.

---

## Lessons Learned

### Mistakes and Corrections
- Initially had issues with keyword arguments (`turtleName=turtleName` vs `turtle name`).
- For partial function calls, need explicit keyword arguments.

### Tips:
- Test as you go when adding small functionalities.
- It's important to understand the roles of server-side code, especially in ROS2.

---
