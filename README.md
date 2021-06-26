# First assignment _Research Track_ course - second module - ros2 branch

- Index:
  - [General Instructions](#general-instructions)
  - [Launch the node](#launch-the-node)
  - [Additional docuementation](#additional-documentation)
## General Instructions

In the branch `ros2`, the cpp nodes (Robot FSM and position server) has been written for ROS2, as **components**, or composable nodes. The file **mapping_rules.yaml** will allow, using the **ros1_bridge** package, to create a link with the respective ROS nodes and with the simulation in Gazebo. **The go_to_point** behaviour is still implemented as a service.

Also:

1. I've written a _**script**_ to start the container manager and all the nodes as components.
2. a   _**launch file**_  to launch all required nodes and the simulation has also been implemented

## Launch the node 

To launch the nodes as components in a container, please run:
```
ros2 launch rt2_assignment1 composable_launch.py
```
To instead launch them as nodes and to launch Gazebo simulation, please run:
```
ros2 launch rt2_assignment1 ros2.launch
```
## Additional documentation

can be found opening the index.html file in the *docs* folder
