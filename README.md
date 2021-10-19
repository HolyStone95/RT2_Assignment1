# First assignment _Research Track_ course - second module 

- Index:
  - [General Instructions](#general-instructions)
  - [More about the "_actions_" branch](#more-about-the-actions-branch)
  - [More about the "_ros2_" branch](#more-about-the-ros2-branch)
  - [More about the "_jupy_doxy_" branch](#more-about-the-jupy_doxy-branch)
  - [More about the "_jupy_sphinx_" branch](#more-about-the-jupy_sphinx-branch)
  - [Launch the node](#launch-the-node-in-this-branch)
  - [Additional documentation](#additional-documentation)
## General Instructions

The [Professor's package](https://github.com/CarmineD8/rt2_assignment1) has been modified by creating this repository, containing the given package and two additional branches:

1. `actions` 
2. `ros2`

Finally, in the main branch (or in one of the two added branches), I added a Vrep scene containing the robot interacting with the simulation. (You can choose if using ROS or ROS2 api)

## More about the "_actions_" branch

This branch contains the same package of the main branch, but with the **go_to_point** node modelled as a ROS action server, instead of a “simple” server.
Given that, **the robot FSM node** should now implement mechanisms for possibly cancelling the goal, when the related user command is received

## More about the "_ros2_" branch

In the branch `ros2`, the cpp nodes (Robot FSM and position server) has been written for ROS2, as **components**, or composable nodes. By using the **ros1_bridge**, they can be linked with the respective ROS nodes and with the simulation in Gazebo. **The go_to_point** behaviour is still implemented as a service.
Also:

1. I've written a  launch file to start the container manager and all the nodes as components.
2. a script to launch all required nodes and the simulation has also been implemented

## More about the "_jupy_doxy_" branch
In this branch I've slightly modified the code present in the branch actions in order to work in parallel with a Jupyter notebook. The code is then commented using Doxygen.

## More about the "_jupy_sphinx_" branch
In this branch I've slightly modified the code present in the branch actions in order to work in parallel with a Jupyter notebook. The code is then commented using Sphinx.

## Launch the node in this branch

To launch the node, please run:
```
roslaunch rt2_assignment1 rt2_assignment1.launch
```
## Additional documentation

Can be found in *docs* folderv opening index.html file in the browser
