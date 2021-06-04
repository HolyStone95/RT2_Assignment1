# First assignment _Research Track_ course - second module - actions branch

- Index:
  - [General Instructions](#general-instructions)
  - [Custom action: the GoalReaching action](#custom-action:-the-goalreaching-action)
  - [Launch the node](#launch-the-node)
## General Instructions

This branch contains the same package of the main branch, but with the **go_to_point** node modelled as a ROS action server, instead of a “simple” server.
Given that, **the robot FSM node** should now implement mechanisms for possibly cancelling the goal, when the related user command is received.

##Custom action: the GoalReaching action

The action used by the **go_to_point** node implemented as an action server:

float32 x
float32 y
float32 theta
---
bool ok
---
float32 updated_x
float32 updated_y
float32 updated_theta

where the first 3 variables represent the goal(target pose)
      the bool ok is the result, set to true once the goal is reached
      the last 3 variables represent the feedback about the actual robot pose
      
## Launch the node 

To launch the node, please run:
```
roslaunch rt2_assignment1 action_branch.launch
```

