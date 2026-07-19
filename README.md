# Horizon Mapless Navigation System

This repository contains a navigation system that responds to a depth image. For this a gazebo simulation is made
to generate mock depth camera feed. This mock data is then used by the decision node to make navigation choices

This repository has two parts

## perception
```
ros2 launch rover_navigation sim_launch.py
```
This gazebo simulation emits `/depth/image` at a frequent interval

## Navigation node
```
ros2 run rover_navigation navigation_node
```
This node makes simple navigation choices based on `/depth/image` feed
