# Particle_filter_mesh

## Overview

This package is a component of the human location estimation module. The nodes leverage data from IoT sensors and the relative position of the person as detected by cameras (using the ZED camera by Stereolabs) to estimate the person’s location, even when they are outside the camera's field of view.

The Particle_filter_mesh package has been tested under [ROS] Humble on Ubuntu 22.04

## Installation

### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- [Eigen] (linear algebra library)

### Building

To build from source

``` 
  mkdir -p ~/ros2_ws/src
  cd ~/ros2_ws/src
  git clone https://olaghattas/particle_filter_mesh.git
  cd ../
  rosdep install --from-paths . --ignore-src
  colcon build --symlink-install
  source install/setup.bash
``` 
## Running in Docker

Refer to this repo to run things in docker https://github.com/olaghattas/particle_filter_docker

## Usage
Run the main node with
``` 
  ros2 run  particle_filter_mesh particle_filter_node
```

## Config files
In order to represent the environment, I used .obj files created in Blender.

Config file cam_view.obj

* **cam_view.obj** The mesh of the areas the cameras can view in the environment

Config file collision_mesh.obj

* **collision.obj** The mesh of the obstacles and doors found in the environment 

Config file person.obj

* **person.obj** The mesh of different rooms in the environment

## Nodes

### particle_filter_node

Estimate the location of the person within the environment and publish it to the /tf topic

#### Subscribed Topics
The system subscribes to the following topics for each ZED camera and sensor in the environment:

* **`/pose_estimate`** ([zed_interfaces::msg::ObjectsStamped])
  Provides the relative position of a detected person within the frame of the corresponding ZED camera.
* **`/door_sensors`** ([std_msgs::msg::Bool])
 Indicates whether the associated door sensor has been triggered.
* **`/motion_sensors`** ([std_msgs::msg::Bool])
  Indicates whether the corresponding motion sensor has been activated.

#### Published Topics

* **`/particles`** visualization_msgs::msg::MarkerArray
  The particles location for visualization purposes.
  

## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[Eigen]: http://eigen.tuxfamily.org
