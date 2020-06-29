# System Architecture

SEE is built on two main components: the game engine ([Unity](https://unity.com/)) and the Robot Operating System ([ROS](https://www.ros.org/)).

## System Diagram

[//]: # (https://docs.google.com/drawings/d/12CaVboBZzEckTj1h_3wNi6LnWkprf9ADrQncQnYaM68/edit)
```eval_rst
.. thumbnail:: /images/see_system_diagram.png
```

Within Unity, we use the [ROS#](https://github.com/siemens/ros-sharp) library to interface with ROS. This ROS interface allows control of a simulated robot via the standard [ROS navigation stack](http://wiki.ros.org/navigation). We currently target [ROS Melodic](http://wiki.ros.org/melodic) with Python 3 compatibility.

## System Components

SEE is built on two high level systems: [Unity](https://unity.com/) and [ROS](https://www.ros.org/).

Our code is divided between these two systems.

- The Unity project is available here: [https://gitlab.com/interactive-machines/simulation/social-sim](https://gitlab.com/interactive-machines/simulation/social-sim).

- The ROS package is available here: [https://github.com/yale-img/social_sim_ros](https://github.com/yale-img/social_sim_ros).

- We also provide a Dockerized ROS workspace here: [https://github.com/yale-img/sim_ws](https://github.com/yale-img/sim_ws).

### Unity System Components

Within Unity we use [ROS#](https://github.com/siemens/ros-sharp) to implement our APIs. This includes the following sub-components.

The basic system's communication graph is shown in the following figure:

```eval_rst
.. thumbnail:: /images/basic-graph.gif
```


#### Robot motor controller

- Subscribing to the `/wheel_right_joint_cmd` and `/left_wheel_joint_cmd` and updating the robot physics in Unity

#### Global Localization

- Publishes the `/odom` frame for the robot to allow for mapping and navigation

Note that the `/odom` frame is relative to the Unity origin coordinate frame. When the robot is far from the Unity origin, this transform can be quite large, which may pose a problem for some mapping algorithms

####

### ROS System Components

#### Robot Differential Drive Controller

The `/differential_drive_sim_controller` accepts commands on the topic `/mobile_base_controller/cmd_vel` and publishes them to the `/wheel_right_joint_cmd` and `/left_wheel_joint_cmd` topics for consumption in Unity.

#### Global Localization
