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

  - To publish this frame to the TF tree in ROS, run:

    rosrun social_sim_ros odom_to_tf.py

Note that the `/odom` frame is relative to the Unity origin coordinate frame. When the robot is far from the Unity origin, this transform can be quite large, which may pose a problem for some mapping algorithms

#### Sensors

Sensors can be found within each robot in the Robots prefab.

The relevant ROS connectors can be found in each top level robot object.

##### RGB Camera

The `camera_rgb_frame` object contains the Unity camera object that publishes images from the robot's perspective to ROS. This object can be found in `Robots -> jackal -> base_link -> camera_rgb_frame`. Physical properties of the camera can be adjust via the `Camera` object.

##### Laser Scanner

The `laser_sensor` object is in the `Robots -> jackal -> base_link -> base_scan -> laser_sensor`. Laser scanner properties can be changed via the `Laser Scan Reader` object and the visualization (red lines) can be enabled or disabled via the `Laser Scan Visualizer Lines` property.

### ROS System Components

#### Robot Differential Drive Controller

The `/differential_drive_sim_controller` accepts commands on the topic `/mobile_base_controller/cmd_vel` and publishes them to the `/wheel_right_joint_cmd` and `/left_wheel_joint_cmd` topics for consumption in Unity.

To start the node, run:

    roslaunch social_sim_ros differential_drive_jackal.launch

#### Robot Description

Be sure to publish the correct robot description. For the jackal, this can be done with:

    roslaunch social_sim_ros jackal_description.launch

#### Mapping

Run:

    roslaunch social_sim_ros gmapping_jackal.launch

#### Default Navigation Stack

The default navigation stack can be started via:

    roslaunch social_sim_ros jackal_move_base.launch

#### RVIZ

Default visualization can be started with:

    rosrun rviz rviz -d $(rospack find social_sim_ros)/config/jackal_move.rviz

The robot can then be controlled via the `mobile_base_controller/cmd_vel` topic, which is published to via the trial runner or choosing a navigation point in RVIZ.

### Trial Runner

Trials, a series of navigation tasks over which evaluation data is collected, can be run as follows:

    rosrun social_sim_ros trial_runner.py _trial_name:=T0

Where `_trial_name:=[trial name]`, the results of the trials are written to `experiments/[trial name]`.

### Data Collection

#### PS3 Joystick Robot Control

Teleop via PS3 Joystick can be started via:

    roslaunch social_sim_teleop ps3_teleop.launch
