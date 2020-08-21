# System Architecture

SEAN is built on two main components: the game engine ([Unity](https://unity.com/)) and the Robot Operating System ([ROS](https://www.ros.org/)).

## System Diagram

[//]: # (https://docs.google.com/drawings/d/12CaVboBZzEckTj1h_3wNi6LnWkprf9ADrQncQnYaM68/edit)
```eval_rst
.. thumbnail:: /images/see_system_diagram.png
```

Within Unity, we use the [ROS#](https://github.com/siemens/ros-sharp) library to interface with ROS. This ROS interface allows control of a simulated robot via the standard [ROS navigation stack](http://wiki.ros.org/navigation). We currently target [ROS Melodic](http://wiki.ros.org/melodic) with Python 3 compatibility.

## System Components

SEAN is built on two high level systems: [Unity](https://unity.com/) and [ROS](https://www.ros.org/).

Our code is divided between these two systems.

- The Unity project is available here: [https://github.com/yale-img/social_sim_unity](https://github.com/yale-img/social_sim_unity).

- The ROS package is available here: [https://github.com/yale-img/social_sim_ros](https://github.com/yale-img/social_sim_ros).

- We also provide a Dockerized ROS workspace and development scripts here: [https://github.com/yale-img/sim_ws](https://github.com/yale-img/sim_ws).

- This documentation is hosted here: [https://github.com/yale-img/social-sim-docs](https://github.com/yale-img/social-sim-docs) and pull requests are welcome.

### Unity System Components

Within Unity we use [ROS#](https://github.com/siemens/ros-sharp) to implement our APIs. This includes the following sub-components.

The basic system's communication graph is shown in the following figure:

```eval_rst
.. thumbnail:: /images/basic-graph.gif
```

#### Robot motor controller

- Subscribing to the `/wheel_right_joint_cmd` and `/left_wheel_joint_cmd` and updating the robot physics in Unity

*Note that currently the only controller that is implemented is the differential drive controller*

#### Global Localization

- Publishes the `/odom` frame for the robot to allow for mapping and navigation

  - To publish this frame to the TF tree in ROS, run:

    rosrun social_sim_ros odom_to_tf.py

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

There are currently two options for mapping: 1) using the [static map generated in Unity](editing.html#map) and published via the `map_server` or 2) gmapping.

1. Start static mapping with the `[environment name]_map_server.launch` file. For example:

    ```
    roslaunch social_sim_ros lab_map_server.launch
    ```

    - You'll also want to start the odom and map frame publisher with:

    ```
    rosrun social_sim_ros odom_to_tf.py _publish_map_frame:=true
    ```

1. gmapping can be run with the `gmapping_[robot_name].launch` launch file. For example:

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

Or with the `[robot_name]_ps3joy` tmuxinator config, run the jackal in teleop mode for example:

    cd ~/sim_ws/tmux/jackal_ps3joy
    tmuxinator

Or with the `[robot_name]_ps3joy.launch` launch file, run the jackal in teleop mode for example:

    TODO

Hold the top left trigger button to enable the remote control.
