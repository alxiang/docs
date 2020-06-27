# SEE: Social Evaluation Environment

The Social Evaluation Environment (SEE) is a high fidelity, extensible, and open source simulation platform designed for the fair evaluation of social navigation algorithms. It leverages advances in graphics and physics modeling to build high fidelity scenes. We include in the platform many scenes with social agents that navigate according to standard pedestrian models. Integration with the Robot Operating System (ROS) allows for compatibility with existing navigation software stacks.

To get started, got to: [Setup](docs/setup)

## System Architecture

SEE is built on two main components: the game engine ([Unity](https://unity.com/)) and the Robot Operating System ([ROS](https://www.ros.org/)).

[//]: # (https://docs.google.com/drawings/d/12CaVboBZzEckTj1h_3wNi6LnWkprf9ADrQncQnYaM68/edit)
![System Diagram](/images/see_system_diagram.png)

Within Unity, we use the [ROS#](https://github.com/siemens/ros-sharp) library to interface with ROS. This ROS interface allows control of a simulated robot via the standard [ROS navigation stack](http://wiki.ros.org/navigation). We currently target [ROS Melodic](http://wiki.ros.org/melodic) with Python 3 compatibility.

## Features

For systematic evaluation, we provide the [trial runner](docs/trials) which allows a user to run their navigation algorithms against a set of common tasks in both indoor and outdoor environments.

## Modifying the Environment

SEE is a flexible platform. We provide documentation for adding or editing [Environments](editing.html#Environments), [Robots](editing.html#Robots), [Sensors](editing.html#Sensors), and [Pedestrian Navigation Models](editing.html#Pedestrian Navigation Models)


