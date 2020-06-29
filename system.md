# System Architecture

SEE is built on two main components: the game engine ([Unity](https://unity.com/)) and the Robot Operating System ([ROS](https://www.ros.org/)).

[//]: # (https://docs.google.com/drawings/d/12CaVboBZzEckTj1h_3wNi6LnWkprf9ADrQncQnYaM68/edit)
![System Diagram](/images/see_system_diagram.png)

Within Unity, we use the [ROS#](https://github.com/siemens/ros-sharp) library to interface with ROS. This ROS interface allows control of a simulated robot via the standard [ROS navigation stack](http://wiki.ros.org/navigation). We currently target [ROS Melodic](http://wiki.ros.org/melodic) with Python 3 compatibility.

