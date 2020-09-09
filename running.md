# Running the Simulator

To run the simulator, the basic steps are: start a roscore and ros bridge, start Unity in play mode and then bring up the rest of the ROS robot communication robot control nodes.

 - Start all the ROS with the command `roslaunch social_sim_ros jackal_demo.launch`.

*Note in development, it may be useful to run the necessary commands in their own shell windows, in which case the `tmuxinator` configurations in the [sim_ws](https://github.com/yale-sean/sim_ws/tree/master/tmux) project may be used.*

 - Open the Unity editor, open the `Assets/Scenes/IndoorLabScene.unity` scene file.

*Alternatively, if you have the Unity binary, running the binary replaces the editor play button*

```eval_rst
.. thumbnail:: /images/running_lab_scene.png
```

 - Press the play button.

```eval_rst
.. thumbnail:: /images/running_play.png
```

 - You should see a map in rviz.

```eval_rst
.. thumbnail:: /images/running_map.png
```

*If you don't see a map, first check the Unity editor console for the "Connect to RosBridge" message. If instead, you see a "Failed to connect to RosBridge" message, then press the play button to stop the simulation and then press play again to restart it.

  - Set a goal position using the `2D Nav Goal` button.

```eval_rst
.. thumbnail:: /images/running_nav_goal.png
```

The robot should move to the goal position!

You're now ready to [run a trial](trials).
