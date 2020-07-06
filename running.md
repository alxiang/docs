# Running the Simulator

If you have the Unity binary, run it now. If you have the Unity editor, press play.


Once you have built the workspace once, if you install [tmux](https://github.com/tmux/tmux) and [Tmuxinator](https://github.com/tmuxinator/tmuxinator), you can launch all the required ROS components with the following command (outside of the Docker VM shell, but in the ROS workspace directory):

```
cd ~/sim_ws
tmuxinator s
```

