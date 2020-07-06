## Ubuntu Installation

This guide will walk you through setting up the simulation environment including Unity and ROS.

Start with a [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) desktop installation.

### Unity

If you only need to test your algorithm or make changes in ROS, follow the [Unity Binary Setup](#binary-setup). If you also need to make edits to the simulator, environment or APIs, follow the [Unity Editor](#editor-setup). In either case, you'll need a working [ROS Setup](#ros-setup).

#### Binary Setup

Download the latest release of SEE from the release page: [https://gitlab.com/interactive-machines/simulation/social-sim/-/releases](https://gitlab.com/interactive-machines/simulation/social-sim/-/releases
).

Now, continue to setting up [ROS](#ros-setup).

#### Editor Setup

Following the [Unity Getting Started Guide](https://docs.unity3d.com/Manual/GettingStartedInstallingHub.html) to download and install the [UnityHub.AppImage](https://unity3d.com/get-unity/download).

- Download the AppImage here: [https://unity3d.com/get-unity/download](https://unity3d.com/get-unity/download)

- Make the file runnable:

```
chmod +x UnityHub.AppImage
```

- Run the file:

```
./UnityHub.AppImage
```

- Login to UnityHub. If you do not have an account create a Unity account now.

Note: signing up for a student account, though not required, will give you access to some free assets: [https://assetstore.unity.com/browse/student-plan-pack](https://assetstore.unity.com/browse/student-plan-pack)

- Add the Unity version 2019.4 (LTS). You can find it [here](https://unity3d.com/unity/qa/lts-releases?_ga=2.203078097.1539413933.1593667443-691579140.1593667443).


Now, continue to setting up the [Unity Project](#unity-project).

### Unity Project

- Clone the unity project, the location of this project is not important, but we'll clone it to the home directory:

```
git clone https://gitlab.com/interactive-machines/simulation/social-sim ~/social-sim
```

In Unity Hub, add the project you just cloned by clicking the "Add" button.

Run the project by clicking on the project name: "SocialNavSim"

*Note that if you are unable to add the project, create a new project by clicking the "New" button, restart Unity Hub, and then try adding the project again*

### ROS Setup

You can follow the [ROS Melodic Setup Guide](http://wiki.ros.org/melodic/Installation/Ubuntu) or use the [Docker Compose](#docker-setup) configuration below.

If you're not using Docker, [setup your workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) as normal and from within your workspace, add the git repository:

```
git clone https://github.com/yale-img/social_sim_ros src/social_sim_ros
```

Then build with `cakin_make`

#### Docker Setup

Instead of creating and installing a workspace from scratch, you can use our [Docker Compose](https://docs.docker.com/compose/) configuration to create a set of virtual machines in which the simulation platform can be run.

An advantage of this is that we also provide tools like a [Tmuxinator](https://github.com/tmuxinator/tmuxinator) configuration to easily start the necessary ROS components.

The Docker configuration is only tested on an Ubuntu 18.04 host machine with an CUDA compatible NVIDIA graphics card.

First, install the dependencies:

- [Docker](https://docs.docker.com/engine/install/ubuntu/)
- [NVIDIA Docker](https://github.com/NVIDIA/nvidia-docker)
- [NVIDIA Container Runtime](https://github.com/nvidia/nvidia-container-runtime)
- [Docker Compose](https://docs.docker.com/compose/install/).
- [Yarn](https://classic.yarnpkg.com/en/docs/install/#debian-stable).

To use the Docker configuration:

- Clone the workspace into your home folder and `cd` into the workspace:

```
git clone https://github.com/yale-img/sim_ws.git ~/sim_ws
cd ~/sim_ws
```

- Clone the dependencies into the project `src` folder

```
git clone https://github.com/yale-img/social_sim_ros src/social_sim_ros
```

- Build the Docker containers

```
yarn build
```

- Start the containers with

```
yarn start
```

- Enter a shell in the Docker Virtual Machine in your ROS workspace

```
yarn shell
```

- Build the workspace with

```
catkin_make
```


