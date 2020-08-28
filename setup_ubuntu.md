## Ubuntu Installation

This guide will walk you through setting up the simulation environment including Unity and ROS.

Start with a [Ubuntu 18.04](https://releases.ubuntu.com/18.04/) desktop installation.

### Unity

If you only need to test your algorithm or make changes in ROS, follow the [Unity Binary Setup](#binary-setup). If you also need to make edits to the simulator, environment or APIs, follow the [Unity Editor](#editor-setup). In either case, you'll need a working [ROS Setup](#ros-setup).

#### Binary Setup

Download the latest release of SEAN from the release page: [https://github.com/yale-img/social_sim_unity/releases](https://github.com/yale-img/social_sim_unity/releases).

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

- Close UnityHub by hitting `CTRL-C`.

- Install Unity Version `2019.4.0f1` by running the `UnityHub.AppImage` binary that was downloaded above with the version argument: `unityhub://2019.4.0f1/0af376155913`, like so:

```
./UnityHub.AppImage unityhub://2019.4.0f1/0af376155913
```

Then accept the defaults to install Unity, optionally installing any target build environments you may want to build for in the future.

Now, continue to setting up the [Unity Project](#unity-project).

### Unity Project

- Clone the unity project, the location of this project is not important, but we'll clone it to the home directory:

```
git clone https://github.com/yale-img/social_sim_unity.git ~/social_sim_unity
```

In Unity Hub, add the project you just cloned by clicking the "Add" button.

Run the project by clicking on the project name: `social_sim_unity`

*Note that if you are unable to add the project, create a new project by clicking the "New" button, restart Unity Hub, and then try adding the project again*

### ROS Setup

You can follow the [ROS Melodic Setup Guide](http://wiki.ros.org/melodic/Installation/Ubuntu) or use the [Docker Compose](#docker-setup) configuration below.

If you're not using Docker, [setup your workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) in `~/sim_ws` as normal. Then, from within your workspace `src` folder add the git repository:

```
git clone https://github.com/yale-img/social_sim_ros src/social_sim_ros
```

Then go to the root of your workspace, install dependencies and build your workspace:

```bash
$ rosdep install -y -r --ignore-src --rosdistro=melodic --from-paths src

# build workspace
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

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

Also, don't forget to add your user to the docker group, as described in the Docker documentation post-install instructions for linux: https://docs.docker.com/engine/install/linux-postinstall/.

To use the Docker configuration:

- Clone the workspace into your home folder and `cd` into the workspace:

```
git clone https://github.com/yale-img/sim_ws.git ~/sim_ws
cd ~/sim_ws
```

- Clone the dependencies into the project `src` folder

```
mkdir -p src
cd src
git clone https://github.com/yale-img/social_sim_ros src/social_sim_ros
```

- Build the Docker containers

```
yarn build
```

- Start the containers with

```
yarn up
```

- Enter a shell in the Docker Virtual Machine in your ROS workspace

```
yarn shell
```

- Then within a container, build the workspace with

```
catkin_make
```

The default nodes can then be launched by running `tmuxinator` in the `sim_ws` folder.
