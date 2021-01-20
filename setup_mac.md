## Mac Installation

This guide will walk you through setting up the simulation environment including Unity and ROS on MacOS.

### Unity

#### Editor Setup

Following the [Unity Getting Started Guide](https://docs.unity3d.com/Manual/GettingStartedInstallingHub.html) to download and install the [UnityHubSetup.dmg](https://public-cdn.cloud.unity3d.com/hub/prod/UnityHubSetup.dmg).

- Download the UnityHub installer here: [https://unity3d.com/get-unity/download](https://unity3d.com/get-unity/download)

- Install UnityHub

- Login to UnityHub. If you do not have an account create a Unity account now.

Note: signing up for a student account, though not required, will give you access to some free assets: [https://assetstore.unity.com/browse/student-plan-pack](https://assetstore.unity.com/browse/student-plan-pack)

- Install Unity Version `2019.4.0f1` from within UnithHub

Now, continue to setting up the [Unity Project](#unity-project).

### Unity Project

- Clone the unity project, the location of this project is not important, but we'll clone it to the home directory:

```
git clone --recurse-submodules --remote-submodules https://github.com/yale-sean/social_sim_unity.git ~/social_sim_unity
```

In Unity Hub, add the project you just cloned by clicking the "Add" button.

Run the project by clicking on the project name: `social_sim_unity`

*Note that if you are unable to add the project, create a new project by clicking the "New" button, restart Unity Hub, and then try adding the project again*

### ROS Setup

ROS will be run in an Ubuntu Docker container on your Mac. Install the following tools to enable this:

- [Docker for Mac](https://docs.docker.com/docker-for-mac/install/)

```
brew install --cask docker
```

#### Checkout Code

- Clone the workspace into your home folder and `cd` into the workspace:

```
git clone --recurse-submodules --remote-submodules https://github.com/yale-sean/sim_ws.git ~/sim_ws
cd ~/sim_ws
```

- Clone the dependencies into the project `src` folder

```
mkdir -p src
cd src
git clone --recurse-submodules --remote-submodules https://github.com/yale-sean/social_sim_ros social_sim_ros
```

- Navigate up to the 'sim_ws' folder and build the Docker container

```
cd ~/sim_ws
./container build rosmac
```

Note: The Docker Desktop application should be running during these steps. Further, building is only required on the first run or if the docker container changes, so in future runs, this step can be skipped.

- Start the containers with

```
./container start rosmac
```

- Enter a shell in the Docker Virtual Machine in your ROS workspace

```
./container shell rosmac
```

- Then within a container, you'll notice the default working directory is `~/sim_ws`. Build the workspace with:

```
catkin_make
```

- Open as many terminal windows as you wish and run multiple shell instances in the Docker container with:

```
./container shell rosmac
```

- To launch a graphical program, like `rviz`, run:

```
./container shell rosmac
./sim_ws/docker/rosmac/vnc.sh
```

Then open [http://localhost:6080](http://localhost:6080) in your browser

You can launch rviz from either your terminal (with `export DISPLAY=:1.0` then the program name) or the web browser.
