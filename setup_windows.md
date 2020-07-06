## Windows Installation

This guide will walk you through setting up the simulation environment including Unity and ROS.

### Dependencies

Start with a Windows 10 desktop installation.

If you need a git client, download and install [git](https://git-scm.com).

You'll also need [git lfs](https://git-lfs.github.com)

### Unity

If you only need to test your algorithm or make changes in ROS, follow the [Unity Binary Setup](#binary-setup). If you also need to make edits to the simulator, environment or APIs, follow the [Unity Editor](#editor-setup). In either case, you'll need a working [ROS Setup](#ros-setup).

#### Binary Setup

Download the latest release of SEE from the release page: [https://gitlab.com/interactive-machines/simulation/social-sim/-/releases](https://gitlab.com/interactive-machines/simulation/social-sim/-/releases
).

Now, continue to setting up [ROS](#ros-setup).

#### Editor Setup

Following the [Unity Getting Started Guide](https://docs.unity3d.com/Manual/GettingStartedInstallingHub.html) to download and install the [UnityHub](https://unity3d.com/get-unity/download).

- Download and install UnityHub from: [https://unity3d.com/get-unity/download](https://unity3d.com/get-unity/download)

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

You can follow the [ROS Melodic Setup Guide](http://wiki.ros.org/Installation/Windows).

You'll need to:

- Install [Visual Studio Community]

  - This should already be installed with Unity, so run the Visual Studio Installer and add the latest Windows 10 SDK

- Install Chocolatey

  - In the Start Menu, find the "x64 Native Tools Command Prompt for VS 2019" item.
  - Right Click, select More then "Run as Administrator"
  - Run:

```
@"%SystemRoot%\System32\WindowsPowerShell\v1.0\powershell.exe" -NoProfile -InputFormat None -ExecutionPolicy Bypass -Command "iex ((New-Object System.Net.WebClient).DownloadString('https://chocolatey.org/install.ps1'))" && SET "PATH=%PATH%;%ALLUSERSPROFILE%\chocolatey\bin"
```

- Install vcpkg

  - Clone to `c:\opt`

```
git clone https://github.com/Microsoft/vcpkg.git c:\opt\vcpkg
```

  - Run `.\bootstrap-vcpkg.bat`

- Install the `ros-melodic-desktop_full`

[Setup your workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) as normal. Then, from within your workspace, add the git repository:

```
git clone https://github.com/yale-img/social_sim_ros src/social_sim_ros
```

Then build with `cakin_make`