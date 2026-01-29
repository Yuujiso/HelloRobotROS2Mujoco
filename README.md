# Mac OS Parallels Desktops (source)[https://askubuntu.com/questions/1405124/install-ubuntu-desktop-22-04-arm64-on-macos-apple-silicon-m1-pro-max-in-parall]

On ARM64 Ubuntu Server Edition

```bash
# Download the ARM64 Server edition from: https://ubuntu.com/download/server/arm
# Install with the ISO when creating a new guest/vm in Parallels 17
# Update System
sudo apt update && sudo apt upgrade
# Reboot
sudo reboot
# Login and install
sudo apt install ubuntu-desktop
# Reboot
sudo reboot
# Parallels Tools Installation: two options, but I used this
# Use Menubar: Menubar → Actions → Install Parallels Tools
# Install via shell
cd /media/USERNAME/Parallels\ Tools/
sudo ./install
sudo reboot
```

**Note**: I once broke the UI. If you do the same, you should reinstall the `ubuntu-desktop`

# Stretch Simulation Tutorial

This tutorial will guide you through setting up and using the Stretch robot simulation in ROS 2.

Note: You do not need to follow this tutorial if you are running on a Stretch robot.

---

## What You'll Learn

By the end of this tutorial, you'll be able to:

- Install and configure the Stretch simulation environment
- Launch the simulated robot in various environments
- Control the robot using keyboard teleop and web interface
- Use the simulation for navigation and mapping
- Integrate the simulation with your own ROS 2 applications

---

## Prerequisites

Before starting this tutorial, you should have:

- Basic familiarity with the Linux command line
- Understanding of ROS 2 concepts (nodes, topics, services). If you're new to ROS 2 concepts, we recommend first reading through the [Introduction to ROS 2](../intro_to_ros2/) tutorial.
- A computer meeting the system requirements below

---

## System Requirements

**Operating System**: Ubuntu 22.04 (recommended) or WSL2 with GPU acceleration

**Hardware Requirements**:

- **Minimum**: 16GB RAM, Nvidia graphics card, at least 64GB free memory (more is better, trust me)
- **Recommended**: 32GB RAM, dedicated GPU

**Why these requirements?** The simulation utilizes the Mujoco physics engine with 3D rendering, which requires significant computational resources and GPU acceleration for smooth performance.

---

## Installation Guide

### Step 1: Install ROS 2 Humble (10 minutes)

**Note**: Skip this step if you're running on a Stretch robot, as ROS 2 is already installed.

```bash
# Add universe repository for additional packages
sudo apt install software-properties-common
sudo add-apt-repository universe

# Install curl for downloading ROS 2 keys
sudo apt update && sudo apt install curl -y

# Add ROS 2 official repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null


# Update package list and install ROS 2
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools rviz python3-pip

# Source ROS 2 environment (you'll need to do this in every new terminal)
source /opt/ros/humble/setup.bash
```

**Expected output**: After installation, you should be able to run ros2 --help and see the ROS 2 command options.

```bash
ros2 --help
```

---

### Step 2: Install Node.js and npm (5 minutes)

Node.js is required for the web teleop interface that allows you to control the robot through a web browser.

```bash
# Install Node.js 22.x (latest LTS version)
curl -sL https://deb.nodesource.com/setup_22.x | sudo -E bash -
sudo apt install -y nodejs

# Verify installation
node --version  # Should show v22.x.x
npm --version   # Should show npm version
```

---

### Step 3: Set up the ROS 2 Workspace (1 hour)

**Note**: Skip this step if you're running on a Stretch robot.

A ROS 2 workspace is a directory containing ROS 2 packages. We'll create an ament_ws workspace with all the necessary Stretch packages.

**⚠️ Warning**: This will delete any existing ~/ament_ws directory. Back up important files first.

```bash
# For AMR64 Download (my custom script, it's different from default doc)
curl -sL https://raw.githubusercontent.com/Yuujiso/HelloRobotROS2Mujoco/34fc3bff96adbf68c01d4435ccecaa2167673f1b/stretch_create_ament_workspace.sh > /tmp/stretch_create_ament_workspace.sh
```

```bash
# For AMD64 Download (default doc script)
curl -sL https://raw.githubusercontent.com/hello-robot/stretch_ros2/refs/heads/humble/stretch_simulation/stretch_create_ament_workspace.sh > /tmp/stretch_create_ament_workspace.sh
```

```bash
# Run the workspace setup script
bash /tmp/stretch_create_ament_workspace.sh

# Add ROS 2 environment to your shell profile (optional but recommended)
echo 'source ~/ament_ws/install/setup.bash' >> ~/.bashrc
```

**Troubleshooting**: If you encounter a numpy header error during compilation:

```bash
sudo ln -s ~/.local/lib/python3.10/site-packages/numpy/core/ /usr/include/numpy
```

**Expected output**: After successful setup, you should see these packages:

```bash
$ ls ~/ament_ws/src
audio_common  realsense-ros  respeaker_ros2  ros2_numpy  rosbridge_suite  sllidar_ros2  stretch_ros2  stretch_tutorials  stretch_web_teleop  tf2_web_republisher_py
```

---

### Step 4: Set up Robot Description (URDF) (15 minutes)

URDF (Unified Robot Description Format) files describe the robot's physical structure, joints, and sensors. This step downloads the 3D models and configurations for the Stretch robot.

```bash
# Source the ROS 2 environment
source ~/ament_ws/install/setup.bash

# Download URDF repository
git clone https://github.com/hello-robot/stretch_urdf.git --depth 1 /tmp/stretch_urdf

# Install Stretch body
python3 -m pip install hello-robot-stretch-body
```

Edit the /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py file to initialize Variables or run script with params `--model 'SE3' --tool 'eoa_wrist_dw3_tool_sg3_pro'`

```py
############## Initialize Variables #####################

tool_name = 'eoa_wrist_dw3_tool_sg3_pro'
model_name = 'SE3'
root_dir = None
ros_repo_path = None
ros_version = None
data_dir = None
```

```bash
# ++++++ Maybe?
cd /tmp/stretch_urdf
python3 urdf_generate.py

# KeyError: 'HELLO_FLEET_PATH' open new terminal
source ~/ament_ws/install/setup.bash

# Update URDF files for ROS 2

python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py  --model 'SE3' --tool 'eoa_wrist_dw3_tool_sg3_pro'
python3 /tmp/stretch_urdf/tools/stretch_urdf_ros_update.py  --ros2_rebuild --model 'SE3' --tool 'eoa_wrist_dw3_tool_sg3_pro'
```

**Expected output**: You should see URDF files in the stretch_description package:

```bash
$ ls ~/ament_ws/src/stretch_ros2/stretch_description/urdf/
d405  d435i  stretch_description.xacro  stretch_main.xacro  # ... and many more files
```

---

### Step 5: Set up Mujoco Simulation (15 minutes)

```bash
# Upgrade pip (important for dependency installation)
pip3 install --upgrade pip

# Source ROS 2 environment
source ~/ament_ws/install/setup.bash

# Run interactive setup script (will ask about robocasa environments)
sh ~/ament_ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/setup.sh

# ++++++++ Fix the mujoco version
pip install mujoco==3.2.6


# Fix OpenGL compatibility issue
pip install PyOpenGL==3.1.4

# Build the workspace
cd ~/ament_ws
source ~/ament_ws/install/setup.bash
colcon build
```

**What is robocasa?** When prompted, robocasa provides realistic kitchen environments for testing household robotics tasks. Choose 'yes' if you want these additional environments.

### Running Your First Simulation

Now let's test the installation by launching the simulation:

```bash
# Source the environment
source ~/ament_ws/install/setup.bash

# +++++++ Fix (related to launching simulation issue on numpy random generator)
pip install numpy==1.23.3

# Download the Kitchen scene if you have not
python3 ~/ament_ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/dependencies/stretch_mujoco/third_party/robocasa/robocasa/scripts/download_kitchen_assets.py

python3 ~/ament_ws/src/stretch_ros2/stretch_simulation/stretch_mujoco_driver/dependencies/stretch_mujoco/third_party/robocasa/robocasa/scripts/setup_macros.py

pip install robosuite_models

pip install robomimic

# Launch the simulation in navigation mode
ros2 launch stretch_simulation stretch_mujoco_driver.launch.py mode:=navigation
```

**Expected output**: - A Mujoco viewer window should open showing the Stretch robot in a kitchen environment - ROS 2 nodes should start without errors - You should see log messages indicating successful initialization

**Troubleshooting**: - If you see GPU-related errors, try: export MUJOCO_GL=egl before launching - If the viewer doesn't open, ensure you have proper graphics drivers installed
