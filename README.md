# Thesis_Avalor: Quadcopter Drone Interception Using PX4 Autopilot

## Installation Guide

This guide consists of 4 steps to set up your environment:

1. **Install Ubuntu 20.04**
2. **Install PX4**
3. **Install ROS Noetic**
4. **Install MAVROS**

### Step 1: Install Ubuntu 20.04

Ubuntu 20.04 is the last version to support ROS1, which is required to run MAVROS.

- Download the Ubuntu 20.04 image from: [Ubuntu Releases](https://cdimage.ubuntu.com/releases/20.04.5/release/)
- Follow the installation instructions for your specific hardware: [Installation Guide](https://medium.com/@shubhjain10102003/install-linux-ubuntu-20-04-on-m1-m2-mac-silicon-de1992d5fa26)

### Step 2: Install PX4

**Install PX4 without the simulator toolchain:**

1. Install Git if you haven't already:

    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install git
    ```

2. Clone the PX4 source code:

    ```bash
    git clone https://github.com/PX4/PX4-Autopilot.git --recursive
    ```

3. Run the setup script without simulation tools and optionally without NuttX:

    ```bash
    bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
    ```

   Follow any prompts as the script progresses. Restart your computer upon completion.

4. Install additional dependencies:

    ```bash
    sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
    ```

5. Build PX4 with Gazebo Classic:

    ```bash
    make px4_sitl gazebo-classic
    ```

### Step 3: Download ROS Noetic

Follow the installation instructions for ROS Noetic: [ROS Noetic Installation](https://wiki.ros.org/noetic/Installation/Ubuntu#Installation)

### Step 4: Install MAVROS

1. Use `apt-get` to install MAVROS and its extras. Replace `${ROS_DISTRO}` with `noetic` (for ROS Noetic):

    ```bash
    sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
    ```

2. Install GeographicLib datasets:

    ```bash
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
    sudo bash ./install_geographiclib_datasets.sh
    ```

**Source Installation:**

This installation assumes you have a catkin workspace located at `~/catkin_ws`. If not, create one:

    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin init
    wstool init src
    ```

You will use the ROS Python tools: `wstool`, `rosinstall`, and `catkin_tools`. They may have been installed with ROS, but you can also install them:

    ```bash
    sudo apt-get install python-catkin-tools python-rosinstall-generator -y
    ```

**Tip:** While you can use `catkin_make`, `catkin_tools` is the preferred method as it is more versatile.

Initialize your source space with `wstool`:

    ```bash
    wstool init ~/catkin_ws/src
    ```

**Build MAVROS:**

1. Install MAVLink:

    ```bash
    rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
    ```

2. Install MAVROS from source (choose released/stable or latest source):

   - Released/stable:

      ```bash
      rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
      ```

   - Latest source:

      ```bash
      rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
      ```

3. Fetch all dependencies:

    ```bash
    rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall
    ```

4. Create workspace & dependencies:

    ```bash
    wstool merge -t src /tmp/mavros.rosinstall
    wstool update -t src -j4
    rosdep install --from-paths src --ignore-src -y
    ```

5. Build the workspace:

    ```bash
    ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
    catkin build
    ```

6. Ensure your `.bashrc` includes:

    ```bash
    source /opt/ros/noetic/setup.bash
    source /home/cle/catkin_ws/src/PX4-Autopilot/Tools/simulation/gazebo-classic/setup.bash
    export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/cle/catkin_ws/src/PX4-Autopilot
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/aarch64-linux-gnu/gazebo-11/plugins
    export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}
    ```

   Source your updated `.bashrc`:

    ```bash
    source ~/.bashrc
    ```

**Common Issues:**

- **Error:** `error: ‘struct mavlink::common::msg::GPS2_RAW’ has no member named ‘hdg_acc’`
  - **Solution:** Open the file and comment out the lines that cause the error.

- **Error:** `Resource not found: px4 ROS path [0]=/opt/ros/noetic/share/ros`
  - **Solution:** Ensure your `.bashrc` includes the proper ROS and PX4 paths as described above.

Feel free to reach out if you encounter any issues or need further assistance!
