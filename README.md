# thesis_avalor
quadcopter drone interception using PX4 Autopilot


Installation guide

consists of 4 steps: 
- install ubuntu
- install px4
- install ros
- install mavros
- install this repository

Step 1: install ubuntu 20.04 (skip to step 2 if already installed)

Ubuntu 20.04 is the last version to support ROS1, which is needed to run Mavros. 

Download the right image on: 
https://cdimage.ubuntu.com/releases/20.04.5/release/

follow installation instruction from: 
https://medium.com/@shubhjain10102003/install-linux-ubuntu-20-04-on-m1-m2-mac-silicon-de1992d5fa26

Step 2: Install px4

Install PX4 without the simulator toolchain:

Install git if you had not done so: 

sh
sudo apt update
sudo apt upgrade
sudo apt install git

Download PX4 Source Code: 

sh
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
Run the ubuntu.sh the --no-sim-tools (and optionally --no-nuttx):

sh
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh --no-sim-tools --no-nuttx
Acknowledge any prompts as the script progress.
Restart the computer on completion.

You may need to install the following additional dependencies:

sh
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y

Step 3 download ROS Noetic: 

follow instruction below: 
https://wiki.ros.org/noetic/Installation/Ubuntu#Installation

step 4: Install mavros

Use apt-get for installation, where ${ROS_DISTRO} below should resolve to kinetic or noetic, depending on your version of ROS:

sh
sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras ros-${ROS_DISTRO}-mavros-msgs
Then install GeographicLib datasets by running the install_geographiclib_datasets.sh script:

sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
Source Installation
This installation assumes you have a catkin workspace located at ~/catkin_ws If you don't create one with:

sh
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
wstool init src
You will be using the ROS Python tools: wstool (for retrieving sources), rosinstall, and catkin_tools (building) for this installation. While they may have been installed during your installation of ROS you can also install them with:

sh
sudo apt-get install python-catkin-tools python-rosinstall-generator -y
TIP

While the package can be built using catkin_make the preferred method is using catkin_tools as it is a more versatile and "friendly" build tool.

If this is your first time using wstool you will need to initialize your source space with:

sh
$ wstool init ~/catkin_ws/src
Now you are ready to do the build:

Install MAVLink:

sh
# We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
Install MAVROS from source using either released or latest version:

Released/stable

sh
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
Latest source

sh
rosinstall_generator --upstream-development mavros | tee -a /tmp/mavros.rosinstall
sh
# For fetching all the dependencies into your catkin_ws,
# just add '--deps' to the above scripts, E.g.:
#   rosinstall_generator --upstream mavros --deps | tee -a /tmp/mavros.rosinstall
Create workspace & deps

sh
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
Install GeographicLib datasets:

sh
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
Build source

sh
catkin build
Make sure that you use setup.bash or setup.zsh from workspace.


Before able to build it. multiple errors may occure. This is how I solved mine. 


error 1: 
error: ‘struct mavlink::common::msg::GPS2_RAW’ has no member named ‘hdg_acc’
  111 |   ros_msg->hdg_acc           = mav_msg.hdg_acc;

solution: open the file and comment out the lines that cause the error




sh
#Needed or rosrun can't find nodes from this workspace.
source devel/setup.bash

