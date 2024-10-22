cmake_minimum_required(VERSION 3.0.2)
project(offboard_py)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  dynamic_reconfigure
)

find_package(Python3 REQUIRED)

catkin_python_setup()

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
  cfg/VelocityConfig.cfg
  cfg/TargetConfig.cfg
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs mavros_msgs dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Add cmake target dependencies of the package
add_dependencies(${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})



#############
## Install ##
#############

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/target.py
  scripts/velocity.py
  scripts/analysis.py
  scripts/main.py
  scripts/tracker.py
  scripts/image_resizer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})