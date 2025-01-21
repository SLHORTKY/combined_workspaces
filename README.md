# Setup Instructions

To reproduce the presented results, you can follow the steps below to configure your environment.

## 1. Modify the `bashrc` File

To set up the necessary environment variables, modify the `.bashrc` file.

1. Open your terminal and run:
   ```bash
   nano ~/.bashrc
# Source ROS Noetic setup file
source /opt/ros/noetic/setup.bash

# Source your Catkin workspace setup file
source ~/catkin_ws/devel/setup.bash

# Set up Gazebo plugin paths
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/path/to/your/plugin/directory

# Set up Gazebo model and resource paths
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/noetic/share/gazebo_models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/noetic/share/gazebo

# Custom Gazebo model and resource paths for your Catkin workspace
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/your/catkin_ws/src/gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/path/to/your/catkin_ws/src/gazebo_models_worlds_collection/worlds

# Pedestrian simulation Gazebo plugin models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/path/to/your/catkin_ws/src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/models

# Add Python3 to PATH
export PATH="/usr/bin/python3:$PATH"

# Update CMake prefix path
export CMAKE_PREFIX_PATH=/opt/ros/noetic/share:$CMAKE_PREFIX_PATH

# Add Cartographer installation paths
CARTOGRAPHER_INSTALL_PATH="/path/to/your/catkin_ws_cartographer/install_isolated"

export CMAKE_PREFIX_PATH="${CARTOGRAPHER_INSTALL_PATH}:${CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="${CARTOGRAPHER_INSTALL_PATH}/lib:${LD_LIBRARY_PATH}"
export PYTHONPATH="${CARTOGRAPHER_INSTALL_PATH}/lib/python3/dist-packages:${PYTHONPATH}"
export PATH="${CARTOGRAPHER_INSTALL_PATH}/bin:${PATH}"
export ROS_PACKAGE_PATH="${CARTOGRAPHER_INSTALL_PATH}/share:${ROS_PACKAGE_PATH}"
export PKG_CONFIG_PATH="${CARTOGRAPHER_INSTALL_PATH}/lib/pkgconfig:${PKG_CONFIG_PATH}"
