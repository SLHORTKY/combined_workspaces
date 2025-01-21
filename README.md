if  you want to replicate the presented results you must 

the first modify bashrc as follows 
source /opt/ros/noetic/setup.bash

source ~/catkin_ws/devel/setup.bash


export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/path/to/your/plugin/directory

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/noetic/share/gazebo_models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/opt/ros/noetic/share/gazebo


export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/salihortakaya/catkin_ws/src/gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/salihortakaya/catkin_ws/src/gazebo_models_worlds_collection/worlds

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/salihortakaya/catkin_ws/src/pedsim_ros_with_gazebo/pedsim_gazebo_plugin/models

export PATH="/usr/bin/python3:$PATH"

export CMAKE_PREFIX_PATH=/opt/ros/noetic/share:$CMAKE_PREFIX_PATH


# Base path to the install directory
CARTOGRAPHER_INSTALL_PATH="/home/salihortakaya/catkin_ws_cartographer/install_isolated"

# Add to CMAKE_PREFIX_PATH
export CMAKE_PREFIX_PATH="${CARTOGRAPHER_INSTALL_PATH}:${CMAKE_PREFIX_PATH}"

# Add to LD_LIBRARY_PATH
export LD_LIBRARY_PATH="${CARTOGRAPHER_INSTALL_PATH}/lib:${LD_LIBRARY_PATH}"

# Add to PYTHONPATH (for Python modules)
export PYTHONPATH="${CARTOGRAPHER_INSTALL_PATH}/lib/python3/dist-packages:${PYTHONPATH}"

# Add to PATH (for executables)
export PATH="${CARTOGRAPHER_INSTALL_PATH}/bin:${PATH}"

# Add to ROS_PACKAGE_PATH (for ROS packages)
export ROS_PACKAGE_PATH="${CARTOGRAPHER_INSTALL_PATH}/share:${ROS_PACKAGE_PATH}"

# Add to PKG_CONFIG_PATH (for pkg-config files)
export PKG_CONFIG_PATH="${CARTOGRAPHER_INSTALL_PATH}/lib/pkgconfig:${PKG_CONFIG_PATH}"

then reconfigure the launch files in the catkin_ws workspace to properly address the required files.
