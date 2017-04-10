#!/bin/bash
# ROS Environment
#USER=stdops
VEHICLE=(apad amussel)
#ROS_HOME=/home/${USER}/ros
#source ${ROS_HOME}/devel/setup.bash
#export ROS_MASTER_URI=http://localhost:11311

# Vehicle configuration environment
for v in ${VEHICLE};
do
    source `rospack find $v`/data/devices/device_config.bash
    source `rospack find $v`/data/control/control_config.bash
    source `rospack find $v`/data/navigation/navigation_config.bash
done

# Launch configuration
export LAUNCH_PKG=subcultron_launch
export LAUNCH_FILE=standard_simulation.launch

# Configure logging
#export LOG_PATH=/home/${USER}/logs/launcher
#export ROS_LOG_DIR=/home/${USER}/logs/ros
export LOGGING=false

# Configure simulation
export XML_SAVE_PATH=`rospack find apad`/data/mission.xml
export USE_NOISE=false

#Location for simulation or predefined position
export LOCATION=labos
export USE_LOCAL_FIX=1
