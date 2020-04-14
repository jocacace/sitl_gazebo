export FIRMWARE_DIR=../../

cd $FIRMWARE_DIR
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

cd

export PX4_HOME_LAT=40.817542
export PX4_HOME_LON=14.158190


