echo "Building ROS2 nodes"

cd visual_stabilization/ros2
mkdir build
colcon build --symlink-install
