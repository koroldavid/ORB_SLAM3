# Install ROS2
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade
sudo apt install -y ros-jazzy-ros-base
source /opt/ros/jazzy/setup.bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

sudo apt install -y python3-rosdep 
sudo rosdep init

# Install mavros
sudo apt install -y ros-jazzy-mavros ros-jazzy-mavros-extras 
sudo apt install -y ros-jazzy-rqt ros-jazzy-rqt-image-view ros-jazzy-compressed-image-transport
sudo apt install -y ros-jazzy-camera-calibration
cd ~/
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# Install camera images reader
# TODO: Works manually, but camera install fails when running automated somewhere...
sudo apt update
sudo apt install -y v4l-utils qv4l2 # to USB camera
sudo apt install -y git python3-pip python3-jinja2 libboost-dev openssl
sudo apt install -y libgnutls28-dev libtiff-dev pybind11-dev meson cmake
sudo apt install -y python3-yaml python3-ply libglib2.0-dev libgstreamer-plugins-base1.0-dev

git clone https://github.com/raspberrypi/libcamera.git
cd libcamera
meson setup build --buildtype=release -Dpipelines=rpi/vc4,rpi/pisp -Dipas=rpi/vc4,rpi/pisp -Dv4l2=true -Dgstreamer=enabled -Dtest=false -Dlc-compliance=disabled -Dcam=enabled -Dqcam=disabled -Ddocumentation=disabled -Dpycamera=enabled
sudo ninja -C build install

mkdir -p ~/rpi_camera/src
cd ~/rpi_camera/src
git clone https://github.com/christianrauch/camera_ros.git
cd ~/rpi_camera
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO --skip-keys=libcamera

colcon build --event-handlers=console_direct+
echo "/usr/local/lib/aarch64-linux-gnu" | sudo tee /etc/ld.so.conf.d/libcamera.conf
echo 'export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH' >> ~/.bashrc
# echo "sudo chmod -R 777 /dev/" >> ~/.bashrc
echo "source ~/rpi_camera/install/setup.bash" >> ~/.bashrc
sudo chmod -R 777 /dev/
source ~/rpi_camera/install/setup.bash

# Install USB_CAM
# cd ~/ORB_SLAM3/visual_stabilization/ros2/src
# git clone https://github.com/ros-drivers/usb_cam.git

# Install exta deps
sudo apt-get update
sudo apt-get install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
sudo apt-get install -y libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libxvidcore-dev libx264-dev
sudo apt-get install -y libatlas-base-dev gfortran python3-dev libeigen3-dev libboost-all-dev libsuitesparse-dev
sudo apt install -y libegl-dev libgl1-mesa-dev libopengl-dev libepoxy-dev python3-pip colcon
python3.12 -m pip install --user wheel

# Setup utorun
bash ~/ORB_SLAM3/visual_stabilization/setup_autorun_wsl.sh

# Docker
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Install python rosbags-convert tool
sudo apt install -y pipx
pipx install rosbags
pipx ensurepath

# Install ROS1 imu calibration tool via docker
cd ~/ORB_SLAM3/visual_stabilization/calibration
sudo docker build -t allan_variance -f allan_variance .

# Install ROS1 imu-camera calibration tools via docker
sudo apt install -y x11-xserver-utils
cd ~
git clone https://github.com/ethz-asl/kalibr.git
cd kalibr
sudo docker build -t kalibr -f Dockerfile_ros1_18_04 .

# Opencv
sudo dpkg --remove --force-all python3-catkin-pkg
sudo dpkg --remove --force-remove-reinstreq python3-rospkg python3-rosdistro
sudo apt --fix-broken install
sudo apt install -y python3-catkin-pkg-modules python3-rospkg-modules python3-rosdistro-modules
sudo apt install -y libopencv-dev python3-opencv ros-jazzy-cv-bridge

# Install Pangolin
cd ~
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build
cd build
cmake ..
make -j2
sudo make install

# Build ORB_SLAM3
cd ~/ORB_SLAM3

mv CMakeLists_ubuntu_24.txt CMakeLists.txt
./build.sh
chmod +x ./visual_stabilization/build_ros2.sh
./visual_stabilization/build_ros2.sh
source ~/ORB_SLAM3/visual_stabilization/ros2/install/setup.bash
echo "source ~/ORB_SLAM3/visual_stabilization/ros2/install/setup.bash" >> ~/.bashrc

sudo ldconfig
