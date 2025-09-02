# ROS 2

## Preparation

1. Follow [full calibration guidance](./calibration/README.md) required for proper work of SLAM.
2. To communicate between your PC WSL and Raspberi Pi you need to follow some [network adjustments steps](./window_network.md).
3. Adupilot params setup:
  - VISO_TYPE = 1;
  - AHRS_GPS_USE = 0;
  - FS_EKF_THRESH = 0;

  - EK3_SRC1_POSXY = 6;
  - EK3_SRC1_POSZ = 1;
  - EK3_SRC1_VELXY = 6;
  - EK3_SRC1_VELZ = 6;
  - EK3_SRC1_YAW = 6;
  - Same values for EK3_SRC2 and EK3_SRC3;

  - EK3_VELD_M_NSE = 0.1;
  - EK3_VELNE_M_NSE = 0.1;
  - EK3_POSNE_M_NSE = 0.1;
  - COMPASS_ENABLED = 0;
  - COMPASS_USE = 0;
  - COMPASS_USE2 = 0;
  - COMPASS_USE3 = 0;

  - SR0_PARAMS = 5;
  - SR0_POSITION = 10;
  - SR0_EXT_STAT = 10;

## Start All

1. Start mavros

- sudo chmod 666 /dev/ttyACM0
- ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 fcu_protocol:="v2.0"
- ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"

Note: `ttyACM0` for USB connection

2. Start left and right camera with 30 fps

Left
```
ros2 run camera_ros camera_node --ros-args \
  --remap __node:=left_camera \
  -p camera:=0 \
  -p camera_info_url:=file://$HOME/ORB_SLAM3/visual_stabilization/calibration/left.yaml \
  -p width:=320 \
  -p height:=240 \
  -r /camera/image_raw:=/stereo/left/image_raw \
  -r /camera/camera_info:=/stereo/left/camera_info \
  -r /camera/image_raw/compressed:=/stereo/left/image_raw/compressed \
  -p FrameDurationLimits:="[32500,32500]"
```

Right
```
ros2 run camera_ros camera_node --ros-args \
  --remap __node:=right_camera \
  -p camera:=1 \
  -p camera_info_url:=file://$HOME/ORB_SLAM3/visual_stabilization/calibration/right.yaml \
  -p width:=320 \
  -p height:=240 \
  -r /camera/image_raw:=/stereo/right/image_raw \
  -r /camera/camera_info:=/stereo/right/camera_info \
  -r /camera/image_raw/compressed:=/stereo/right/image_raw/compressed \
  -p FrameDurationLimits:="[32500,32500]"
```

Mono USB
```
ros2 run usb_cam usb_cam_node_exe --ros-args \
  --remap __node:=mono_camera \
  -p video_device:=/dev/video0 \
  -p image_width:=640 \
  -p image_height:=480 \
  -p pixel_format:="mjpeg2rgb" \
  -r image_raw:=/mono/image_raw \
  -r /camera/image_raw/compressed:=/mono/image_raw/compressed \
  -r camera_info:=/mono/camera_info \
  -p FrameDurationLimits:="[32500,32500]"
```

3. Start SLAM

Without IMU:
```
CAMERA_TOPIC_NAME_LEFT="/left_camera/image_raw/compressed"
CAMERA_TOPIC_NAME_RIGHT="/right_camera/image_raw/compressed"
VOCABLUARY="//$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIBRATION="//$HOME/ORB_SLAM3/visual_stabilization/calibration/slam_rpi5_stereo_calibration.yaml"

ros2 run drones_stabilization Stabilization $CAMERA_TOPIC_NAME_LEFT $CAMERA_TOPIC_NAME_RIGHT $VOCABLUARY $CAMERA_CALIBRATION
```
x
OR with IMU:
```
CAMERA_TOPIC_NAME_LEFT="/left_camera/image_raw/compressed"
CAMERA_TOPIC_NAME_RIGHT="/right_camera/image_raw/compressed"
IMU_TOPIC_NAME="/mavros/imu/data"
VOCABLUARY="//$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIBRATION="//$HOME/ORB_SLAM3/visual_stabilization/calibration/slam_rpi5_stereo_calibration.yaml"

ros2 run drones_stabilization Stabilization $CAMERA_TOPIC_NAME_LEFT $CAMERA_TOPIC_NAME_RIGHT $IMU_TOPIC_NAME $VOCABLUARY $CAMERA_CALIBRATION
```

OR MONO USB:
```
CAMERA_TOPIC_NAME="/image_raw/compressed"
IMAGE_FORMAT="mono"
VOCABLUARY="//$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIBRATION="//$HOME/ORB_SLAM3/visual_stabilization/calibration/slam_rpi5_stereo_calibration.yaml"

ros2 run drones_stabilization Stabilization $CAMERA_TOPIC_NAME $IMAGE_FORMAT $VOCABLUARY $CAMERA_CALIBRATION
```