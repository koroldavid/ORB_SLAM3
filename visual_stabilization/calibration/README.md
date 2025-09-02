# Camera calibration

NOTE: This instruction is extension of https://github.com/UbiquityRobotics/raspicam_node?tab=readme-ov-file#calibration

1. Print our a [chessboard_8x8.png](./print/chessboard_8x8.png), it is fine to print in A4 format

2. Start left and right camra with 30 fps.

Left
```
ros2 run camera_ros camera_node --ros-args \
  --remap __node:=left_camera \
  -p camera:=0 \
  -p camera_info_url:=file:///home/drones/ORB_SLAM3/visual_stabilization/calibration/left.yaml \
  -p width:=320 \
  -p height:=240 \
  -r /camera/image_raw:=/left_camera/image_raw \
  -r /camera/camera_info:=/stereo/left/camera_info \
  -r /camera/image_raw/compressed:=/left_camera/image_raw/compressed \
  -p FrameDurationLimits:="[32500,32500]"
```

Right
```
ros2 run camera_ros camera_node --ros-args \
  --remap __node:=right_camera \
  -p camera:=1 \
  -p camera_info_url:=file:///home/drones/ORB_SLAM3/visual_stabilization/calibration/right.yaml \
  -p width:=320 \
  -p height:=240 \
  -r /camera/image_raw:=/stereo/right/image_raw \
  -r /camera/camera_info:=/stereo/right/camera_info \
  -r /camera/image_raw/compressed:=/stereo/right/image_raw/compressed \
  -p FrameDurationLimits:="[32500,32500]"
```

2. Start calibration tool with you grid params e.g.

```
size="7x7"    # How many square your grid has? and substracted 1. (e.g. I had 8x8 grid)
square="0.02" # How big your square in meters? (e.g. I had 2cm square edge length)
```

```
ros2 run camera_calibration cameracalibrator \
  --size 7x7 \
  --square 0.02 \
  --no-service-check \
  --approximate 0.1 \
  --ros-args \
  --remap right:=/right_camera/image_raw \
  --remap left:=/left_camera/image_raw \
  --remap right_camera:=/stereo/right/camera_info \
  --remap left_camera:=/stereo/left/camera_info \
  --remap stereo:=true
```

3. Now move you grid to different poses in camera. Best follow advises from: https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

4. Save you calibration results and extract `left.yaml` and `right.yaml` files from calibration zip to `/home/drones/ORB_SLAM3/visual_stabilization/calibration` folder. 

5. Adjust SLAM calibration files based on following [this guide](./camera_calimration_to_yaml.md)

# IMU calibration

1. Enable your flight controller to run IMU in higher hz rate. For this you need to:
    - connect flight controller `Type C` port to laptop `USB` port.
    - start [mission planner](https://ardupilot.org/planner/docs/mission-planner-installation.html) app
    - Go to `Config` -> `Full Parameter List`
    - Change `SR0_RAW_SENSE` and `SR2_RAW_SENSE` value to hz (fps) you wish. It is fine if your value exceeds planner max value.

2. Start Mavros:
```
sudo chmod 666 /dev/ttyACM0
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600
```

3. Record only IMU for at least 4 hours in stale possition:

```
ros2 bag record -o /home/drones/ORB_SLAM3/visual_stabilization/calibration/imu_noise_bag --topics /mavros/imu/data
```

4. Conver it to ROS1 bag file
```
rosbags-convert --src /home/drones/ORB_SLAM3/visual_stabilization/calibration/imu_noise_bag --dst /home/drones/ORB_SLAM3/visual_stabilization/calibration/imu_noise.bag
```

5. Start docker ROS1 with allan_variance tool

```
FOLDER=/home/drones/ORB_SLAM3/visual_stabilization/calibration/
xhost +local:root
sudo docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" allan_variance
```

6. Calibrate IMU using [allan_variance_ros](https://github.com/ori-drs/allan_variance_ros)

```
source devel/setup.bash
roscore & sleep 5

rosrun allan_variance_ros cookbag.py --input /data/imu_noise.bag --output cooked.bag
rosrun allan_variance_ros allan_variance /data /data/allan.yaml
rosrun allan_variance_ros analysis.py --data /data/allan_variance.csv
```

7. Now you have your [kalibr_imu.yaml](./visual_stabilization/calibration/kalibr_imu.yaml)

# IMU + Camera calibration

Now with `kalibr_imu.yaml` file we can calibrate distance matrix between camera and imu.

1. Print out [april grid](./print/april_6x6.pdf) as A4 format and put in on a wall.

2. Start left camera and mavros as in steps above.

3. Record april grid using drone with IMU and left camera running as in [this youtube guidance](https://www.youtube.com/watch?app=desktop&v=puNXsnrYWTY&ab_channel=SimpleKernel).

```
ros2 bag record -o /home/drones/ORB_SLAM3/visual_stabilization/calibration/april_bag --topics /left_camera/image_raw /mavros/imu/data
```

4. Conver it to ROS1 bag file

```
rosbags-convert --src /home/drones/ORB_SLAM3/visual_stabilization/calibration/april_bag --dst /home/drones/ORB_SLAM3/visual_stabilization/calibration/april_calib.bag
```

5. Start tool in docker container to calibrate data:

```
FOLDER=/home/drones/ORB_SLAM3/visual_stabilization/calibration/
xhost +local:root
sudo docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr
```

6. Run calibration commands data:

```
source devel/setup.bash
rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/april_calib.bag \
    --target /data/april_6x6_A4.yaml \
    --models pinhole-radtan \
    --topics /left_camera/image_raw

rosrun kalibr kalibr_calibrate_imu_camera \
    --bag /data/april_calib.bag \
    --target /data/april_6x6_A4.yaml \
    --imu /data/kalibr_imu.yaml \
    --cam /data/april_calib-camchain.yaml
```

7. From `april_calib_camchain-imucam.yaml` transfer values of `T_cam_imu` to SLAM config `IMU.T_b_c1` param.
