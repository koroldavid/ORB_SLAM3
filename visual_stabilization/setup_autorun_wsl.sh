#!/bin/bash

# 1. Install tmux
sudo apt update
sudo apt install tmux -y

# 2. Create autorun directory for all scripts
mkdir -p ~/autorun

# 2.1 MAVROS
cat > ~/autorun/start_mavros.sh << 'EOF'
#!/bin/bash
# Wait until /dev/ttyACM0 appears
while [ ! -e /dev/ttyACM0 ]; do
    echo "/dev/ttyACM0 not found, waiting..."
    sleep 2
done

echo "Starting MAVROS..."

# Launch MAVROS
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0:921600 fcu_protocol:="v2.0"

# Set stream rate
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"
EOF
chmod +x ~/autorun/start_mavros.sh

# 2.2 Left camera
cat > ~/autorun/start_left_camera.sh << 'EOF'
#!/bin/bash
while true; do
    if [ -e /dev/video0 ]; then
        echo "Left camera found, starting..."
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
        break
    else
        echo "Left camera not found, retrying..."
        sleep 2
    fi
done
EOF
chmod +x ~/autorun/start_left_camera.sh

# 2.3 Right camera
cat > ~/autorun/start_right_camera.sh << 'EOF'
#!/bin/bash
while true; do
    if [ -e /dev/video1 ]; then
        echo "Right camera found, starting..."
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
        break
    else
        echo "Right camera not found, retrying..."
        sleep 2
    fi
done
EOF
chmod +x ~/autorun/start_right_camera.sh

# 2.4 SLAM
cat > ~/autorun/start_slam.sh << 'EOF'
#!/bin/bash

CAMERA_TOPIC_NAME_LEFT="/left_camera/image_raw/compressed"
CAMERA_TOPIC_NAME_RIGHT="/right_camera/image_raw/compressed"
VOCABLUARY="//$HOME/ORB_SLAM3/Vocabulary/ORBvoc.txt"
CAMERA_CALIBRATION="//$HOME/ORB_SLAM3/visual_stabilization/calibration/slam_rpi5_stereo_calibration.yaml"

while true; do
    if ros2 topic list | grep "$CAMERA_TOPIC_NAME_LEFT" && ros2 topic list | grep "$CAMERA_TOPIC_NAME_RIGHT"; then
        echo "Camera topics ready, starting SLAM..."
        ros2 run drones_stabilization Stabilization $CAMERA_TOPIC_NAME_LEFT $CAMERA_TOPIC_NAME_RIGHT $VOCABLUARY $CAMERA_CALIBRATION
        break
    else
        echo "Camera topics not ready, waiting..."
        sleep 2
    fi
done
EOF
chmod +x ~/autorun/start_slam.sh

# 3. Script to launch all services via tmux
cat > ~/autorun/start_all_tmux.sh << 'EOF'
#!/bin/bash
SESSION="slam_session"

if tmux has-session -t $SESSION 2>/dev/null; then
    tmux kill-session -t $SESSION
fi

tmux new-session -d -s $SESSION

tmux send-keys -t $SESSION "bash ~/autorun/start_mavros.sh" C-m

tmux split-window -v -t $SESSION
tmux send-keys -t $SESSION "bash ~/autorun/start_left_camera.sh" C-m

tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "bash ~/autorun/start_right_camera.sh" C-m

tmux select-pane -t 0
tmux split-window -h -t $SESSION
tmux send-keys -t $SESSION "bash ~/autorun/start_slam.sh" C-m

exit 0
EOF
chmod +x ~/autorun/start_all_tmux.sh