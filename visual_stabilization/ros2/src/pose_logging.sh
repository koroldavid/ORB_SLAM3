while true; do
    ros2 topic echo --once /mavros/vision_pose/pose --field pose.position | awk '
    /x:/ {x=$2}
    /y:/ {y=$2}
    /z:/ {z=$2}
    END {print x","y","z}
    '
    >> vision_pose.csv & \
    ros2 topic echo --once /mavros/local_position/pose --field pose.position | awk '
    /x:/ {x=$2}
    /y:/ {y=$2}
    /z:/ {z=$2}
    END {print x","y","z}
    '
    >> local_position.csv
    
    sleep 5
done