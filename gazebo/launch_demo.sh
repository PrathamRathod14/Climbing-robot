#!/bin/bash
export GAZEBO_PLUGIN_PATH=/mnt/c/Users/prath/Desktop/geckogrip-climber/gazebo:$GAZEBO_PLUGIN_PATH
export DISPLAY=:0
export WAYLAND_DISPLAY=wayland-0

# Starting gzserver (or remote connection) with cleanup

REMOTE_GAZEBO_HOST=${REMOTE_GAZEBO_HOST:-127.0.0.1}
export GAZEBO_MASTER_URI="http://${REMOTE_GAZEBO_HOST}:11345"

if [[ "$REMOTE_GAZEBO_HOST" == "127.0.0.1" || "$REMOTE_GAZEBO_HOST" == "localhost" ]]; then
    echo "[Launch Script] Starting local gzserver with climb animator C++ plugin..."
    pkill -f "gzserver" || true
    stdbuf -oL -eL gzserver --verbose /mnt/c/Users/prath/Desktop/geckogrip-climber/gazebo/geckogrip.world &
    GZSERVER_PID=$!
else
    echo "[Launch Script] Connecting to remote Gazebo at ${REMOTE_GAZEBO_HOST}"
    # No local gzserver started; leave PID empty
    GZSERVER_PID=
fi

sleep 4

echo "[Launch Script] Starting gzclient GUI..."
stdbuf -oL -eL gzclient --verbose &
GZCLIENT_PID=$!

sleep 3

echo "[Launch Script] Starting ROS 2 launch (robot_state_publisher, rviz2, and stability bridge)..."
source /opt/ros/humble/setup.bash
stdbuf -oL -eL ros2 launch /mnt/c/Users/prath/Desktop/geckogrip-climber/gazebo/gazebo_rviz.launch.py &
LAUNCH_PID=$!

echo "[Launch Script] All components launched successfully!"
echo "[Launch Script] gzserver PID: $GZSERVER_PID"
echo "[Launch Script] gzclient PID: $GZCLIENT_PID"
echo "[Launch Script] ros2 launch PID: $LAUNCH_PID"

# Wait for all background processes to keep this session alive
wait
