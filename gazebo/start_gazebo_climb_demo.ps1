$Root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$World = "/mnt/c/Users/prath/Desktop/geckogrip-climber/gazebo/geckogrip.world"
$GazeboDir = "/mnt/c/Users/prath/Desktop/geckogrip-climber/gazebo"

Write-Host "Killing old simulation, RViz and bridge processes..."
wsl.exe sh -lc "pkill -9 -f 'gz model' 2>/dev/null || true; pkill -9 -f 'gzserver' 2>/dev/null || true; pkill -9 -f 'gzclient' 2>/dev/null || true; pkill -9 -f 'gazebo' 2>/dev/null || true; pkill -9 -f 'gzmaster' 2>/dev/null || true; pkill -9 -f 'animate_climb.py' 2>/dev/null || true; pkill -9 -f 'gazebo_rviz_bridge.py' 2>/dev/null || true; pkill -9 -f 'ros2 launch.*gazebo_rviz.launch.py' 2>/dev/null || true; pkill -9 -f 'rviz2' 2>/dev/null || true; pkill -9 -f 'robot_state_publisher' 2>/dev/null || true"
wsl.exe sh -lc "pgrep -f '[g]zserver|[g]zclient|[g]zmaster|[r]viz2|[r]obot_state_publisher|[g]azebo_rviz_bridge.py|[g]azebo_rviz.launch.py' | xargs -r kill -9"
Start-Sleep -Seconds 1

Write-Host "Compiling GeckoGrip Climb Animator Plugin..."
wsl.exe bash -lc "cd $GazeboDir && g++ -std=c++17 -shared -fPIC geckogrip_climb_animator.cc -o libgeckogrip_climb_animator.so `$(pkg-config --cflags --libs gazebo)"

Write-Host "Starting Gazebo Server..."
Start-Process -FilePath wsl.exe -ArgumentList "sh -lc `"export GAZEBO_PLUGIN_PATH=${GazeboDir}:`$GAZEBO_PLUGIN_PATH; gzserver --verbose $World > /tmp/geckogrip_server.log 2>&1`"" -WindowStyle Hidden
Start-Sleep -Seconds 3

Write-Host "Starting Gazebo Client..."
Start-Process -FilePath wsl.exe -ArgumentList "sh -lc `"gzclient --verbose > /tmp/geckogrip_client.log 2>&1`""

Write-Host "Launching RViz2 and Gazebo-RViz Bridge..."
Start-Process -FilePath wsl.exe -ArgumentList "bash -lc `"source /opt/ros/humble/setup.bash && ros2 launch ${GazeboDir}/gazebo_rviz.launch.py > /tmp/geckogrip_rviz.log 2>&1`""

Write-Host "Gazebo and RViz GeckoGrip climb demo started successfully."
