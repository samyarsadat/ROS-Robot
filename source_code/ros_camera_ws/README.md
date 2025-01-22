## Building `camera_ros`
The `ros_ws_robot_infra` setup script ([`source_code/ros_ws_robot_infra/deployment/setup_robot.sh`](../ros_ws_robot_infra/deployment/setup_robot.bash)) 
will take care of building the `camera_ros` package, but in case you want to build it manually yourself:<br>
<br>
Fisrt run the `build_setup_camera_ros.sh` script and then the `build_camera_ros.sh` script, either on the
Raspberry Pi (Ubuntu 24.04, with ROS installed) or inside the `ros_ws_robot_infra` devcontainer.<br>
<br>
These scripts assume that ROS 2 Jazzy Jalisco has already been installed on the system.