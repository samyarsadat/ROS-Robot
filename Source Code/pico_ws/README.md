## How To Build MicroROS
Run all of these commands inside of the devcontainer.

1. `cd uros_ws`
2. `colcon build --packages-select rrp_pico_coms`
3. `source install/local_setup.bash`
4. `ros2 run micro_ros_setup create_firmware_ws.sh generate_lib`
5. `ros2 run rrp_pico_coms create_fwws.sh`
6. `ros2 run micro_ros_setup build_firmware.sh $(pwd)/my_toolchain.cmake $(pwd)/my_colcon.meta`