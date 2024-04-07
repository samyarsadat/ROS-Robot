## How To Build MicroROS
Run all of these commands inside of the devcontainer.

1. `cd uros_ws`
2. `colcon build --packages-select rrp_pico_coms`
3. `source install/local_setup.bash`
4. `ros2 run micro_ros_setup create_firmware_ws.sh generate_lib`
5. `ros2 run rrp_pico_coms create_fwws.sh`
6. `ros2 run micro_ros_setup build_firmware.sh $(pwd)/my_toolchain.cmake $(pwd)/my_colcon.meta`

<br>

## Pico Firmware Pre-built Binaries
If you don't want to build MicroROS and the Pico firmware from source, pre-built `.uf2` binaries can be found in the `build/src` folder.

<br>

## Note On Devcontainer
When re-building the devcontainer, the re-build might fail the first time.<br>
In this case, simply retry the build a second time.<br>
<br>
You might have to wait (even after the devcontainer has started) for the MicroROS tools to install.<br>
Reload the devcontainer after you have verified that the MicroROS tools have installed successfully.<br>
(Check whether `~/pico_ws/uros_ws/src/micro_ros_setup/install` exists.)