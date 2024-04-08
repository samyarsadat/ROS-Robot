## How To Build MicroROS
Run `bash ~/pico_ws/uros_ws/build_uros.sh` as `urosdev` inside of the devcontainer to build the MicroROS library.

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
(Check whether `~/pico_ws/uros_ws/src/micro_ros_setup/install` exists.)<br>
<br>
When prompted to select a build kit by the CMake Tools extension, select `GCC [version] arm-none-eabi`.