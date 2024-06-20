## How To Build MicroROS
Run `bash ~/pico_ws/libmicroros/build_uros.sh -f` as `nonroot` inside of the devcontainer to build the MicroROS library.

<br>

## Pico Firmware Pre-built Binaries
~~If you don't want to build MicroROS and the Pico firmware from source, pre-built `.uf2` binaries can be found in the `build/src` folder.~~<br>
Please take a look at the [`pico-build` workflow](https://github.com/samyarsadat/ROS-Robot/actions/workflows/pico-build.yml) artifact called 
`pico_binary_artifacts` for the latest binary builds.

#### Action Status
[![Build Pico Source Code](https://github.com/samyarsadat/ROS-Robot/actions/workflows/pico-build.yml/badge.svg)](https://github.com/samyarsadat/ROS-Robot/actions/workflows/pico-build.yml)<br>
[![CodeQL Analysis - Pico Source](https://github.com/samyarsadat/ROS-Robot/actions/workflows/pico-codeql.yml/badge.svg)](https://github.com/samyarsadat/ROS-Robot/actions/workflows/pico-codeql.yml)

<br>
<br>

**NOTE: This devcontainer and the following instructions are written with the assumption that the host machine running Docker runs Linux (specifically, Ubuntu)!
If you're using Windows (and can't dualboot Linux), please use the Windows Subsystem for Linux (WSL) to run Docker.**<br>


## Note On Devcontainer
When re-building the devcontainer, the re-build might fail the first time.<br>
In this case, simply retry the build a second time.<br>
<br>
You might have to wait (even after the devcontainer has started) for the MicroROS tools to install.<br>
Reload the devcontainer after you have verified that the MicroROS tools have installed successfully.<br>
(Check whether `~/pico_ws/libmicroros/src/micro_ros_setup/install` exists.)<br>
<br>
When prompted to select a build kit by the CMake Tools extension, select `GCC [version] arm-none-eabi`.

<br>

## Note On Using Debug Probes
If you're using a debug probe (such as the Raspberry Pi Debug Probe), make sure to create
a file named `60-openocd.rules` in `/etc/udev/rules.d` on the host machine (not the devcontainer)
and copy the contents of `https://github.com/raspberrypi/openocd/blob/rp2040-v0.12.0/contrib/60-openocd.rules`
into it. This is so that the probe can be accessed witout having to use `sudo`, which is needed when using
the Cortex-Debug extension in VSCode.<br>
<br>
Run `udevadm control --reload` (again, on the host machine) to make sure that the new rules take effect.

<br>

## Note On GUI Apps & Non-NVIDIA GPUs
If you're using an NVIDIA GPU, make sure to install the NVIDIA Container Toolkit (https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html).
If you aren't using an NVIDIA GPU, please remove the `"--gpus", "all",` line from `"runArgs"` in `devcontainer.json`.<br>
<br>
For GUI support, the devcontainer is currently configured for using X11. If you're using Wayland, you will have to change the
`"-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",` line from `"runArgs"` in `devcontainer.json`.
If you don't want GUI support at all, simply remove `"-v", "/tmp/.X11-unix:/tmp/.X11-unix:rw",` and `"--env=DISPLAY",`.

<br>

## Updating MicroROS Tools or the FreeRTOS Kernel
Discard all changes to the submodule repository, pull the latest version, and rebuild the devcontainer.