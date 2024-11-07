#  The ROS robot project - Raspberry Pi Pico Communication Messages Package (CMake Config)
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2022-2024.
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <https: www.gnu.org/licenses/>.


cmake_minimum_required(VERSION 3.8)
project(rrp_pico_coms)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
                            "msg/MotorCtrlState.msg"
                            "msg/MiscSensorsA.msg"
                            "msg/MiscSensorsB.msg"
                            "msg/FalloffSensors.msg"
                            "msg/MicroSwSensors.msg"
                            "msg/UltrasonicSensors.msg"
                            "msg/FastOdometry.msg"
                            "srv/SetCameraLeds.srv"
                            "srv/SetPidTunings.srv"
                            "srv/RunCalibrationsA.srv"
                            "srv/GetConfigA.srv"
                            DEPENDENCIES builtin_interfaces geometry_msgs
                          )

ament_export_dependencies(rosidl_default_runtime)
ament_package()

install(
    PROGRAMS
        scripts/create_firmware_ws.sh
    DESTINATION lib/${PROJECT_NAME}
)