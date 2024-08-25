#  The ROS robot project (Robot Driver ROS Wrapper Package) - Main file
#  Copyright 2024 Samyar Sadat Akhavi
#  Written by Samyar Sadat Akhavi, 2024.
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

import sys
import threading
import traceback
from time import sleep
from ros_robot_driver.ros_main import RosRobotDriverThread
from ros_robot_driver_wrapper.config import RosConfig, ProgramConfig
from ros_robot_driver_wrapper.ros_main import ros_executor_thread, get_ros_node
from ros_robot_driver_wrapper.ros_main import is_ros_node_initialized


def main():
    stop_wrapper_ros_thread = False
    wrapper_ros_thread = threading.Thread(target=ros_executor_thread, args=(lambda: stop_wrapper_ros_thread, ), name=RosConfig.THREAD_NAME)

    wrapper_ros_thread.start()
    RosRobotDriverThread.start_thread()

    try:
        while True:
            if not wrapper_ros_thread.is_alive():
                print("The driver wrapper thread has died. Terminating program.")
                RosRobotDriverThread.stop_ros_thread(True)
                sys.exit(1)

            if not RosRobotDriverThread.is_alive():
                if is_ros_node_initialized():
                    get_ros_node().get_logger().fatal("The robot driver thread has died. Terminating program.")

                stop_wrapper_ros_thread = True
                wrapper_ros_thread.join()
                sys.exit(1)

            sleep(ProgramConfig.THREADS_LIVELINESS_CHECK_INTERVAL_S)

    except KeyboardInterrupt:
        if is_ros_node_initialized():
            get_ros_node().get_logger().warn("Keyboard interrupt detected, terminating program.")

    except Exception:
        if is_ros_node_initialized():
            get_ros_node().get_logger().fatal(traceback.format_exc())
            get_ros_node().get_logger().fatal("Exception caught, terminating program.")

    finally:
        stop_wrapper_ros_thread = True
        RosRobotDriverThread.stop_thread(True)
        wrapper_ros_thread.join()
