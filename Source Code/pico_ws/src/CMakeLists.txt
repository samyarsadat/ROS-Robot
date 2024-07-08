#  The ROS robot project - Firmware CMake config (NOTE: THIS IS NOT THE TOP-LEVEL CMAKE FILE!)
#  Copyright 2022-2024 Samyar Sadat Akhavi
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


# Custom library subdirectories
add_subdirectory(lib)

# Pico A firmware
add_subdirectory(pico_a)

# Pico B firmware
add_subdirectory(pico_b)

# Multithread MicroROS test firmware
add_subdirectory(multithread_test)