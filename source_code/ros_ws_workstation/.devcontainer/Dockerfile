#  The ROS robot project - ROS2 Development Workspace (Workstation) Dockerfile
#  Copyright 2024-2025 Samyar Sadat Akhavi
#  Prepared by Samyar Sadat Akhavi, 2024-2025.
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
#  
#  Bits of code taken from:
#  https://github.com/joshnewans/dockerfile_example/blob/main/Dockerfile
#  https://github.com/BrunoB81HK/ros2-workspace-devcontainer-template

FROM samyarsadat/custom_base_images:base-ros-jazzy

# Install pip
RUN apt-get update \
    && apt-get install -y python3-pip \
    && rm -rf /var/lib/apt/lists/* && apt-get autoremove && apt-get autoclean

# Copy configuration files
USER ${USERNAME}
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc

# Environment setup
COPY entrypoint.sh /entrypoint.sh
USER root
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]