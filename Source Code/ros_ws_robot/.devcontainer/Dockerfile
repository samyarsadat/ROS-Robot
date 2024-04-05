#  The ROS robot project - ROS2 Development Workspace (Robot) Dockerfile
#  Copyright 2024 Samyar Sadat Akhavi
#  Prepared by Samyar Sadat Akhavi, 2024.
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

FROM ros:humble


# Install git & pip
RUN apt-get update \
    && apt-get install -y git python3-pip \
    && rm -rf /var/lib/apt/lists/* && apt-get autoremove && apt-get autoclean

# Create non-root user
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Install and configure SUDO
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
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