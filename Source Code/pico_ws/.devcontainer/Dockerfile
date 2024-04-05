#  The ROS robot project - MicroROS Library Build Workspace Dockerfile
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
#  https://micro.ros.org/docs/tutorials/core/first_application_linux/
#  https://circuitdigest.com/microcontroller-projects/how-to-program-raspberry-pi-pico-using-c
#  https://github.com/BrunoB81HK/ros2-workspace-devcontainer-template

FROM ros:humble


# Install git, pip and curl
RUN apt-get update \
    && apt-get install -y git python3-pip curl \
    && rm -rf /var/lib/apt/lists/* && apt-get autoremove && apt-get autoclean

# Create non-root user
ARG USERNAME=urosdev
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

# Pico setup
COPY install_pico.sh /pico_setup.sh
RUN export NONROOT_USERNAME=${USERNAME} \
    && bash /pico_setup.sh \
    && rm -rf /var/lib/apt/lists/* && apt-get autoremove && apt-get autoclean

# Download MicroROS tools (this is done here to save some time when running uros_init.sh)
USER ${USERNAME}
RUN cd /home/$USERNAME && mkdir uros_tools_temp \
    && git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git uros_tools_temp

# Copy configuration files
USER ${USERNAME}
RUN echo 'source /opt/ros/'$ROS_DISTRO'/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /home/'$USERNAME'/pico_ws/uros_ws/src/micro_ros_setup/install/local_setup.bash' >> /home/$USERNAME/.bashrc

# Environment setup
COPY entrypoint.sh /entrypoint.sh
COPY uros_init.sh /uros_init.sh
USER root
RUN chmod +x /entrypoint.sh && chmod +x /uros_init.sh
ENTRYPOINT ["/entrypoint.sh"]
USER ${USERNAME}
CMD ["/uros_init.sh"]