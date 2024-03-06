# The ROS Robot Project
# MicroROS Library Build Workspace Dockerfile.
# Prepared by Samyar Sadat Akhavi, 2024.
# 
# Bits of code taken from:
# https://github.com/joshnewans/dockerfile_example/blob/main/Dockerfile
# https://micro.ros.org/docs/tutorials/core/first_application_linux/
# https://circuitdigest.com/microcontroller-projects/how-to-program-raspberry-pi-pico-using-c

FROM ros:humble


# Install git & pip
RUN apt-get update \
    && apt-get install -y git python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create non-root user
ARG USERNAME=microros_ws
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
    && rm -rf /var/lib/apt/lists/*

# Bashrc files
COPY bashrc /home/${USERNAME}/.bashrc
COPY bashrc ~/.bashrc

# Pico setup
COPY install_pico.sh /pico_setup.sh
RUN export NONROOT_USERNAME=${USERNAME} \
    && bash /pico_setup.sh \
    && rm -rf /var/lib/apt/lists/*

# Download and install the micro-ROS tools
USER ${USERNAME}
RUN mkdir -p /home/${USERNAME}/pico_ws/uros_ws/src && cd /home/${USERNAME}/pico_ws/uros_ws/src \
    && rm -rf micro_ros_setup \
    && mkdir micro_ros_setup \
    && git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git micro_ros_setup
USER root
RUN apt-get update
USER ${USERNAME}
RUN rosdep update
USER root
RUN cd /home/${USERNAME}/pico_ws/uros_ws \
    && rosdep install --from-paths src --ignore-src -y \
    && rm -rf /var/lib/apt/lists/*

# Environment setup
COPY init.sh /init.sh
CMD ["/bin/bash", "/init.sh"]