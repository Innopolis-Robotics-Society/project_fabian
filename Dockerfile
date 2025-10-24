# Base
FROM osrf/ros:humble-desktop-full
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Create user
ARG USERNAME=mobile
ARG USER_UID=1000
ARG USER_GID=${USER_UID}
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && apt-get update \
 && apt-get install -y sudo curl gnupg2 lsb-release net-tools python3-pip git build-essential \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# Extra ROS deps and tools
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-hardware-interface \
    ros-${ROS_DISTRO}-transmission-interface \
    ros-${ROS_DISTRO}-urdf \
    ros-${ROS_DISTRO}-urdfdom \
    ros-${ROS_DISTRO}-urdfdom-headers \
    ros-${ROS_DISTRO}-urdf-tutorial \
    ros-${ROS_DISTRO}-apriltag-ros \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-v4l2-camera \
    ros-${ROS_DISTRO}-camera-calibration \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-ros-gz \
    libcanberra-gtk-module libcanberra-gtk3-module \
    at-spi2-core x11-apps xauth \
    --fix-missing

# can-utils (если реально нужно)
RUN git clone https://github.com/linux-can/can-utils.git /tmp/can-utils \
 && make -C /tmp/can-utils && make -C /tmp/can-utils install \
 && rm -rf /tmp/can-utils

# Clean apt cache
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# rosdep init (as root)
RUN rosdep init || true && rosdep update

# ---- switch to user ----
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# ROS environment in shell
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc

# Pre-create workspace
RUN mkdir -p /home/${USERNAME}/ros2_ws/src

# Default command
CMD ["bash"]
