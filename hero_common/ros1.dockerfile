FROM ros:noetic

RUN apt-get update && apt-get install -y wget gnupg lsb-release && \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    wget -O - https://packages.osrfoundation.org/gazebo.key | apt-key add - && \
    apt-get update

ARG DEBIAN_FRONTEND=noninteractive

# install ros package
RUN apt-get update && apt-get install -y \
      git \
      nano \
      byobu \
      qt5-default \
      python3-pyqt5 \
      python3-catkin-tools \
      ros-${ROS_DISTRO}-robot-state-publisher \
      ros-${ROS_DISTRO}-usb-cam \
      ros-${ROS_DISTRO}-xacro \
      ros-${ROS_DISTRO}-urdfdom-py \
      ros-${ROS_DISTRO}-rosserial \
      ros-${ROS_DISTRO}-rosserial-server \
      ros-${ROS_DISTRO}-urdf \
      ros-${ROS_DISTRO}-rqt \
      ros-${ROS_DISTRO}-rqt-plot \
      ros-${ROS_DISTRO}-rqt-ez-publisher \
      ros-${ROS_DISTRO}-rqt-rviz \
      ros-${ROS_DISTRO}-rqt-image-view \
      ros-${ROS_DISTRO}-map-server \
      ros-${ROS_DISTRO}-image-geometry \
      ros-${ROS_DISTRO}-image-proc \
      ros-${ROS_DISTRO}-camera-calibration \
      ros-${ROS_DISTRO}-teleop-twist-keyboard \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \
      ros-${ROS_DISTRO}-gazebo-ros-pkgs \ 
      ros-${ROS_DISTRO}-gazebo-plugins \
      gazebo11 libgazebo11-dev \
      && apt-get install -y --no-install-recommends \
    xfce4-session xfce4-panel xfce4-terminal xfce4-settings thunar \
        xfwm4 xfdesktop4 \
        tigervnc-standalone-server tigervnc-common \
        novnc websockify net-tools \
        x11-xserver-utils dbus-x11 xdg-utils x11-apps \
      && rm -rf /var/lib/apt/lists/*

# Booststrap workspace.
ENV CATKIN_DIR=/catkin_ws
RUN mkdir -p $CATKIN_DIR/src \
    && cd $CATKIN_DIR \
    && /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && catkin build \
    && echo 'source /opt/ros/${ROS_DISTRO}/setup.bash' >> /root/.bashrc \
    && echo 'source ${CATKIN_DIR}/devel/setup.bash' >> /root/.bashrc"

WORKDIR $CATKIN_DIR

# Re-add your clones (cleaned '\' lines)
RUN bash -lc '\
  cd $CATKIN_DIR/src && \
  git clone --depth 1 --branch noetic-devel https://github.com/rezeck/rosserial.git && \
  cd $CATKIN_DIR && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build \
'

# AprilTag
RUN bash -lc '\
  cd $CATKIN_DIR/src && \
  git clone https://github.com/AprilRobotics/apriltag.git && \
  git clone https://github.com/AprilRobotics/apriltag_ros.git && \
  cd $CATKIN_DIR && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build \
'

# HeRo
RUN bash -lc '\
  cd $CATKIN_DIR/src && \
  git clone --depth 1 --branch noetic-devel https://github.com/verlab/hero_common.git && \
  cd $CATKIN_DIR && source /opt/ros/${ROS_DISTRO}/setup.bash && catkin build \
'

# --- VNC/NoVNC startup ---
RUN mkdir -p /root/.vnc && \
    echo 'ros' | vncpasswd -f > /root/.vnc/passwd && \
    chmod 600 /root/.vnc/passwd && \
    printf '#!/bin/sh\nunset SESSION_MANAGER\nunset DBUS_SESSION_BUS_ADDRESS\nstartxfce4 &\n' > /root/.vnc/xstartup && \
    chmod +x /root/.vnc/xstartup && \
    printf "#!/usr/bin/env bash\n\
vncserver -kill :1 >/dev/null 2>&1 || true\n\
rm -rf /tmp/.X1-lock /tmp/.X11-unix/X1\n\
vncserver :1 -geometry 1600x900 -depth 24\n\
websockify --web=/usr/share/novnc 0.0.0.0:8080 localhost:5901\n" > /usr/local/bin/start-vnc && \
    chmod +x /usr/local/bin/start-vnc

ENV DISPLAY=:1
ENV QT_X11_NO_MITSHM=1
WORKDIR /root
