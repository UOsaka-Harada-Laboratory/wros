FROM ros:noetic

RUN apt-get update && apt-get install -y --no-install-recommends curl lsb-release wget gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get install -y --no-install-recommends tzdata
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-tf \
    python3-osrf-pycommon \
    python3-catkin-tools \
    python3-rosdep \
    python3-rospkg \
    python3-pip \
    python3-tk \
    byobu \
    git \
    vim \
    build-essential \
    libxkbcommon-x11-0 \
    libcanberra-gtk* \
    libxcb*

RUN mkdir -p /tmp/
RUN mkdir -p /catkin_ws/src && \
	cd /catkin_ws/src && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin_init_workspace" && \
	cd /catkin_ws && \
	/bin/bash -c "source /opt/ros/noetic/setup.bash; catkin build" && \
	echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
	echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc && \
	echo "export ROS_PACKAGE_PATH=\${ROS_PACKAGE_PATH}:/catkin_ws" >> ~/.bashrc && \
	echo "export ROS_WORKSPACE=/catkin_ws" >> ~/.bashrc

RUN wget --no-check-certificate https://www.python.org/ftp/python/3.9.5/Python-3.9.5.tgz \
    && tar -xf Python-3.9.5.tgz \
    && cd Python-3.9.5 \
    && ./configure --enable-optimizations\
    && make \
    && make install
RUN apt-get autoremove -y

COPY requirements.txt /tmp/
RUN pip install -r /tmp/requirements.txt

# Generates bash commands
WORKDIR /catkin_ws/src/
RUN echo "catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3" >> ~/.bash_history
RUN echo "rosservice call /plan_grasp" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_sgb30_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_suction_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiq140_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiq85_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiqhe_example.yaml" >> ~/.bash_history
