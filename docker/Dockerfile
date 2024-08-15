FROM ros:noetic

RUN apt update -y && apt install -y --no-install-recommends curl lsb-release wget gnupg
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -

# setting environmental variables
ENV DEBIAN_FRONTEND noninteractive

RUN apt update -y && apt install -y --no-install-recommends tzdata
RUN apt update -y && apt install -y --no-install-recommends \
    ros-noetic-desktop-full \
    ros-noetic-tf \
    python3-osrf-pycommon \
    python3-catkin-tools \
    python3-rosdep \
    python3-rospkg \
    python3-pip \
    python3-tk \
    libxkbcommon-x11-0 \
    libcanberra-gtk* \
    build-essential \
    libxcb* \
    byobu \
    git \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /tmp/
RUN mkdir -p /catkin_ws/src && \
	/bin/bash -c "cd /catkin_ws/src && source /opt/ros/noetic/setup.bash; catkin_init_workspace" && \
	/bin/bash -c "cd /catkin_ws && source /opt/ros/noetic/setup.bash; catkin build" && \
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
RUN apt autoremove -y

COPY requirements.txt /tmp/
RUN pip install -r /tmp/requirements.txt

# Generates bash commands
WORKDIR /catkin_ws/src/
RUN echo "/usr/bin/python3 /catkin_ws/src/wros_tutorials/wrs/robot_sim/end_effectors/single_contact/suction/sgb30/sgb30.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /catkin_ws/src/wros_tutorials/wrs/robot_sim/end_effectors/single_contact/suction/sandmmbs/sdmbs.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /catkin_ws/src/wros_tutorials/wrs/robot_sim/end_effectors/gripper/robotiq140/robotiq140.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /catkin_ws/src/wros_tutorials/wrs/robot_sim/end_effectors/gripper/robotiq85/robotiq85.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /catkin_ws/src/wros_tutorials/wrs/robot_sim/end_effectors/gripper/robotiqhe/robotiqhe.py" >> ~/.bash_history
RUN echo "catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3" >> ~/.bash_history
RUN echo "byobu" >> ~/.bash_hist
RUN echo "rosservice call /plan_grasp" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_sgb30_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_suction_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiq140_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiq85_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiqhe_example.yaml" >> ~/.bash_history
