FROM ros:noetic

# Remove any existing references to snapshots.ros.org
RUN rm -f /etc/apt/sources.list.d/*ros* && \
    sed -i '/snapshots.ros.org/d' /etc/apt/sources.list

# Add packages.ros.org for Noetic
RUN apt update && apt install -y --no-install-recommends \
    curl lsb-release wget gnupg \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - \
 && echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros1.list \
 && apt update

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
RUN echo "/usr/bin/python3 /catkin_ws/src/wros_tutorials/wrs/robot_sim/end_effectors/gripper/robotiq140/robotiq140.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /catkin_ws/src/wros_tutorials/wrs/robot_sim/end_effectors/gripper/robotiq85/robotiq85.py" >> ~/.bash_history
RUN echo "/usr/bin/python3 /catkin_ws/src/wros_tutorials/wrs/robot_sim/end_effectors/gripper/robotiqhe/robotiqhe.py" >> ~/.bash_history
RUN echo "catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiq140_single_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiq85_single_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiqhe_single_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiq140_obstcl_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiq85_obstcl_example.yaml" >> ~/.bash_history
RUN echo "roslaunch wros_tutorials plan_grasp.launch config:=planner_params_robotiqhe_obstcl_example.yaml" >> ~/.bash_history
RUN echo "rosservice call /plan_grasp" >> ~/.bash_history
RUN echo "byobu" >> ~/.bash_hist