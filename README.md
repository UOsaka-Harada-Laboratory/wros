# wros

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/Osaka-University-Harada-Laboratory/onrobot)

- ROS Noetic node examples with robot motion planners implemented in [WRS](https://github.com/wanweiwei07/wrs).

# Features

- Docker environment for ROS Noetic packages
- ROS node examples with a [grasp planner (Wan et al., IROS'17)](https://ieeexplore.ieee.org/abstract/document/8206011)

# Dependency

- [Ubuntu 20.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=20.04+LTS)
  - [ROS Noetic (Python3)](https://wiki.ros.org/noetic/Installation/Ubuntu)
  - docker 20.10.12
  - docker-compose 1.29.2
  - nvidia-docker2 2.8.0-1

# Installation
```bash
git clone git@github.com:Osaka-University-Harada-Laboratory/wros.git --recursive --depth 1
sudo apt install byobu -y
cd wros
COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel 
```

# Usage
```bash
docker compose up
```
```bash
xhost +
docker exec -it wros_noetic_container bash
byobu
```

## Robotiq Hand-E
```bash
roslaunch wros_tutorials plan_grasp.launch gripper_name:=robotiqhe object_mesh_path:=/catkin_ws/src/wros_tutorials/wrs/0000_examples/objects/tubebig.stl
rosservice call /plan_grasp
```
<img src=image/robotiqhe.gif width=720>  

## Robotiq 2F-85
```bash
roslaunch wros_tutorials plan_grasp.launch gripper_name:=robotiq85 object_mesh_path:=/catkin_ws/src/wros_tutorials/wrs/0000_examples/objects/bunnysim.stl
rosservice call /plan_grasp
```
<img src=image/robotiq85.gif width=720>  

## Robotiq 2F-140
```bash
roslaunch wros_tutorials plan_grasp.launch gripper_name:=robotiq140 object_mesh_path:=/catkin_ws/src/wros_tutorials/wrs/0000_examples/objects/milkcarton.stl
rosservice call /plan_grasp
```
<img src=image/robotiq140.gif width=720>  

## Suction gripper
```bash
roslaunch wros_tutorials plan_grasp.launch gripper_name:=suction object_mesh_path:=/catkin_ws/src/wros_tutorials/wrs/pyhiro/suction/objects/sandpart2.stl
rosservice call /plan_grasp
```
<img src=image/suction.gif width=720>  

## CONVUM baroon hand SGB30
```bash
roslaunch wros_tutorials plan_grasp.launch gripper_name:=sgb30 object_mesh_path:=/catkin_ws/src/wros_tutorials/wrs/pyhiro/suction/objects/ttube.stl
rosservice call /plan_grasp
```
<img src=image/sgb30.gif width=720>  

# Contributors


# Author

[Takuya Kiyokawa](https://takuya-ki.github.io/)  
[Weiwei Wan](https://wanweiwei07.github.io/)  
[Keisuke Koyama](https://kk-hs-sa.website/)  
[Kensuke Harada](https://www.roboticmanipulation.org/members2/kensuke-harada/)  
