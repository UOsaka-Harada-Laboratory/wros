# wros

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
![repo size](https://img.shields.io/github/repo-size/UOsaka-Harada-Laboratory/wros)

- ROS Noetic node examples with robot motion planners implemented in [WRS](https://github.com/wanweiwei07/wrs).

# Features

- Docker environment for ROS Noetic packages
- ROS node examples with [grasp planners (Wan et al., IEEE TRO 2021)](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9170578)

# Dependency (tested as a host machine)

- [Ubuntu 22.04 PC](https://ubuntu.com/certified/laptops?q=&limit=20&vendor=Dell&vendor=Lenovo&vendor=HP&release=22.04+LTS)
  - NVIDIA GeForce RTX 3070
  - NVIDIA Driver 470.256.02
  - Docker 26.1.1
  - Docker Compose 2.27.0
  - NVIDIA Docker 2.13.0

# Installation
```bash
git clone git@github.com:UOsaka-Harada-Laboratory/wros.git --recursive --depth 1 && cd wros && COMPOSE_DOCKER_CLI_BUILD=1 DOCKER_BUILDKIT=1 docker compose build --no-cache --parallel 
```

# Usage
1. Build and run the docker environment
- Create and start docker containers in the initially opened terminal
    ```bash
    docker compose up
    ```
 - Execute the container in another terminal
    ```bash
    xhost + && docker exec -it wros_noetic_container bash
    ```
2. Change planning parameters in wros_tutorials/config/XXX.yaml  
3. Build program files with the revised yaml
    ```bash
    cd /catkin_ws && catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 && source devel/setup.bash
    ```
4. Run a planning process in the container  
- Use byobu to easily command several commands  
    ```bash
    byobu
    ```
     - First command & F2 to create a new window & Second command ...
     - Ctrl + F6 to close the selected window
 - Run the grasp planner
    ```bash
    roslaunch wros_tutorials plan_grasp.launch config:=XXX.yaml
    ```
 - Call the planning service
    ```bash
    rosservice call /plan_grasp
    ```

#### Please refer to [wiki page](https://github.com/UOsaka-Harada-Laboratory/wros/wiki/Usage-examples) for usage examples.

# Contributors

We always welcome collaborators!

# Author

[Takuya Kiyokawa](https://takuya-ki.github.io/)  
[Weiwei Wan](https://wanweiwei07.github.io/)  
[Keisuke Koyama](https://kk-hs-sa.website/)  
[Kensuke Harada](https://www.roboticmanipulation.org/members2/kensuke-harada/)  

## License

This software is released under the MIT License, see [LICENSE](./LICENSE).
