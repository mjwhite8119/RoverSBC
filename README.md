# RoverSBC Package

## ROS2 Install

    sudo apt-key adv --fetch-keys https://raw.githubusercontent.com/ros/rosdistro/master/ros.key

    sudo apt-add-repository http://packages.ros.org/ros2/ubuntu

    sudo apt install ros-foxy-ros-base

    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc 

    sudo apt install python3-argcomplete

    sudo apt install ros-foxy-example-interfaces

    sudo apt install ros-foxy-xacro

## Create a Workspace

    mkdir -p dev_ws/src
    cd dev_ws/src
    git clone https://github.com/ros2/demos.git

    sudo apt install python3-colcon-common-extensions

    git clone https://github.com/ros2/examples src/examples -b foxy

    colcon build --symlink-install

    source install/setup.bash

    



