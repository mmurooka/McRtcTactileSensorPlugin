# [McRtcTactileSensorPlugin](https://github.com/isri-aist/McRtcTactileSensorPlugin)
mc_rtc plugin to use tactile sensor in controller via ROS interface

[![CI](https://github.com/isri-aist/McRtcTactileSensorPlugin/actions/workflows/ci.yaml/badge.svg)](https://github.com/isri-aist/McRtcTactileSensorPlugin/actions/workflows/ci.yaml)
[![Documentation](https://img.shields.io/badge/doxygen-online-brightgreen?logo=read-the-docs&style=flat)](https://isri-aist.github.io/McRtcTactileSensorPlugin/)
[![LICENSE](https://img.shields.io/github/license/isri-aist/McRtcTactileSensorPlugin)](https://github.com/isri-aist/McRtcTactileSensorPlugin/blob/master/LICENSE)

## Install

### Requirements
- Compiler supporting C++17
- Tested on `Ubuntu 20.04 / ROS Noetic`

### Dependencies
- [mc_rtc](https://jrl-umi3218.github.io/mc_rtc)
- [isri-aist/MujocoTactileSensorPlugin](https://github.com/isri-aist/MujocoTactileSensorPlugin) (Install as a ROS package)
- ros-${ROS_DISTRO}-eigen-conversions

### Installation procedure
```bash
# Load the ROS setup file so that the `mujoco_tactile_sensor_plugin` package can be found
$ mkdir ${HOME}/src && cd ${HOME}/src
$ git clone git@github.com:isri-aist/McRtcTactileSensorPlugin.git --recursive
$ cd McRtcTactileSensorPlugin
$ mkdir build && cd build
$ cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
$ make
$ make install
```
