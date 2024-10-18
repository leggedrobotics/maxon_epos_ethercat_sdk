# SDO Example

This is meant as a simple example it uses only SDO communication and was therefor only used with the profiled velocity /
position mode.

## Setup

Install ROS1 (checkout the ament_package branch for ros2, this example not supported).
Install yaml-cpp and vcs tools

```
 sudo apt install python3-vcstool\
 python3-colcon-common-extensions\
 libyaml-cpp-dev
```

Create a ros workspace and clone this repo and pull in all the deps with vcs tools.

* setup workspace `mkdir -p maxon_test_ws/src`
* clone this repo into `src`
* in the workspace import all required repos with:

```
vcs import < src/maxon_epos_thercat_sdk/maxon_sdo_only_example/example_dependencies.repos --recursive
```

* build the examples with: `catkin build maxon_epos_ethercat_sdk_example`

# Launch

Source your workspace and launch the ros launchfile:

```
roslaunch maxon_sdo_only_example maxon_sdo_only_example.launch
```

Note: you have to run as root, or run the give_elevate_permission.sh shell script.

## Building
Requires gcc ≥ 7.5 (default for Ubuntu ≥ 18.04).
