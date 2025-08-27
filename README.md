# audioROS for Indoor Position Simulation
This repository is built based on (https://github.com/LCAV/audioROS). Refer to the original repository for more detailed information about audioROS implementation.
This repository mainly focuses on simulating drone localization under controlled experimental conditions. 

## Installation
The repository was built for Ubuntu 20.04 (Focal Fossa) with ROS2 (Galactic) and Python 3.8.12.

To install, clone this repo including submodules by running
```
git clone --recurse-submodules https://github.com/Eureka-wong/audioROS.git
```

## Docker containers
There is a docker image which has already set the environment required for running the repository. To pull the docker image from docker hub, run:
```
docker pull aliyawang/audioros_docker:latest
```
To build the dependencies for ROS 2 visualization nodes (suggested), add the following command lines when running the docker container
```
--env="DISPLAY" --env="QT_X11_NO_MITSHM=1"--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"
```
For example:
```
docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1"--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" audioros_docker:latest
```
## Execute the program
### Execute for the first time
To execute the program on your devices, first go to the audioROS directory `cd audioROS`.
Then run the "run.sh" file to set all the ros 2 environments:
```
./run.sh
```
After that, you may run `xvfb-run -a ros2 launch audio_bringup live_demo.launch.py` to start the indoor positioning simulation experiment.
### Execute with a existing docker container
If you've already built a docker container following the aforementioned steps, try the following lines to activate the ROS 2 environment required for this project
```
source /opt/ros/galactic/setup.bash
source install/setup.bash
export PYTHONPATH="/audioROS/crazyflie-audio/python:$PYTHONPATH"
export PYTHONPATH="/audioROS/python:$PYTHONPATH"
export PYTHONPATH="/audioROS/python/utils:$PYTHONPATH"
export PYTHONPATH="/audioROS:$PYTHONPATH"
```
After that, you may run `xvfb-run -a ros2 launch audio_bringup live_demo.launch.py` to start the indoor positioning simulation experiment.

## Recompile the ROS 2 nodes
The ROS 2 nodes need to be compiled again every time after you have modified them. To compile the ROS 2 nodes, run
```
colcon build --packages-select package_you_modified
```
## Visualization results
Below are some visualization results 
1. Wall approaching simulation error analysis
 ![Wall approaching simulation](https://github.com/Eureka-wong/audioROS/blob/master/AUDIOROS/8.26/wall_approaching_err.png)

2. Drone circular motion trail visualization
* Motion trail-2D
![circular motion trail 2D](https://github.com/Eureka-wong/audioROS/blob/master/AUDIOROS/SLAM_VIS_2/slam_0080.png)
* Motion trail-3D
![circular motion trail 3D](https://github.com/Eureka-wong/audioROS/blob/master/AUDIOROS/8.26/drone_trajectory.png)
* Error analysis
![circular trail error](https://github.com/Eureka-wong/audioROS/blob/master/AUDIOROS/8.26/wall_detection_error_analysis-1.png)

## References
Please refer to the below publications for more information.

[1] [Blind as a Bat: Audible Echolocation on
Small Robots]( 10.1109/LRA.2022.3194669)