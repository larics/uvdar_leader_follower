# MRS Summer School 2020

## Leader-follower operation without communication

In this task, we will explore a state-of-the-art system for coordinated motion of multiple robots without direct communication. The system consists of two parts - blinking UV markers, which are fitted to each robot, and onboard cameras capable of UV-only sensing. Using this unique approach, the robots are capable of determining the relative position of their neighbors without the need to exchange any information via a communication link. This allows the UAVs to operate in environments with dense electromagnetic interference, or coordinated motion of multiple robots with mutually incompatible localization systems (e.g. GPS and SLAM).

## Installation
1) The installation requires the open-source [MRS system] (https://github.com/ctu-mrs/mrs_uav_system), or if you prefer to install it separately, you will need the [uav_core](https://github.com/ctu-mrs/uav_core) and [simulation](https://github.com/ctu-mrs/simulation) repositories.
2) You will also need the [UVDAR core](https://github.com/ctu-mrs/uvdar) and the [UVDAR plugin](https://github.com/ctu-mrs/uvdar_gazebo_plugin) for the Gazebo simulator.
3) The [summer_school_supervisor](https://github.com/ctu-mrs/summer_school_supervisor) , which will be enforcing the rules of comeptition for this task.
4) Finally, you will need the [trajectory_loader](https://github.com/ctu-mrs/trajectory_loader). This package will help you load a trajectory for the leader UAV, which will come in handy for testing in simulations.

## Task overview

You are given a system consisting of two autonomous unmanned aerial vehicles (UAVs). Both vehicles are equipped with the MRS control pipeline, and the UVDAR mutual localization system.

One of the UAVs is designated as a formation leader. It will follow a smooth, pre-defined trajectory. This UAV is out of your control and you should not interfere with the onboard control software.

The second UAV is designated as a follower. This is the vehicle, that you will be directly interacting with.
> **__NOTE:__** Minor changes may still be made to the assignment.

Your task is to develop a control routine for the follower such that it keeps up with the leader for as long as possible. We have prepared some C++ code to get you started.
Your entry point to the system is the the `src/follower.cpp` file. It creates a library, which is periodically called by the supervisor node to provide new control commands to the UAV. The file already contains the following methods:
* initialize - called only once on startup. This method is suitable for loading parameters from a configuration file. This way, you can tweak the parameters without needing to compile the package with every change. The default configuration file is `config/follower.yaml`.
* dynamicReconfigureCallback - use is optional, recommended for fine tuning of system parameters on the fly. Allows you to tweak system parameters while the program is running using the [rqt_reconfigure](http://wiki.ros.org/rqt_reconfigure). The default dynamic configuration file is `cfg/Follower.cfg`
* recieveOdometry - called every time a new odometry message is recieved. This will provide you with the latest information on the state of the follower UAV (position, orientation, velocity...).
* recieveUvdar - called every time a new UVDAR message is recieved. This will provide you the estimate of position of the leader UAV, based on the markers visible by the onboard cameras.
* createReferencePoint - called periodically by the supervisor node. You are expected to provide the onboard controller a new reference point through this method.
* createSpeedCommand - called periodically by the supervisor node. Alternative to the reference point commands. Allows you to have a deeper level of control by using velocity commands instead of position reference.
* getCurrentEstimate - uses a simple Kalman filter to estimate the position and velocity of the leader UAV. Filtering the raw UVDAR data will allow you to smooth out the control commands. In return, the camera image will be less shaky and the estimation will be more accurate.

These methods will be called by the `summer_school_supervisor` node running onboard the follower UAV. Do not modify the supervisor node! You will not be able to use the customized code onboard real UAVs during the experiments.

## Simulation and a Reference solution
The provided code also includes a very crude solution. The code will take the latest known leader pose, add a desired offset (loaded from a config file), and set it as the reference point for the follower. You may try running this code by opening the `tmux_scripts/two_drones_uvdar` folder and running the script:
```bash
./start.sh
````
> **__NOTE:__** The script may require some modification in the spawn block (around line 23) to set an existing path to the calibration file on your computer. Make sure the filepath points to the `uvdar_core` package on your device.

## Implementation tips
You may notice that the reference solution does not produce a smooth control input for the follower UAV. Also, the following may break after a while. The jumps in follower motion are caused by multiple contributing factors:
  * The camera as well as the target do not stay completely still, even during hovering.
  * Aggressive control manoeuvres will also move the camera more, and make it harder to estimate the leader position accurately.
  * Setting a single reference point to the controller will cause the UAV to decelerate and stop moving, once the destination has been reached.

LET'S IMPROVE THE FOLLOWER CODE.
There are a few steps that may help you. It is not necessary to follow them. You may skip this section completely and craft a solution on your own.
  * The first course of action should be ensuring that the leader is always visible by the follower. Once you break the visual contact, you lose the leader position updates and cannot procceed.
  * Next, you will probably want to smooth out the motion of the follower. You can precompute multiple references and construct a trajectory for a few seconds into the future, utilize the SpeedTracker, employ the VelocityEstimator....
  * The leader trajectory gets progressively more difficult to follow. Therefore, you can easily assess the quality of your solution by measuring how long can the follower keep the pace.
  * Since in vision distance of object tends to be more difficult to estimate than its bearing, the position tracking works much better if the line of sight from camera is directly perpendicular to the direction of leader's motion. If the leader moves along the line of sight from the camera, retrieving the position becomes much more difficult.
