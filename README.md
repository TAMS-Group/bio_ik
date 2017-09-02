# bio-ik

Intro

## Installation and Setup

You will need [ROS], version Indigo or newer (wiki.ros.org).
The software was developd on Ubuntu Linux 16.04 LTS with ROS Kinetic,
but has also been tested on Ubuntu Linux 14.04 LTS with ROS Indigo.
Newer versions of ROS should work, but may need some adaptation.
See below for version specific instructions.

* Download the bio-ik package and unpack into your catkin workspace
* Run `catkin_make` to compile your workspace
* Configure Moveit to use bio-ik as the kinematics solver
* Use Moveit! or your own programs to move your robot.

## Basic Usage

In the basic form, bio-ik can be used as a kinematics plugin for Moveit!.
In particular, bio-ik can be used as as a direct replacement 
of the default Orocos/KDL solver.
In our tests, bio-ik regularly outperformed the Orocos solver
both in terms of solutation time and success rate.
Bio-ik can also be used for high-DOF systems (e.g. robot snakes)
and it will generate approximate solutions for low-DOF arms.

While you can write the configuration files by hand, 
the easiest way is to run the Moveit setup assistant for your robot, 
and then to select bio-ik as the IK solver.
Once configured, the solver can be called using the standard Moveit! API
or used interactively from rviz using the MotionPlanning GUI plugin.

* Make sure that you have a URDF (or xacro) model for your robot.

* Run the moveit setup assistant to create the Moveit configuration files.
  `rosrun moveit_setup_assistant moveit_setup_assistant`
  The setup assistant automatically finds all available IK solver plugins
  in your workspace. 
  Therefore, you can just select select bio-ik as the IK solver 
  from the drop-down list for every end effector and then configure 
  the kinematics parameters, namely the default position accuracy (meters)
  and the timeout (in seconds). For typical 6-DOF or 7-DOF arms,
  an accuracy of 0.001 m and a timeout of 1 msec should be ok.
  More complex robots might need a longer timeout.
  We also recommmend that you enable approximate solutions.

* Generate the moveit configuration files from the setup assistant.
  Of course, you can also edit the `config/kinematics.yaml` configuration
  file with your favorite text editor. 
  For example, a configuration for the PR2 robot might look like this:

`
# example kinematics.yaml for the PR2 robot
right_arm:
  # kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  # kinematics_solver_attempts: 1
  kinematics_solver: bio_ik_kinematics_plugin/BioIKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 1
left_arm:
  kinematics_solver: bio_ik_kinematics_plugin/BioIKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 1
all:
  kinematics_solver: bio_ik_kinematics_plugin/BioIKKinematicsPlugin
  kinematics_solver_search_resolution: 0.005
  kinematics_solver_timeout: 0.02
  kinematics_solver_attempts: 1
`


* For a first test, run the Moveit-created demo. Once rviz is running,
  select the motion planning plugin, then select one of the end effectors
  of you robot. Rviz should show an 6-D (position and orientation)
  interactive marker for the selected end-effector(s).
  Move the interactive marker and watch bio-ik calculating poses
  for your robot.
  `roslaunch pr2_bioik_moveit demo.launch`

* You are now ready to use bio-ik from your C/C++ and Python programs,
  using the standard Moveit! API. 
  A typical application could use the `move_group` node:




blblblb

## Advanced Usage

blblblbl

## Example Performance data


## Running the Self-Tests


A repository for ROS catkin packages related to the TAMS robot demonstrators 
in CML subprojects A4, B5, and Z3.
Most of the software is specific to the robots and sensors available 
at our TAMS lab, in particular the reording and human-robot interaction 
setup. 


  * Warning: 
     1. Finger initialization usually succeeds, but thumb controller
       initialization fails often (perhaps 30% of the time), 
       resulting in CAN bus data corruption and random hand configurations.
     2. If this happens, close the air-supply immediately,
     3. Cick `stop_controllers` quickly,
     4. Close the air-supply value and click `empty_hand` several times,
     5. Move the fingers (especially the thumb) back to a partly flexed
       position,
     6. Click `stop_robot`, then try to repeat the full start sequence.
  * Shutting down the hand:
     1. Stop the `hand_server`, 
     2. Click `stop_controllers`. 
     3. Close the air-supply valve, 
     4. Click  `empty_hand` several times until all muscles are empty.
     5. Click `stop_robot`
     6. Shut-down the control PC and power-off the hand.

* Kinect v2
  * Download, compile, and install `freenect2` for Ubuntu 14.04
  * Download, compile, and install `iai_kinect`
    Note that building will fail if OpenCV3 is installed
    due to problems in the iai_kinect stack (actually, compiling
    succeeds but the resulting nodes segfault on initialization,
    as they are partly linked against opencv2 and partly against
    opencv3. If necessary, de-install ros-indigo-opencv3.)
    
    `roslaunch kinect2_bridge kinect2_bridge.launch _reg_method:=opencl`

  * Please make sure that OpenCL or CUDA support is compiled,
    as reg_method:=cpu results in very poor performance.

* Cameras
  * Note that the gscam webcam driver is not included in the
    Ubuntu packages for ROS Indigo and up. Please check out
    the gscam repository from github into your catkin workspace
    and recompile:
    `https://github.com/ros-drivers/gscam.git`
  * Single-camera and stereo calibration should be performed
    using the standard `camera_calibration` package and tools.
    Note that stereo calibration may need the `--approximate=0.1'
    command line option, as multiple cameras driven by `gscam`
    are not perfectly synchronized.
  * Consult Eugen for precise calibration of the Phasespace X2
    system and his three-camera stereo-camera system.

* Leapmotion
  * coming soon.



## Usage and Howtos

See the README.md in the individual catkin packages for detals
about installation, calibration, and starting the individual
applications and demos.
