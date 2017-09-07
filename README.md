# bio-ik

Intro

## Installation and Setup

You will need ROS version Indigo or newer (wiki.ros.org).
The software was developed on Ubuntu Linux 16.04 LTS with ROS Kinetic,
but has also been tested on Ubuntu Linux 14.04 LTS with ROS Indigo.
Newer versions of ROS should work, but may need some adaptation.
See below for version specific instructions.

* Download the `bio_ik` package and unpack into your catkin workspace
* Run `catkin_make` to compile your workspace
  ```
  roscd
  cd src
  git clone https://gogs.crossmodal-learning.org/TAMS/bio_ik.git
  catkin_make
  ```
    
* Configure Moveit to use bio-ik as the kinematics solver
* Use Moveit or your own programs to move your robot.

## Basic Usage

For best compatibility with existing code, the bio-ik algorithm
is encapsulated as a kinematics plugin for Moveit
and can be used as a direct replacement of the default Orocos/KDL-based IK solver.
That is, given the name of an end-effector and a 6-DOF target pose,
bio-ik will search a valid robot joints configuration that reaches the given target.

In our tests (see below), bio-ik regularly outperformed the Orocos solver
both in terms of success rate and solution time.
The bio-ik algorithm can also be used for high-DOF system like robot snakes,
and it will automatically converge to the best approximate solutions
for low-DOF arms where some target poses are not reachable exactly.

While you can write the Moveit configuration files by hand, 
the easiest way is to run the Moveit setup assistant for your robot, 
and then to select bio-ik as the IK solver when configuring the end effectors.
Once configured, the solver can be called using the standard Moveit API
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
  an accuracy of 0.001 m (or smaller) and a timeout of 1 msec should be ok.
  More complex robots might need a longer timeout.
* Generate the moveit configuration files from the setup assistant.
  Of course, you can also edit the `config/kinematics.yaml` configuration
  file with your favorite text editor. 
  For example, a configuration for the PR2 robot might look like this:
  ```
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
  ```


* For a first test, run the Moveit-created demo. Once rviz is running,
  enable the motion planning plugin, then select one of the end effectors
  of you robot. Rviz should show an 6-D (position and orientation)
  interactive marker for the selected end-effector(s).
  Move the interactive marker and watch bio-ik calculating poses for your robot.
  
    roslaunch pr2_bioik_moveit demo.launch

* You are now ready to use bio-ik from your C/C++ and Python programs,
  using the standard Moveit! API. 
  A typical application could use the `move_group` node:

    blablubb


## Advanced Usage

For many robot applications, it is essential to specify more than just
a single end-effector pose. Typical examples include

* two-arm manipulation tasks on two-arm robots (e.g. Baxter)
* multi end-effector tasks with shared kinematic links, in particular
* grasping and manipulation tasks with multi-finger hands
* full-body motion on humanoid robots
* reaching tasks with additional constraints (e.g. shoulder position)
* incremental tool motions without robot arm configuration changes
* and many more

In bio-ik, such tasks are specified as a combination of multiple
individual *goals*.  
The algorithm then tries to find a robot configuration
that fulfills all given goals simultaneously by minimizing 
a quadratic error function built from the weighted individual goals.
While the current Moveit API does not support multiple-goals tasks directly,
it provides the KinematicQueryOptions class.
Therefore, bio-ik simply provides a set of predefined motion goals,
and a combination of the user-specified goals is passed via Moveit to the IK solver.
No API changes are required in Moveit, but using the IK solver now consists
passing the weighted goals via the KinematicQueryOptions.
The predefined goals include:

* `PoseGoal`: a full 6-DOF robot pose
* `PositionGoal`: a 3-DOF (x,y,z) position
* `OrientationGoal`: a 3-DOF orientation, encoded as a quaternion (qx,qy,qz,qw)
* `LookAtGoal`: a 3-DOF (x,y,z) position intended as a looking direction
   for a camera or robot head
* `JointGoal`: a set of joint angles, e.g. to specify a 
* `FunctionGoal`: an arbitrary function of the robot joint values,
   e.g. to model underactuated joints or mimic joints
* and several more



To solve a motion problem on your robot, the trick now is to construct
a suitable combination of individual goals. 



blblblbl

## How it works

The bio-ik solver is based on a memetic algorithm that combines 
traditional gradient-based search with a hybrid genetic
and particle-swarm optimization.
See (Starke,Hendrich,Zhang CEC-2017) for the basic idea and the details
of the evolutionary operators and (Starke,Hendrich,Krupke,Zhang IROS-2017)
for the description of the algorithm applied to many IK and manipulation tasks.

Internally, vectors of all robot joint values are used to encode
different intermediate solutions (the *genotype* of the genetic algorithm).
During the optimization, joint values are always checked against the
active lower and upper joint limits, so that only valid robot configurations
are generated.
To calculate the fitness of individuals, the cumulative error over all
given individual goals is calculated. An individual with no error encodes
a solution for the IK problem, while individuals with small error give
approximate solutions.
Individuals are sorted by their fitness, and gradient-based optimization
is tried on the best few configuration, resulting in fast convergence 
and good performance for many problems. 
If no solution is found from the gradient-based optimization,
new individuals are created by a set of mutation and recombination operators,
resulting in good search-space exploration.



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
