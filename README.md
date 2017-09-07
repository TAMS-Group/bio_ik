# bio-ik

Intro

## Installation and Setup

You will need ROS version Indigo or newer (wiki.ros.org).
The software was developed on Ubuntu Linux 16.04 LTS with ROS Kinetic,
but has also been tested on Ubuntu Linux 14.04 LTS with ROS Indigo.
Newer versions of ROS should work, but may need some adaptation.
See below for version specific instructions.

* Download the `bio_ik` package and unpack into your catkin workspace.
* Run `catkin_make` to compile your workspace:
  ```
    roscd
    cd src
    git clone https://gogs.crossmodal-learning.org/TAMS/bio_ik.git
    roscd
    catkin_make
  ```

* Configure Moveit to use bio-ik as the kinematics solver.
* Use Moveit to plan and execute motions or use your own code
  together with `move_group` node to move your robot.


## Basic Usage

For ease of use and compatibility with existing code, 
the bio-ik algorithm is encapsulated as a Moveit kinematics plugin.
Therefore, bio-ik can be used as a direct replacement of
the default Orocos/KDL-based IK solver.
Given the name of an end-effector and a 6-DOF target pose,
bio-ik will search a valid robot joint configuration that reaches the given target.

In our tests (see below), both in terms of success rate and solution time,
bio-ik regularly outperformed the Orocos [1] solver
and is competitive with trac-ik [2].
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
  The setup assistant automatically searches for all available IK solver plugins
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

    # optional bio-ik configuration parameters
    #  center_joints_weight: 1
    #  minimal_displacement_weight: 1
    #  avoid_joint_limits_weight: 1
  ```


* For a first test, run the Moveit-created demo launch. Once rviz is running,
  enable the motion planning plugin, then select one of the end effectors
  of you robot. Rviz should show an 6-D (position and orientation)
  interactive marker for the selected end-effector(s).
  Move the interactive marker and watch bio-ik calculating poses for your robot.
  ```
    roslaunch pr2_bioik_moveit demo.launch
  ```

* You are now ready to use bio-ik from your C/C++ and Python programs,
  using the standard Moveit API. 
  A typical application could use the `move_group` node:

  ```
    robot_model_loader::RobotModelLoader robot_model_loader(robot, false);
    robot_model_loader.loadKinematicsSolvers(
        kinematics_plugin_loader::KinematicsPluginLoaderPtr(
          new kinematics_plugin_loader::KinematicsPluginLoader(solver, timeout, attempts)));

    auto robot_model = robot_model_loader.getModel();
    auto joint_model_group = robot_model->getJointModelGroup(group);
    auto tip_names = joint_model_group->getSolverInstance()->getTipFrames();

    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true; // optional

    robot_state::RobotState robot_state_fk(robot_model);
    robot_state::RobotState robot_state_ik(robot_model);

    bool ok = robot_state_ik.setFromIK(
                joint_model_group, // joints to be used for IK
                tip_transforms,    // multiple end-effector goal poses
                tip_names,         // names of the end-effector links
                attempts, timeout, // solver attempts and timeout
                moveit::core::GroupStateValidityCallbackFn(), 
                opts               // bio-ik cost function
              );
  ```

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

  ```
    robot_model_loader::RobotModelLoader robot_model_loader(robot, false);
    robot_model_loader.loadKinematicsSolvers(
        kinematics_plugin_loader::KinematicsPluginLoaderPtr(
          new kinematics_plugin_loader::KinematicsPluginLoader(solver, timeout, attempts)));

    auto robot_model = robot_model_loader.getModel();
    auto joint_model_group = robot_model->getJointModelGroup(group);
    auto tip_names = joint_model_group->getSolverInstance()->getTipFrames();

    kinematics::KinematicsQueryOptions opts;
    opts.return_approximate_solution = true; // optional

    robot_state::RobotState robot_state_fk(robot_model);
    robot_state::RobotState robot_state_ik(robot_model);

    bool ok = robot_state_ik.setFromIK(
                joint_model_group, // joints to be used for IK
                tip_transforms,    // multiple end-effector goal poses
                tip_names,         // names of the end-effector links
                attempts, timeout, // solver attempts and timeout
                moveit::core::GroupStateValidityCallbackFn(), 
                opts               // bio-ik cost function
              );
  ```



blblblbl

## How it works

The bio-ik solver is based on a memetic algorithm that combines 
traditional gradient-based search with a hybrid genetic
and particle-swarm optimization.
See [3] for the basic idea and the details of the evolutionary operators 
and [4] for the description of the algorithm applied to many IK and manipulation tasks.

Internally, vectors of all robot joint values are used to encode
different intermediate solutions (the *genotype* of the genetic algorithm).
During the optimization, joint values are always checked against the
active lower and upper joint limits, so that only valid robot configurations
are generated.

To calculate the fitness of individuals, the cumulative error over all
given individual goals is calculated. 
Any individual with zero error is an exact solution for the IK problem, 
while individuals with small error correspond to approximate solutions.

Individuals are sorted by their fitness, and gradient-based optimization
is tried on the best few configuration, resulting in fast convergence 
and good performance for many problems. 
If no solution is found from the gradient-based optimization,
new individuals are created by a set of mutation and recombination operators,
resulting in good search-space exploration.



## Example Performance data

To be written.


## Running the Self-Tests




## References

 1. Orocos Kinematics and Dynamics, http://www.orocos.org
 
 2. P. Beeson and B. Ames, *TRAC-IK: 
    An open-source library for improved solving of generic inverse kinematics*,
    Proceedings of the IEEE RAS Humanoids Conference, Seoul, Korea, November 2015.

 3. Sebastian Starke, Norman Hendrich, Jianwei Zhang,  *A Memetic 
    Evolutionary Algorithm for Real-Time Articulated Kinematic Motion*, 
    IEEE Intl. Congress on Evolutionary Computation (CEC-2017), p.2437-2479, June 4-8, 2017, 
    San Sebastian, Spain. 
    DOI: [10.1109/CEC.2017.7969605](http://doi.org/10.1109/CEC.2017.7969605)

 4. Sebastian Starke, Norman Hendrich, Dennis Krupke, Jianwei Zhang,  
    *Multi-Objective Evolutionary Optimisation for Inverse Kinematics 
     on Highly Articulated and Humanoid Robots*, 
    IEEE Intl. Conference on Intelligent Robots and Systems (IROS-2017), 
    September 24-28, 2017, Vancouver, Canada 


