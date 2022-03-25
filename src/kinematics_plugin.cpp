/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Philipp Sebastian Ruppel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "bio_ik/kinematics_plugin.h"

using namespace bio_ik;

// implement BioIKKinematicsQueryOptions

namespace bio_ik
{

  std::mutex bioIKKinematicsQueryOptionsMutex;
  std::unordered_set<const void *> bioIKKinematicsQueryOptionsList;

  BioIKKinematicsQueryOptions::BioIKKinematicsQueryOptions()
      : replace(false), solution_fitness(0)
  {
    std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
    bioIKKinematicsQueryOptionsList.insert(this);
  }

  BioIKKinematicsQueryOptions::~BioIKKinematicsQueryOptions()
  {
    std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
    bioIKKinematicsQueryOptionsList.erase(this);
  }

  bool isBioIKKinematicsQueryOptions(const void *ptr)
  {
    std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
    return bioIKKinematicsQueryOptionsList.find(ptr) !=
           bioIKKinematicsQueryOptionsList.end();
  }

  const BioIKKinematicsQueryOptions *
  toBioIKKinematicsQueryOptions(const void *ptr)
  {
    if (isBioIKKinematicsQueryOptions(ptr))
      return (const BioIKKinematicsQueryOptions *)ptr;
    else
      return 0;
  }

} // namespace bio_ik

// BioIK Kinematics Plugin

namespace bio_ik_kinematics_plugin
{
  BioIKKinematicsPlugin::BioIKKinematicsPlugin() : enable_profiler(false)
  {
  }

  const std::vector<std::string> &BioIKKinematicsPlugin::getJointNames() const
  {
    //LOG_FNC();
    return joint_names;
  }

  const std::vector<std::string> &BioIKKinematicsPlugin::getLinkNames() const
  {
    //LOG_FNC();
    return link_names;
  }

  bool BioIKKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr &node,
                                         const moveit::core::RobotModel &robot_model,
                                         const std::string &group_name,
                                         const std::string &base_frame,
                                         const std::vector<std::string> &tip_frames,
                                         double search_discretization)
  {
    node_ = node;
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);
    joint_model_group_ = robot_model_->getJointModelGroup(group_name);

    if (!joint_model_group_)
    {
      LOG("failed to get joint model group");
      return false;
    }

    joint_names.clear();

    for (auto *joint_model : joint_model_group_->getJointModels())
      if (joint_model->getName() != base_frame_ &&
          joint_model->getType() != moveit::core::JointModel::UNKNOWN &&
          joint_model->getType() != moveit::core::JointModel::FIXED)
        joint_names.push_back(joint_model->getName());

    auto tips2 = tip_frames_;
    joint_model_group_->getEndEffectorTips(tips2);
    if (!tips2.empty())
      tip_frames_ = tips2;

    link_names = tip_frames_;

    // for(auto& n : joint_names) LOG("joint", n);
    // for(auto& n : link_names) LOG("link", n);

    // bool enable_profiler;
    lookupParam(node_, "profiler", enable_profiler, false);
    // if(enable_profiler) Profiler::start();

    robot_info = RobotInfo(robot_model_);

    ikparams.robot_model = robot_model_;
    ikparams.joint_model_group = joint_model_group_;

    // initialize parameters for IKParallel
    lookupParam(node_, "mode", ikparams.solver_class_name,
                std::string("bio2_memetic"));
    lookupParam(node_, "counter", ikparams.enable_counter, false);
    lookupParam(node_, "threads", ikparams.thread_count, 0);
    lookupParam(node_, "random_seed", ikparams.random_seed, static_cast<int>(std::random_device()()));

    // initialize parameters for Problem
    lookupParam(node_, "dpos", ikparams.dpos, DBL_MAX);
    lookupParam(node_, "drot", ikparams.drot, DBL_MAX);
    lookupParam(node_, "dtwist", ikparams.dtwist, 1e-5);

    // initialize parameters for ik_evolution_1
    lookupParam(node_, "no_wipeout", ikparams.opt_no_wipeout, false);
    lookupParam(node_, "population_size", ikparams.population_size, 8);
    lookupParam(node_, "elite_count", ikparams.elite_count, 4);
    lookupParam(node_, "linear_fitness", ikparams.linear_fitness, false);

    temp_state.reset(new moveit::core::RobotState(robot_model_));

    ik.reset(new IKParallel(ikparams));

    {

      BLOCKPROFILER("default ik goals");

      default_goals.clear();

      for (size_t i = 0; i < tip_frames_.size(); i++)
      {
        PoseGoal *goal = new PoseGoal();

        goal->setLinkName(tip_frames_[i]);

        // LOG_VAR(goal->link_name);

        double rotation_scale = 0.5;

        lookupParam(node_, "rotation_scale", rotation_scale, rotation_scale);

        bool position_only_ik = false;
        lookupParam(node_, "position_only_ik", position_only_ik, position_only_ik);
        if (position_only_ik)
          rotation_scale = 0;

        goal->setRotationScale(rotation_scale);

        default_goals.emplace_back(goal);
      }

      {
        double weight = 0;
        lookupParam(node_, "center_joints_weight", weight, weight);
        if (weight > 0.0)
        {
          auto *center_joints_goal = new bio_ik::CenterJointsGoal();
          center_joints_goal->setWeight(weight);
          default_goals.emplace_back(center_joints_goal);
        }
      }

      {
        double weight = 0;
        lookupParam(node_, "avoid_joint_limits_weight", weight, weight);
        if (weight > 0.0)
        {
          auto *avoid_joint_limits_goal = new bio_ik::AvoidJointLimitsGoal();
          avoid_joint_limits_goal->setWeight(weight);
          default_goals.emplace_back(avoid_joint_limits_goal);
        }
      }

      {
        double weight = 0;
        lookupParam(node_, "minimal_displacement_weight", weight, weight);
        if (weight > 0.0)
        {
          auto *minimal_displacement_goal =
              new bio_ik::MinimalDisplacementGoal();
          minimal_displacement_goal->setWeight(weight);
          default_goals.emplace_back(minimal_displacement_goal);
        }
      }
    }

    // LOG("init ready");

    return true;
  }

  bool BioIKKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                            const std::vector<double> &joint_angles,
                                            std::vector<geometry_msgs::msg::Pose> &poses) const
  {
    //LOG_FNC();
    return false;
  }

  bool BioIKKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                            const std::vector<double> &ik_seed_state,
                                            std::vector<double> &solution,
                                            moveit_msgs::msg::MoveItErrorCodes &error_code,
                                            const kinematics::KinematicsQueryOptions &options) const
  {
    //LOG_FNC();
    return false;
  }

  bool BioIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                             const std::vector<double> &ik_seed_state,
                                             double timeout,
                                             std::vector<double> &solution,
                                             moveit_msgs::msg::MoveItErrorCodes &error_code,
                                             const kinematics::KinematicsQueryOptions &options) const
  {
    const IKCallbackFn solution_callback = 0;
    std::vector<double> consistency_limits;

    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state,
                            timeout,
                            consistency_limits,
                            solution,
                            solution_callback,
                            error_code,
                            options);
  }

  bool BioIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               const std::vector<double> &consistency_limits,
                                               std::vector<double> &solution,
                                               moveit_msgs::msg::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
  {
    const IKCallbackFn solution_callback = 0;

    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state,
                            timeout,
                            consistency_limits,
                            solution,
                            solution_callback,
                            error_code,
                            options);
  }

  bool BioIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               std::vector<double> &solution,
                                               const IKCallbackFn &solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
  {
    std::vector<double> consistency_limits;
    
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state,
                            timeout,
                            consistency_limits,
                            solution,
                            solution_callback,
                            error_code,
                            options);
  }

  bool BioIKKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               const std::vector<double> &consistency_limits,
                                               std::vector<double> &solution,
                                               const IKCallbackFn &solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options) const
  {
    return searchPositionIK(std::vector<geometry_msgs::msg::Pose>{ik_pose},
                            ik_seed_state,
                            timeout,
                            consistency_limits,
                            solution,
                            solution_callback,
                            error_code,
                            options);
  }

  bool BioIKKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::msg::Pose> &ik_poses,
                                               const std::vector<double> &ik_seed_state,
                                               double timeout,
                                               const std::vector<double> &consistency_limits,
                                               std::vector<double> &solution,
                                               const IKCallbackFn &solution_callback,
                                               moveit_msgs::msg::MoveItErrorCodes &error_code,
                                               const kinematics::KinematicsQueryOptions &options,
                                               const moveit::core::RobotState *context_state) const
  {
    auto current_time = std::chrono::system_clock::now();
    auto duration_in_seconds = std::chrono::duration<double>(current_time.time_since_epoch());
    auto t0 = duration_in_seconds.count();

    if (enable_profiler)
      Profiler::start();

    auto *bio_ik_options = toBioIKKinematicsQueryOptions(&options);

    // //LOG_FNC();

    // FNPROFILER();

    // LOG(typeid(options).name());
    // LOG(((OptMod*)&options)->test);

    // get variable default positions / context state
    state.resize(robot_model_->getVariableCount());

    RCLCPP_INFO(node_->get_logger(), "robot_model");
    
    context_state = nullptr;
    if (context_state)
      for (size_t i = 0; i < robot_model_->getVariableCount(); i++)
        state[i] = context_state->getVariablePositions()[i];
    else
      robot_model_->getVariableDefaultPositions(state);

    RCLCPP_INFO(node_->get_logger(), "get variable count");

    // overwrite used variables with seed state
    solution = ik_seed_state;
    {
      int i = 0;
      for (auto &joint_name : getJointNames())
      {
        auto *joint_model = robot_model_->getJointModel(joint_name);
        if (!joint_model)
          continue;
        for (size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
          state.at(joint_model->getFirstVariableIndex() + vi) =
              solution.at(i++);
      }
    }

    RCLCPP_INFO(node_->get_logger(), "seed state");

    if (!bio_ik_options || !bio_ik_options->replace)
    {
      // transform tips to baseframe
      tipFrames.clear();
      for (size_t i = 0; i < ik_poses.size(); i++)
      {
        Eigen::Isometry3d p, r;
        poseMsgToEigen(ik_poses[i], p);
        if (context_state)
        {
          r = context_state->getGlobalLinkTransform(getBaseFrame());
        }
        else
        {
          if (i == 0)
            temp_state->setToDefaultValues();
          r = temp_state->getGlobalLinkTransform(getBaseFrame());
        }
        tipFrames.emplace_back(r * p);
      }
    }

    // init ik

    problem.timeout = t0 + timeout;
    problem.initial_guess = state;

    // for(auto& v : state) LOG("var", &v - &state.front(), v);

    // problem.tip_objectives = tipFrames;

    /*for(size_t i = 0; i < problem.goals.size(); i++)
    {
        problem.goals[i].frame = tipFrames[i];
    }*/

    // LOG("---");

    /*{
        BLOCKPROFILER("ik goals");
        std::vector<std::unique_ptr<Goal>> goals;
        for(size_t i = 0; i < tip_frames_.size(); i++)
        {
            //if(rand() % 2) break;
            PoseGoal* goal = new PoseGoal();
            goal->link_name = tip_frames_[i];
            goal->position = tipFrames[i].pos;
            goal->orientation = tipFrames[i].rot;
            goals.emplace_back(goal);
            //if(rand() % 20) break;
        }
        //std::random_shuffle(goals.begin(), goals.end());
        //LOG_VAR(goals.size());
        setRequestGoals(problem, goals, ikparams);
    }*/

    {

      if (!bio_ik_options || !bio_ik_options->replace)
      {
        for (size_t i = 0; i < tip_frames_.size(); i++)
        {
          auto *goal = (PoseGoal *)default_goals[i].get();
          goal->setPosition(tipFrames[i].pos);
          goal->setOrientation(tipFrames[i].rot);
        }
      }

      all_goals.clear();

      if (!bio_ik_options || !bio_ik_options->replace)
        for (auto &goal : default_goals)
          all_goals.push_back(goal.get());

      if (bio_ik_options)
        for (auto &goal : bio_ik_options->goals)
          all_goals.push_back(goal.get());

      {
        BLOCKPROFILER("problem init");
        problem.initialize(ikparams.robot_model, ikparams.joint_model_group,
                           ikparams, all_goals, bio_ik_options);
        // problem.setGoals(default_goals, ikparams);
      }
    }

    {
      BLOCKPROFILER("ik init");
      ik->initialize(problem);
    }

    // run ik solver
    {
      BLOCKPROFILER("ik_solve");
      ik->solve();
    }

    // get solution
    state = ik->getSolution();

    // wrap angles
    for (auto ivar : problem.active_variables)
    {
      auto v = state[ivar];
      if (robot_info.isRevolute(ivar) &&
          robot_model_->getMimicJointModels().empty())
      {
        auto r = problem.initial_guess[ivar];
        auto lo = robot_info.getMin(ivar);
        auto hi = robot_info.getMax(ivar);

        // move close to initial guess
        if (r < v - M_PI || r > v + M_PI)
        {
          v -= r;
          v /= (2 * M_PI);
          v += 0.5;
          v -= std::floor(v);
          v -= 0.5;
          v *= (2 * M_PI);
          v += r;
        }

        // wrap at joint limits
        if (v > hi)
          v -= std::ceil(std::max(0.0, v - hi) / (2 * M_PI)) * (2 * M_PI);
        if (v < lo)
          v += std::ceil(std::max(0.0, lo - v) / (2 * M_PI)) * (2 * M_PI);

        // clamp at edges
        if (v < lo)
          v = lo;
        if (v > hi)
          v = hi;
      }
      state[ivar] = v;
    }

    // wrap angles
    robot_model_->enforcePositionBounds(state.data());

    // map result to jointgroup variables
    {
      solution.clear();
      for (auto &joint_name : getJointNames())
      {
        auto *joint_model = robot_model_->getJointModel(joint_name);
        if (!joint_model)
          continue;
        for (size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
          solution.push_back(
              state.at(joint_model->getFirstVariableIndex() + vi));
      }
    }

    // set solution fitness
    if (bio_ik_options)
    {
      bio_ik_options->solution_fitness = ik->getSolutionFitness();
    }

    // return an error if an accurate solution was requested, but no accurate
    // solution was found
    if (!ik->getSuccess() && !options.return_approximate_solution)
    {
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    // callback?
    if (!solution_callback.empty())
    {
      // run callback
      solution_callback(ik_poses.front(), solution, error_code);

      // return success if callback has accepted the solution
      return error_code.val == error_code.SUCCESS;
    }
    else
    {
      // return success
      error_code.val = error_code.SUCCESS;
      return true;
    }
  }

  // bool BioIKKinematicsPlugin::supportsGroup(const moveit::core::JointModelGroup *jmg,
  //                                           std::string *error_text_out = 0) const
  // {
  //   //LOG_FNC();
  //   // LOG_VAR(jmg->getName());
  //   return true;
  // }
} // namespace bio_ik_kinematics_plugin

// register plugin

#undef LOG
#undef ERROR
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(bio_ik_kinematics_plugin::BioIKKinematicsPlugin,
                       kinematics::KinematicsBase);
