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
 *   * Neither the name of Willow Garage nor the names of its
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

#include <bio_ik/goal.h>

#include "forward_kinematics.h"
#include "ik_base.h"
#include "ik_parallel.h"
#include "problem.h"
#include "utils.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <pluginlib/class_list_macros.h>
#include <srdfdom/model.h>
#include <urdf/model.h>
#include <urdf_model/model.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>
//#include <moveit/common_planning_interface_objects/common_objects.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <atomic>
#include <mutex>
#include <random>
#include <tuple>
#include <type_traits>

#include <bio_ik/goal_types.h>

using namespace bio_ik;

// implement BioIKKinematicsQueryOptions

namespace bio_ik {

std::mutex bioIKKinematicsQueryOptionsMutex;
std::unordered_set<const void *> bioIKKinematicsQueryOptionsList;

BioIKKinematicsQueryOptions::BioIKKinematicsQueryOptions()
    : replace(false), solution_fitness(0) {
  std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
  bioIKKinematicsQueryOptionsList.insert(this);
}

BioIKKinematicsQueryOptions::~BioIKKinematicsQueryOptions() {
  std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
  bioIKKinematicsQueryOptionsList.erase(this);
}

bool isBioIKKinematicsQueryOptions(const void *ptr) {
  std::lock_guard<std::mutex> lock(bioIKKinematicsQueryOptionsMutex);
  return bioIKKinematicsQueryOptionsList.find(ptr) !=
         bioIKKinematicsQueryOptionsList.end();
}

const BioIKKinematicsQueryOptions *
toBioIKKinematicsQueryOptions(const void *ptr) {
  if (isBioIKKinematicsQueryOptions(ptr))
    return (const BioIKKinematicsQueryOptions *)ptr;
  else
    return 0;
}

} // namespace bio_ik

// BioIK Kinematics Plugin

namespace bio_ik_kinematics_plugin {

// Fallback for older MoveIt versions which don't support lookupParam yet
template <class T>
static void lookupParam(const std::string &param, T &val,
                        const T &default_val) {
  ros::NodeHandle nodeHandle("~");
  val = nodeHandle.param(param, default_val);
}

struct BioIKKinematicsPlugin : kinematics::KinematicsBase {
  std::vector<std::string> joint_names, link_names;
  moveit::core::RobotModelConstPtr robot_model;
  const moveit::core::JointModelGroup *joint_model_group;
  mutable std::unique_ptr<IKParallel> ik;
  mutable std::vector<double> state, temp;
  mutable std::unique_ptr<moveit::core::RobotState> temp_state;
  mutable std::vector<Frame> tipFrames;
  RobotInfo robot_info;
  bool enable_profiler;

  BioIKKinematicsPlugin() { enable_profiler = false; }

  virtual const std::vector<std::string> &getJointNames() const {
    LOG_FNC();
    return joint_names;
  }

  virtual const std::vector<std::string> &getLinkNames() const {
    LOG_FNC();
    return link_names;
  }

  virtual bool getPositionFK(const std::vector<std::string> &link_names,
                             const std::vector<double> &joint_angles,
                             std::vector<geometry_msgs::Pose> &poses) const {
    LOG_FNC();
    return false;
  }

  virtual bool getPositionIK(const geometry_msgs::Pose &ik_pose,
                             const std::vector<double> &ik_seed_state,
                             std::vector<double> &solution,
                             moveit_msgs::MoveItErrorCodes &error_code,
                             const kinematics::KinematicsQueryOptions &options =
                                 kinematics::KinematicsQueryOptions()) const {
    LOG_FNC();
    return false;
  }

  EigenSTL::vector_Isometry3d tip_reference_frames;

  mutable std::vector<std::unique_ptr<Goal>> default_goals;

  mutable std::vector<const bio_ik::Goal *> all_goals;

  IKParams ikparams;

  mutable Problem problem;

  static moveit::core::RobotModelConstPtr
  loadRobotModel(const std::string &robot_description) {
    static std::map<std::string, moveit::core::RobotModelConstPtr>
        robot_model_cache;
    static std::mutex cache_mutex;
    std::lock_guard<std::mutex> lock(cache_mutex);
    if (robot_model_cache.find(robot_description) == robot_model_cache.end()) {
      rdf_loader::RDFLoader rdf_loader(robot_description);
      auto srdf = rdf_loader.getSRDF();
      auto urdf_model = rdf_loader.getURDF();

      if (!urdf_model || !srdf) {
        LOG("URDF and SRDF must be loaded for kinematics solver to work.");
        return nullptr;
      }
      robot_model_cache[robot_description] = moveit::core::RobotModelConstPtr(
          new robot_model::RobotModel(urdf_model, srdf));
    }
    return robot_model_cache[robot_description];

    // return
    // moveit::planning_interface::getSharedRobotModel(robot_description);
  }

  bool load(const moveit::core::RobotModelConstPtr &model,
            std::string robot_description, std::string group_name) {
    LOG_FNC();

    // LOG_VAR(robot_description);
    // LOG_VAR(group_name);

    LOG("bio ik init", ros::this_node::getName());

    /*rdf_loader::RDFLoader rdf_loader(robot_description_);
    auto srdf = rdf_loader.getSRDF();
    auto urdf_model = rdf_loader.getURDF();

    if(!urdf_model || !srdf)
    {
        LOG("URDF and SRDF must be loaded for kinematics solver to work.");
        return false;
    }

    robot_model.reset(new robot_model::RobotModel(urdf_model, srdf));*/

    if (model) {
      this->robot_model = model;
    } else {
      this->robot_model = loadRobotModel(robot_description);
    }

    joint_model_group = robot_model->getJointModelGroup(group_name);
    if (!joint_model_group) {
      LOG("failed to get joint model group");
      return false;
    }

    joint_names.clear();

    for (auto *joint_model : joint_model_group->getJointModels())
      if (joint_model->getName() != base_frame_ &&
          joint_model->getType() != moveit::core::JointModel::UNKNOWN &&
          joint_model->getType() != moveit::core::JointModel::FIXED)
        joint_names.push_back(joint_model->getName());

    auto tips2 = tip_frames_;
    joint_model_group->getEndEffectorTips(tips2);
    if (!tips2.empty())
      tip_frames_ = tips2;

    link_names = tip_frames_;

    // for(auto& n : joint_names) LOG("joint", n);
    // for(auto& n : link_names) LOG("link", n);

    // bool enable_profiler;
    lookupParam("profiler", enable_profiler, false);
    // if(enable_profiler) Profiler::start();

    robot_info = RobotInfo(robot_model);

    ikparams.robot_model = robot_model;
    ikparams.joint_model_group = joint_model_group;

    // initialize parameters for IKParallel
    lookupParam("mode", ikparams.solver_class_name,
                std::string("bio2_memetic"));
    lookupParam("counter", ikparams.enable_counter, false);
    lookupParam("threads", ikparams.thread_count, 0);

    // initialize parameters for Problem
    lookupParam("dpos", ikparams.dpos, DBL_MAX);
    lookupParam("drot", ikparams.drot, DBL_MAX);
    lookupParam("dtwist", ikparams.dtwist, 1e-5);

    // initialize parameters for ik_evolution_1
    lookupParam("no_wipeout", ikparams.opt_no_wipeout, false);
    lookupParam("population_size", ikparams.population_size, 8);
    lookupParam("elite_count", ikparams.elite_count, 4);
    lookupParam("linear_fitness", ikparams.linear_fitness, false);

    temp_state.reset(new moveit::core::RobotState(robot_model));

    ik.reset(new IKParallel(ikparams));

    {

      BLOCKPROFILER("default ik goals");

      default_goals.clear();

      for (size_t i = 0; i < tip_frames_.size(); i++) {
        PoseGoal *goal = new PoseGoal();

        goal->setLinkName(tip_frames_[i]);

        // LOG_VAR(goal->link_name);

        double rotation_scale = 0.5;

        lookupParam("rotation_scale", rotation_scale, rotation_scale);

        bool position_only_ik = false;
        lookupParam("position_only_ik", position_only_ik, position_only_ik);
        if (position_only_ik)
          rotation_scale = 0;

        goal->setRotationScale(rotation_scale);

        default_goals.emplace_back(goal);
      }

      {
        double weight = 0;
        lookupParam("center_joints_weight", weight, weight);
        if (weight > 0.0) {
          auto *avoid_joint_limits_goal = new bio_ik::CenterJointsGoal();
          avoid_joint_limits_goal->setWeight(weight);
          default_goals.emplace_back(avoid_joint_limits_goal);
        }
      }

      {
        double weight = 0;
        lookupParam("avoid_joint_limits_weight", weight, weight);
        if (weight > 0.0) {
          auto *avoid_joint_limits_goal = new bio_ik::AvoidJointLimitsGoal();
          avoid_joint_limits_goal->setWeight(weight);
          default_goals.emplace_back(avoid_joint_limits_goal);
        }
      }

      {
        double weight = 0;
        lookupParam("minimal_displacement_weight", weight, weight);
        if (weight > 0.0) {
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

  virtual bool initialize(const std::string &robot_description,
                          const std::string &group_name,
                          const std::string &base_frame,
                          const std::string &tip_frame,
                          double search_discretization) {
    LOG_FNC();
    std::vector<std::string> tip_frames;
    tip_frames.push_back(tip_frame);
    initialize(robot_description, group_name, base_frame, tip_frames,
               search_discretization);
    return true;
  }

  virtual bool initialize(const std::string &robot_description,
                          const std::string &group_name,
                          const std::string &base_frame,
                          const std::vector<std::string> &tip_frames,
                          double search_discretization) {
    LOG_FNC();
    setValues(robot_description, group_name, base_frame, tip_frames,
              search_discretization);
    load(moveit::core::RobotModelConstPtr(), robot_description, group_name);
    return true;
  }

  virtual bool initialize(const moveit::core::RobotModel &robot_model,
                          const std::string &group_name,
                          const std::string &base_frame,
                          const std::vector<std::string> &tip_frames,
                          double search_discretization) {
    LOG_FNC();
    setValues("", group_name, base_frame, tip_frames, search_discretization);
    load(moveit::core::RobotModelConstPtr(
             (moveit::core::RobotModel *)&robot_model,
             [](const moveit::core::RobotModel *robot_model) {}),
         "", group_name);
    return true;
  }

  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state, double timeout,
                   std::vector<double> &solution,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const {
    LOG_FNC();
    return searchPositionIK(std::vector<geometry_msgs::Pose>{ik_pose},
                            ik_seed_state, timeout, std::vector<double>(),
                            solution, IKCallbackFn(), error_code, options);
  }

  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state, double timeout,
                   const std::vector<double> &consistency_limits,
                   std::vector<double> &solution,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const {
    LOG_FNC();
    return searchPositionIK(std::vector<geometry_msgs::Pose>{ik_pose},
                            ik_seed_state, timeout, consistency_limits,
                            solution, IKCallbackFn(), error_code, options);
  }

  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state, double timeout,
                   std::vector<double> &solution,
                   const IKCallbackFn &solution_callback,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const {
    LOG_FNC();
    return searchPositionIK(std::vector<geometry_msgs::Pose>{ik_pose},
                            ik_seed_state, timeout, std::vector<double>(),
                            solution, solution_callback, error_code, options);
  }

  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state, double timeout,
                   const std::vector<double> &consistency_limits,
                   std::vector<double> &solution,
                   const IKCallbackFn &solution_callback,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions()) const {
    LOG_FNC();
    return searchPositionIK(std::vector<geometry_msgs::Pose>{ik_pose},
                            ik_seed_state, timeout, consistency_limits,
                            solution, solution_callback, error_code, options);
  }

  /*struct OptMod : kinematics::KinematicsQueryOptions
  {
      int test;
  };*/

  virtual bool
  searchPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                   const std::vector<double> &ik_seed_state, double timeout,
                   const std::vector<double> &consistency_limits,
                   std::vector<double> &solution,
                   const IKCallbackFn &solution_callback,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options =
                       kinematics::KinematicsQueryOptions(),
                   const moveit::core::RobotState *context_state = NULL) const {
    double t0 = ros::WallTime::now().toSec();

    // timeout = 0.1;

    // LOG("a");

    if (enable_profiler)
      Profiler::start();

    auto *bio_ik_options = toBioIKKinematicsQueryOptions(&options);

    LOG_FNC();

    FNPROFILER();

    // LOG(typeid(options).name());
    // LOG(((OptMod*)&options)->test);

    // get variable default positions / context state
    state.resize(robot_model->getVariableCount());
    if (context_state)
      for (size_t i = 0; i < robot_model->getVariableCount(); i++)
        state[i] = context_state->getVariablePositions()[i];
    else
      robot_model->getVariableDefaultPositions(state);

    // overwrite used variables with seed state
    solution = ik_seed_state;
    {
      int i = 0;
      for (auto &joint_name : getJointNames()) {
        auto *joint_model = robot_model->getJointModel(joint_name);
        if (!joint_model)
          continue;
        for (size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
          state.at(joint_model->getFirstVariableIndex() + vi) =
              solution.at(i++);
      }
    }

    if (!bio_ik_options || !bio_ik_options->replace) {
      // transform tips to baseframe
      tipFrames.clear();
      for (size_t i = 0; i < ik_poses.size(); i++) {
        Eigen::Isometry3d p, r;
        tf::poseMsgToEigen(ik_poses[i], p);
        if (context_state) {
          r = context_state->getGlobalLinkTransform(getBaseFrame());
        } else {
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

      if (!bio_ik_options || !bio_ik_options->replace) {
        for (size_t i = 0; i < tip_frames_.size(); i++) {
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
    for (auto ivar : problem.active_variables) {
      auto v = state[ivar];
      if (robot_info.isRevolute(ivar) &&
          robot_model->getMimicJointModels().empty()) {
        auto r = problem.initial_guess[ivar];
        auto lo = robot_info.getMin(ivar);
        auto hi = robot_info.getMax(ivar);

        // move close to initial guess
        if (r < v - M_PI || r > v + M_PI) {
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
    robot_model->enforcePositionBounds(state.data());

    // map result to jointgroup variables
    {
      solution.clear();
      for (auto &joint_name : getJointNames()) {
        auto *joint_model = robot_model->getJointModel(joint_name);
        if (!joint_model)
          continue;
        for (size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
          solution.push_back(
              state.at(joint_model->getFirstVariableIndex() + vi));
      }
    }

    // set solution fitness
    if (bio_ik_options) {
      bio_ik_options->solution_fitness = ik->getSolutionFitness();
    }

    // return an error if an accurate solution was requested, but no accurate
    // solution was found
    if (!ik->getSuccess() && !options.return_approximate_solution) {
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    // callback?
    if (!solution_callback.empty()) {
      // run callback
      solution_callback(ik_poses.front(), solution, error_code);

      // return success if callback has accepted the solution
      return error_code.val == error_code.SUCCESS;
    } else {
      // return success
      error_code.val = error_code.SUCCESS;
      return true;
    }
  }

  virtual bool supportsGroup(const moveit::core::JointModelGroup *jmg,
                             std::string *error_text_out = 0) const {
    LOG_FNC();
    // LOG_VAR(jmg->getName());
    return true;
  }
};
} // namespace bio_ik_kinematics_plugin

// register plugin

#undef LOG
#undef ERROR
PLUGINLIB_EXPORT_CLASS(bio_ik_kinematics_plugin::BioIKKinematicsPlugin,
                       kinematics::KinematicsBase);
