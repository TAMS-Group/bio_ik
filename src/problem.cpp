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

#include "ik_base.h"

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>

#include <unordered_set>

#include <mutex>

#include <bio_ik/goal_types.h>

namespace bio_ik
{

enum class Problem::GoalType
{
    Unknown,
    Position,
    Orientation,
    Pose,
};

size_t Problem::addTipLink(const moveit::core::LinkModel* link_model)
{
    if(link_tip_indices[link_model->getLinkIndex()] < 0)
    {
        link_tip_indices[link_model->getLinkIndex()] = tip_link_indices.size();
        tip_link_indices.push_back(link_model->getLinkIndex());
    }
    return link_tip_indices[link_model->getLinkIndex()];
}

Problem::Problem()
    : ros_params_initrd(false)
{
}

void Problem::initialize(moveit::core::RobotModelConstPtr robot_model, const moveit::core::JointModelGroup* joint_model_group, const IKParams& params, const std::vector<const Goal*>& goals2, const BioIKKinematicsQueryOptions* options)
{
    if(robot_model != this->robot_model)
    {
        modelInfo = RobotInfo(robot_model);
#if (MOVEIT_FCL_VERSION < FCL_VERSION_CHECK(0, 6, 0))
        collision_links.clear();
        collision_links.resize(robot_model->getLinkModelCount());
#endif
    }

    this->robot_model = robot_model;
    this->joint_model_group = joint_model_group;
    this->params = params;

    if(!ros_params_initrd)
    {
        ros_params_initrd = true;
        dpos = params.dpos;
        drot = params.drot;
        dtwist = params.dtwist;
        if(dpos < 0.0 || dpos >= FLT_MAX || !std::isfinite(dpos)) dpos = DBL_MAX;
        if(drot < 0.0 || drot >= FLT_MAX || !std::isfinite(drot)) drot = DBL_MAX;
        if(dtwist < 0.0 || dtwist >= FLT_MAX || !std::isfinite(dtwist)) dtwist = DBL_MAX;
    }

    link_tip_indices.clear();
    link_tip_indices.resize(robot_model->getLinkModelCount(), -1);
    tip_link_indices.clear();

    active_variables.clear();
    auto addActiveVariable = [this, robot_model, joint_model_group, options](const std::string& name) -> ssize_t {
        if(options)
        {
            auto& joint_name = robot_model->getJointOfVariable(name)->getName();
            for(auto& fixed_joint_name : options->fixed_joints)
            {
                if(fixed_joint_name == joint_name)
                {
                    return (ssize_t)-1 - (ssize_t)robot_model->getVariableIndex(name);
                }
            }
        }
        for(size_t i = 0; i < active_variables.size(); i++)
            if(name == robot_model->getVariableNames()[active_variables[i]]) return i;
        for(auto& n : joint_model_group->getVariableNames())
        {
            if(n == name)
            {
                active_variables.push_back(robot_model->getVariableIndex(name));
                return active_variables.size() - 1;
            }
        }
        ERROR("joint variable not found", name);
    };

    goals.clear();
    secondary_goals.clear();
    for(auto& goal : goals2)
    {
        GoalInfo goal_info;

        goal_info.goal = goal;

        goal->describe(goal_info.goal_context);

        for(auto& link_name : goal_info.goal_context.goal_link_names_)
        {
            auto* link_model = robot_model->getLinkModel(link_name);
            if(!link_model) ERROR("link not found", link_name);
            goal_info.goal_context.goal_link_indices_.push_back(addTipLink(link_model));
        }

        for(auto& variable_name : goal_info.goal_context.goal_variable_names_)
        {
            goal_info.goal_context.goal_variable_indices_.push_back(addActiveVariable(variable_name));
        }

        goal_info.weight = goal_info.goal_context.goal_weight_;
        goal_info.weight_sq = goal_info.weight * goal_info.weight;

        goal_info.goal_type = GoalType::Unknown;

        goal_info.frame = Frame::identity();
        goal_info.tip_index = 0;
        if(goal_info.goal_context.goal_link_indices_.size()) goal_info.tip_index = goal_info.goal_context.goal_link_indices_[0];

        if(auto* g = dynamic_cast<const PositionGoal*>(goal_info.goal))
        {
            goal_info.goal_type = GoalType::Position;
            goal_info.frame.pos = g->getPosition();
        }

        if(auto* g = dynamic_cast<const OrientationGoal*>(goal_info.goal))
        {
            goal_info.goal_type = GoalType::Orientation;
            goal_info.frame.rot = g->getOrientation();
        }

        if(auto* g = dynamic_cast<const PoseGoal*>(goal_info.goal))
        {
            goal_info.goal_type = GoalType::Pose;
            goal_info.frame.pos = g->getPosition();
            goal_info.frame.rot = g->getOrientation();
        }

        goal_info.goal_context.joint_model_group_ = joint_model_group;
        goal_info.goal_context.initial_guess_ = initial_guess;

        if(goal_info.goal_context.goal_secondary_)
            secondary_goals.push_back(goal_info);
        else
            goals.push_back(goal_info);

        // if(goal_info.variable_indices.size() > temp_variables.size()) temp_variables.resize(goal_info.variable_indices.size());

        // if(goal_info.link_indices.size() > temp_frames.size()) temp_frames.resize(goal_info.link_indices.size());
    }

    // update active variables from active subtree
    joint_usage.resize(robot_model->getJointModelCount());
    for(auto& u : joint_usage)
        u = 0;
    for(auto tip_index : tip_link_indices)
        for(auto* link_model = robot_model->getLinkModels()[tip_index]; link_model; link_model = link_model->getParentLinkModel())
            joint_usage[link_model->getParentJointModel()->getJointIndex()] = 1;
    if(options)
        for(auto& fixed_joint_name : options->fixed_joints)
            joint_usage[robot_model->getJointModel(fixed_joint_name)->getJointIndex()] = 0;
    for(auto* joint_model : joint_model_group->getActiveJointModels())
        if(joint_usage[joint_model->getJointIndex()] && !joint_model->getMimic())
            for(auto& n : joint_model->getVariableNames())
                addActiveVariable(n);

    // init weights for minimal displacement goals
    {
        minimal_displacement_factors.resize(active_variables.size());
        double s = 0;
        for(auto ivar : active_variables)
            s += modelInfo.getMaxVelocityRcp(ivar);
        if(s > 0)
        {
            for(size_t i = 0; i < active_variables.size(); i++)
            {
                auto ivar = active_variables[i];
                minimal_displacement_factors[i] = modelInfo.getMaxVelocityRcp(ivar) / s;
            }
        }
        else
        {
            for(size_t i = 0; i < active_variables.size(); i++)
                minimal_displacement_factors[i] = 1.0 / active_variables.size();
        }
    }

    initialize2();
}

void Problem::initialize2()
{
    for(auto* gg : {&goals, &secondary_goals})
    {
        for(auto& g : *gg)
        {
            g.goal_context.problem_active_variables_ = active_variables;
            g.goal_context.problem_tip_link_indices_ = tip_link_indices;
            g.goal_context.velocity_weights_ = minimal_displacement_factors;
            g.goal_context.robot_info_ = &modelInfo;
        }
    }
}

double Problem::computeGoalFitness(GoalInfo& goal_info, const Frame* tip_frames, const double* active_variable_positions)
{
    goal_info.goal_context.tip_link_frames_ = tip_frames;
    goal_info.goal_context.active_variable_positions_ = active_variable_positions;
    return goal_info.goal->evaluate(goal_info.goal_context) * goal_info.weight_sq;
}

double Problem::computeGoalFitness(std::vector<GoalInfo>& goals, const Frame* tip_frames, const double* active_variable_positions)
{
    double sum = 0.0;
    for(auto& goal : goals)
        sum += computeGoalFitness(goal, tip_frames, active_variable_positions);
    return sum;
}

bool Problem::checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
{
    for(auto& goal : goals)
    {
        const auto& fa = goal.frame;
        const auto& fb = tip_frames[goal.tip_index];

        switch(goal.goal_type)
        {

        case GoalType::Position:
        {
            if(dpos != DBL_MAX)
            {
                double p_dist = (fb.pos - fa.pos).length();
                if(!(p_dist <= dpos)) return false;
            }
            if(dtwist != DBL_MAX)
            {
                KDL::Frame fk_kdl, ik_kdl;
                frameToKDL(fa, fk_kdl);
                frameToKDL(fb, ik_kdl);
                KDL::Twist kdl_diff(fk_kdl.M.Inverse() * KDL::diff(fk_kdl.p, ik_kdl.p), fk_kdl.M.Inverse() * KDL::diff(fk_kdl.M, ik_kdl.M));
                if(!KDL::Equal(kdl_diff.vel, KDL::Twist::Zero().vel, dtwist)) return false;
            }
            continue;
        }

        case GoalType::Orientation:
        {
            if(drot != DBL_MAX)
            {
                double r_dist = fb.rot.angleShortestPath(fa.rot);
                r_dist = r_dist * 180 / M_PI;
                if(!(r_dist <= drot)) return false;
            }
            if(dtwist != DBL_MAX)
            {
                KDL::Frame fk_kdl, ik_kdl;
                frameToKDL(fa, fk_kdl);
                frameToKDL(fb, ik_kdl);
                KDL::Twist kdl_diff(fk_kdl.M.Inverse() * KDL::diff(fk_kdl.p, ik_kdl.p), fk_kdl.M.Inverse() * KDL::diff(fk_kdl.M, ik_kdl.M));
                if(!KDL::Equal(kdl_diff.rot, KDL::Twist::Zero().rot, dtwist)) return false;
            }
            continue;
        }

        case GoalType::Pose:
        {
            if(dpos != DBL_MAX || drot != DBL_MAX)
            {
                double p_dist = (fb.pos - fa.pos).length();
                double r_dist = fb.rot.angleShortestPath(fa.rot);
                r_dist = r_dist * 180 / M_PI;
                if(!(p_dist <= dpos)) return false;
                if(!(r_dist <= drot)) return false;
            }
            if(dtwist != DBL_MAX)
            {
                KDL::Frame fk_kdl, ik_kdl;
                frameToKDL(fa, fk_kdl);
                frameToKDL(fb, ik_kdl);
                KDL::Twist kdl_diff(fk_kdl.M.Inverse() * KDL::diff(fk_kdl.p, ik_kdl.p), fk_kdl.M.Inverse() * KDL::diff(fk_kdl.M, ik_kdl.M));
                if(!KDL::Equal(kdl_diff, KDL::Twist::Zero(), dtwist)) return false;
            }
            continue;
        }

        default:
        {
            double dmax = DBL_MAX;
            dmax = std::fmin(dmax, dpos);
            dmax = std::fmin(dmax, dtwist);
            double d = computeGoalFitness(goal, tip_frames.data(), active_variable_positions);
            if(!(d < dmax * dmax)) return false;
        }
        }
    }

    // LOG("checkSolutionActiveVariables true");

    return true;
}
}
