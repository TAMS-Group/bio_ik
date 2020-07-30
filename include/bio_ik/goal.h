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

#pragma once

#include "frame.h"

#include <moveit/kinematics_base/kinematics_base.h>

#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>

namespace bio_ik
{

class RobotInfo;

class GoalContext
{
protected:
    const double* active_variable_positions_;
    const Frame* tip_link_frames_;
    std::vector<ssize_t> goal_variable_indices_;
    std::vector<size_t> goal_link_indices_;
    bool goal_secondary_;
    std::vector<std::string> goal_link_names_, goal_variable_names_;
    double goal_weight_;
    const moveit::core::JointModelGroup* joint_model_group_;
    std::vector<size_t> problem_active_variables_;
    std::vector<size_t> problem_tip_link_indices_;
    std::vector<double> initial_guess_;
    std::vector<double> velocity_weights_;
    const RobotInfo* robot_info_;
    mutable std::vector<double> temp_vector_;

public:
    GoalContext() {}
    inline const Frame& getLinkFrame(size_t i = 0) const { return tip_link_frames_[goal_link_indices_[i]]; }
    inline const double getVariablePosition(size_t i = 0) const
    {
        auto j = goal_variable_indices_[i];
        if(j >= 0)
            return active_variable_positions_[j];
        else
            return initial_guess_[-1 - j];
    }
    inline const Frame& getProblemLinkFrame(size_t i) const { return tip_link_frames_[i]; }
    inline size_t getProblemLinkCount() const { return problem_tip_link_indices_.size(); }
    inline size_t getProblemLinkIndex(size_t i) const { return problem_tip_link_indices_[i]; }
    inline double getProblemVariablePosition(size_t i) const { return active_variable_positions_[i]; }
    inline size_t getProblemVariableCount() const { return problem_active_variables_.size(); }
    inline size_t getProblemVariableIndex(size_t i) const { return problem_active_variables_[i]; }
    inline double getProblemVariableInitialGuess(size_t i) const { return initial_guess_[problem_active_variables_[i]]; }
    inline double getProblemVariableWeight(size_t i) const { return velocity_weights_[i]; }
    inline const RobotInfo& getRobotInfo() const { return *robot_info_; }
    void addLink(const std::string& name) { goal_link_names_.push_back(name); }
    void addVariable(const std::string& name) { goal_variable_names_.push_back(name); }
    void setSecondary(bool secondary) { goal_secondary_ = secondary; }
    void setWeight(double weight) { goal_weight_ = weight; }
    const moveit::core::JointModelGroup& getJointModelGroup() const { return *joint_model_group_; }
    const moveit::core::RobotModel& getRobotModel() const { return joint_model_group_->getParentModel(); }
    std::vector<double>& getTempVector() const { return temp_vector_; }
    friend class Problem;
};

class Goal
{
protected:
    bool secondary_;
    double weight_;

public:
    Goal()
        : weight_(1)
        , secondary_(false)
    {
    }
    virtual ~Goal() {}
    bool isSecondary() const { return secondary_; }
    double getWeight() const { return weight_; }
    void setWeight(double w) { weight_ = w; }
    virtual void describe(GoalContext& context) const
    {
        context.setSecondary(secondary_);
        context.setWeight(weight_);
    }
    virtual double evaluate(const GoalContext& context) const { return 0; }
};

struct BioIKKinematicsQueryOptions : kinematics::KinematicsQueryOptions
{
    std::vector<std::unique_ptr<Goal>> goals;
    std::vector<std::string> fixed_joints;
    bool replace;
    mutable double solution_fitness;
    BioIKKinematicsQueryOptions();
    ~BioIKKinematicsQueryOptions();
};
}
