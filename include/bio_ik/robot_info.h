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

#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <bio_ik/goal.h>

namespace bio_ik
{

class RobotInfo
{
    struct VariableInfo
    {
        double clip_min, clip_max;
        double span;
        double min;
        double max;
        double max_velocity, max_velocity_rcp;
    };
    std::vector<VariableInfo> variables;
    std::vector<size_t> activeVariables;
    std::vector<moveit::core::JointModel::JointType> variable_joint_types;
    moveit::core::RobotModelConstPtr robot_model;

    __attribute__((always_inline)) static inline double clamp2(double v, double lo, double hi)
    {
        if(__builtin_expect(v < lo, 0)) v = lo;
        if(__builtin_expect(v > hi, 0)) v = hi;
        return v;
    }

public:
    RobotInfo() {}
    RobotInfo(moveit::core::RobotModelConstPtr model)
        : robot_model(model)
    {
        for(auto& name : model->getVariableNames())
        {
            auto& bounds = model->getVariableBounds(name);

            VariableInfo info;

            bool bounded = bounds.position_bounded_;

            auto* joint_model = model->getJointOfVariable(variables.size());
            if(auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model))

                if(bounds.max_position_ - bounds.min_position_ >= 2 * M_PI * 0.9999) bounded = false;

            info.min = bounds.min_position_;
            info.max = bounds.max_position_;

            info.clip_min = bounded ? info.min : -DBL_MAX;
            info.clip_max = bounded ? info.max : +DBL_MAX;

            info.span = info.max - info.min;

            if(!(info.span >= 0 && info.span < FLT_MAX)) info.span = 1;

            info.max_velocity = bounds.max_velocity_;
            info.max_velocity_rcp = info.max_velocity > 0.0 ? 1.0 / info.max_velocity : 0.0;

            variables.push_back(info);
        }

        for(size_t ivar = 0; ivar < model->getVariableCount(); ivar++)
        {
            variable_joint_types.push_back(model->getJointOfVariable(ivar)->getType());
        }
    }

public:
    __attribute__((always_inline)) inline double clip(double p, size_t i) const
    {
        auto& info = variables[i];
        return clamp2(p, info.clip_min, info.clip_max);
    }

    inline double getSpan(size_t i) const { return variables[i].span; }
    inline double getClipMin(size_t i) const { return variables[i].clip_min; }
    inline double getClipMax(size_t i) const { return variables[i].clip_max; }
    inline double getMin(size_t i) const { return variables[i].min; }
    inline double getMax(size_t i) const { return variables[i].max; }

    inline bool isRevolute(size_t variable_index) const { return variable_joint_types[variable_index] == moveit::core::JointModel::REVOLUTE; }
    inline bool isPrismatic(size_t variable_index) const { return variable_joint_types[variable_index] == moveit::core::JointModel::PRISMATIC; }
    inline double getMaxVelocity(size_t i) const { return variables[i].max_velocity; }
    inline double getMaxVelocityRcp(size_t i) const { return variables[i].max_velocity_rcp; }
};
}
