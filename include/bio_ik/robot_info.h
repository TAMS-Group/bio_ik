// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <bio_ik/goal.h>

namespace bio_ik
{

// Compatability
// MoveIt API changed from boost::shared_ptr to std::shared_ptr
// Built-in RobotModelConstPtr is only available in recent versions
// Define compatible RobotModel pointer
typedef std::decay<decltype(((moveit::core::RobotState*)0)->getRobotModel())>::type MoveItRobotModelConstPtr;

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
    MoveItRobotModelConstPtr robot_model;

    __attribute__((always_inline)) static inline double clamp2(double v, double lo, double hi)
    {
        if(__builtin_expect(v < lo, 0)) v = lo;
        if(__builtin_expect(v > hi, 0)) v = hi;
        return v;
    }

public:
    RobotInfo() {}
    RobotInfo(MoveItRobotModelConstPtr model)
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
