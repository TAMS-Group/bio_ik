// Bio IK for ROS
// Philipp Ruppel

#pragma once

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "utils.h"
#include "frame.h"

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
        //bool bounded;
        double min;
        double max;
    };
    std::vector<VariableInfo> variables;
    std::vector<size_t> activeVariables;
    std::vector<moveit::core::JointModel::JointType> variable_joint_types;
    MoveItRobotModelConstPtr robot_model;
public:
    RobotInfo() { }
    RobotInfo(MoveItRobotModelConstPtr model) : robot_model(model)
    {
        for(auto& name : model->getVariableNames())
        {
            auto& bounds = model->getVariableBounds(name);
            
            VariableInfo info;
            
            //info.bounded = bounds.position_bounded_;
            //bounds.position_
            
            bool bounded = bounds.position_bounded_;
            
            
            auto* joint_model = model->getJointOfVariable(variables.size());
            if(auto* revolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model))
                //if(revolute->isContinuous()) bounded = false;
                if(bounds.max_position_ - bounds.min_position_ >= 2 * M_PI * 0.9999) bounded = false;
            
            
            info.min = bounds.min_position_;
            info.max = bounds.max_position_;

            
            info.clip_min = bounded ? info.min : -DBL_MAX;
            info.clip_max = bounded ? info.max : +DBL_MAX;
            
            
            //if(info.max == DBL_MAX) info.min = -1, info.max = 1, info.clip_min = -1, info.clip_max = 1;
            //if(info.max == DBL_MAX) info.min = -1, info.max = 1;
            
            
            
            info.span = info.max - info.min;
            
            
            LOG("joint variable", name, info.min, info.max);
            
            
            //if(info.span == DBL_MAX || !std::isfinite(info.span)) info.span = 0;
            //if(info.span == DBL_MAX || !std::isfinite(info.span) || !std::isfinite((float)info.span)) info.span = 1;
            //if(info.span == DBL_MAX || !std::isfinite(info.span)) info.span = 32;
            //if(info.span == DBL_MAX || !std::isfinite(info.span) || variables.size() < 7) info.span = 0;
            
            if(!(info.span >= 0 && info.span < FLT_MAX)) info.span = 1;
            
            //info.span = 1;
            
            variables.push_back(info);
        }
        for(size_t variable_index = 0; variable_index < model->getVariableCount(); variable_index++)
        {
            variable_joint_types.push_back(model->getJointOfVariable(variable_index)->getType());
        }
    }
    /*void initialize(const std::vector<size_t>& tip_link_indices)
    {
        for(auto tip_link_index : tip_link_indices)
        {
            auto* tipLink = robot_model->getLinkModel(tip_link_index);
            std::vector<const moveit::core::JointModel*> chain;
            for(auto* link = tipLink; link; link = link->getParentLinkModel())
            {
                auto* joint = link->getParentJointModel();
                if(joint->getMimic()) continue;
                if(joint->isPassive()) continue;
                chain.push_back(joint);
            }
            std::reverse(chain.begin(), chain.end());
            for(auto* joint : chain)
            {
                auto first = joint->getFirstVariableIndex();
                auto count = joint->getVariableCount();
                for(size_t vi = first; vi < first + count; vi++)
                {
                    if(find(activeVariables.begin(), activeVariables.end(), vi) != activeVariables.end()) continue;
                    activeVariables.push_back(vi);
                }
            }
        }
        for(auto& i : activeVariables) LOG("active variable", i);
    }*/
public:
    __attribute__((always_inline))
    inline double clip(double p, size_t i) const
    {
        auto& info = variables[i];
        
        //LOG(i, info.clip_min, info.clip_max);
        
        return clamp2(p, info.clip_min, info.clip_max);
        
        /*if(p > info.clip_max) p = info.clip_max;
        if(p < info.clip_min) p = info.clip_min;
        return p;*/
        
        //return std::fmax(info.clip_min, std::fmin(p, info.clip_max));
        //return std::clamp(p, info.clip_min, info.clip_max);
    }
    //inline bool isBounded(size_t i) const { return variables[i].bounded; }
    inline double getSpan(size_t i) const { return variables[i].span; }
    inline double getClipMin(size_t i) const { return variables[i].clip_min; }
    inline double getClipMax(size_t i) const { return variables[i].clip_max; }
    inline double getMin(size_t i) const { return variables[i].min; }
    inline double getMax(size_t i) const { return variables[i].max; }
    //inline const std::vector<size_t>& getActiveVariables() const { return activeVariables; }
    inline bool isRevolute(size_t variable_index) const { return variable_joint_types[variable_index] == moveit::core::JointModel::REVOLUTE; }
    inline bool isPrismatic(size_t variable_index) const { return variable_joint_types[variable_index] == moveit::core::JointModel::PRISMATIC; }
};

/*
class HeuristicErrorTree
{
    size_t variable_count, tip_count;
    std::vector<double> table;
    std::vector<double> chain_lengths;
    std::vector<std::vector<double>> chain_lengths_2;
public:
    HeuristicErrorTree(MoveItRobotModelConstPtr robot_model, const std::vector<std::string>& tip_names)
    {
        tip_count = tip_names.size();
        variable_count = robot_model->getVariableCount();
        table.resize(tip_count * variable_count);
        for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
        {
            auto& tip_name = tip_names[tip_index];
            for(auto* link_model = robot_model->getLinkModel(tip_name); link_model; link_model = link_model->getParentLinkModel())
            {
                auto* joint_model = link_model->getParentJointModel();
                size_t v1 = joint_model->getFirstVariableIndex();
                size_t vn = joint_model->getVariableCount();
                for(size_t variable_index = v1; variable_index < v1 + vn; variable_index++)
                    table[variable_index * tip_count + tip_index] = 1;
            }
        }
        for(size_t variable_index = 0; variable_index < variable_count; variable_index++)
        {
            double sum = 0;
            for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
                sum += table[variable_index * tip_count + tip_index];
            if(sum > 0)
                for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
                    table[variable_index * tip_count + tip_index] /= sum;
        }
        
        chain_lengths.resize(tip_count);
        for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
        {
            auto& tip_name = tip_names[tip_index];
            double chain_length = 0;
            for(auto* link_model = robot_model->getLinkModel(tip_name); link_model; link_model = link_model->getParentLinkModel())
            {
                chain_length += Frame(link_model->getJointOriginTransform()).pos.length();
            }
            chain_lengths[tip_index] = chain_length;
        }
        
        chain_lengths_2.resize(tip_count);
        for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
        {
            auto& tip_name = tip_names[tip_index];
            double chain_length = 0;
            chain_lengths_2[tip_index].resize(variable_count, 0.0);
            for(auto* link_model = robot_model->getLinkModel(tip_name); link_model; link_model = link_model->getParentLinkModel())
            {
                //chain_lengths_2.push_back(chain_length);
                auto* joint_model = link_model->getParentJointModel();
                int vmin = joint_model->getFirstVariableIndex();
                int vmax = vmin + joint_model->getVariableCount();
                for(int vi = vmin; vi < vmax; vi++)
                    chain_lengths_2[tip_index][vi] = chain_length;
                chain_length += Frame(link_model->getJointOriginTransform()).pos.length();
            }
        }
    }
    inline double getInfluence(size_t variable_index, size_t tip_index) const
    {
        return table[variable_index * tip_count + tip_index];
    }
    inline double getChainLength(size_t tip_index) const
    {
        return chain_lengths[tip_index];
    }
    inline double getJointVariableChainLength(size_t tip_index, size_t variable_index) const
    {
        return chain_lengths_2[tip_index][variable_index];
    }
};
*/

}