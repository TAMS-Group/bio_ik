// Bio IK for ROS
// Philipp Ruppel

#pragma once

#include <string>
#include <vector>
#include <memory>

#include <tf/tf.h>

#include <moveit/kinematics_base/kinematics_base.h>

namespace bio_ik
{

struct Goal
{
    Goal()
    {
    }
    virtual ~Goal()
    {
    }
};

struct LinkGoalBase : Goal
{
    std::string link_name;
    double weight;
    LinkGoalBase() : weight(1)
    {
    }
};

struct PositionGoal : LinkGoalBase
{
    tf::Vector3 position;
};

struct OrientationGoal : LinkGoalBase
{
    tf::Quaternion orientation;
};

struct PoseGoal : LinkGoalBase
{
    tf::Vector3 position;
    tf::Quaternion orientation;
    double rotation_scale;
    PoseGoal() : rotation_scale(0.5)
    {
    }
};

struct LookAtGoal : LinkGoalBase
{
    tf::Vector3 target;
};

struct MaxDistanceGoal : LinkGoalBase
{
    tf::Vector3 target;
    double distance;
};

struct BioIKKinematicsQueryOptions : kinematics::KinematicsQueryOptions
{
    std::vector<std::unique_ptr<Goal>> goals;
    bool replace;
    BioIKKinematicsQueryOptions() : replace(false)
    {
    }
};

}




