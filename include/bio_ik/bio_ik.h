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

struct GoalBase : Goal
{
    double weight;
    GoalBase() : weight(1)
    {
    }
};

struct LinkGoalBase : GoalBase
{
    std::string link_name;
};

struct PositionGoal : LinkGoalBase
{
    tf::Vector3 position;
    PositionGoal() : position(0, 0, 0)
    {
    }
};

struct OrientationGoal : LinkGoalBase
{
    tf::Quaternion orientation;
    OrientationGoal() : orientation(0, 0, 0, 1)
    {
    }
};

struct PoseGoal : LinkGoalBase
{
    tf::Vector3 position;
    tf::Quaternion orientation;
    double rotation_scale;
    PoseGoal() : rotation_scale(0.5), position(0, 0, 0), orientation(0, 0, 0, 1)
    {
    }
};

struct LookAtGoal : LinkGoalBase
{
    tf::Vector3 axis;
    tf::Vector3 target;
    LookAtGoal() : axis(1, 0, 0), target(0, 0, 0)
    {
    }
};

struct MaxDistanceGoal : LinkGoalBase
{
    tf::Vector3 target;
    double distance;
    MaxDistanceGoal() : target(0, 0, 0), distance(1)
    {
    }
};

struct AvoidJointLimitsGoal : GoalBase
{
    bool secondary;
    AvoidJointLimitsGoal() : secondary(true)
    {
    }
};

struct CenterJointsGoal : GoalBase
{
    bool secondary;
    CenterJointsGoal() : secondary(true)
    {
    }
};

struct MinimalDisplacementGoal : GoalBase
{
    bool secondary;
    MinimalDisplacementGoal() : secondary(true)
    {
    }
};

struct JointVariableGoal : GoalBase
{
    bool secondary;
    std::string variable_name;
    double variable_position;
    JointVariableGoal() : variable_position(0), secondary(false)
    {
    }
};

/*
struct LambdaGoalBase : GoalBase
{
    std::vector<std::string> variable_names;
    std::function<double(const double*)> lambda;
};

struct LambdaGoal : LambdaGoalBase
{
    template<class F>
    LambdaGoal(const std::string& a, const std::string& b, const F& f)
    {
        variable_names = { a, b };
        lambda = [f] (const double* a) { return f(a[0], a[1]); };
    }
};
*/

struct JointTransmissionGoal : GoalBase
{
    bool secondary;
    std::vector<std::string> variable_names;
    std::function<void(std::vector<double>&)> lambda;
    
    template<class F>
    JointTransmissionGoal(const std::string& a, const std::string& b, const F& f)
    {
        variable_names = { a, b };
        lambda = f;
        secondary = false;
    }
    
    template<class F>
    JointTransmissionGoal(const std::vector<std::string>& variables, const F& f)
    {
        variable_names = variables;
        lambda = f;
        secondary = false;
    }
};

struct BioIKKinematicsQueryOptions : kinematics::KinematicsQueryOptions
{
    std::vector<std::unique_ptr<Goal>> goals;
    bool replace;
    BioIKKinematicsQueryOptions();
    ~BioIKKinematicsQueryOptions();
};

}




