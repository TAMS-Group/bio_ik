// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <tf/tf.h>

#include <moveit/kinematics_base/kinematics_base.h>

namespace bio_ik
{

struct Goal
{
    Goal() {}
    virtual ~Goal() {}
};

struct GoalBase : Goal
{
    double weight;
    GoalBase()
        : weight(1)
    {
    }
    GoalBase(double weight)
        : weight(weight)
    {
    }
};

struct LinkGoalBase : GoalBase
{
    std::string link_name;
    LinkGoalBase() {}
    LinkGoalBase(const std::string& link_name, double weight)
        : link_name(link_name)
        , GoalBase(weight)
    {
    }
};

struct PositionGoal : LinkGoalBase
{
    tf::Vector3 position;
    PositionGoal()
        : position(0, 0, 0)
    {
    }
    PositionGoal(const std::string& link_name, const tf::Vector3& position, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(position)
    {
    }
};

struct OrientationGoal : LinkGoalBase
{
    tf::Quaternion orientation;
    OrientationGoal()
        : orientation(0, 0, 0, 1)
    {
    }
    OrientationGoal(const std::string& link_name, const tf::Quaternion& orientation, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , orientation(orientation)
    {
    }
};

struct PoseGoal : LinkGoalBase
{
    tf::Vector3 position;
    tf::Quaternion orientation;
    double rotation_scale;
    PoseGoal()
        : rotation_scale(0.5)
        , position(0, 0, 0)
        , orientation(0, 0, 0, 1)
    {
    }
    PoseGoal(const std::string& link_name, const tf::Vector3& position, const tf::Quaternion& orientation, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(position)
        , orientation(orientation)
    {
    }
};

struct LookAtGoal : LinkGoalBase
{
    tf::Vector3 axis;
    tf::Vector3 target;
    LookAtGoal()
        : axis(1, 0, 0)
        , target(0, 0, 0)
    {
    }
    LookAtGoal(const std::string& link_name, const tf::Vector3& axis, const tf::Vector3& target, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , axis(axis)
        , target(target)
    {
    }
};

struct MaxDistanceGoal : LinkGoalBase
{
    tf::Vector3 target;
    double distance;
    MaxDistanceGoal()
        : target(0, 0, 0)
        , distance(1)
    {
    }
    MaxDistanceGoal(const std::string& link_name, const tf::Vector3& target, double distance, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , target(target)
        , distance(distance)
    {
    }
};

struct MinDistanceGoal : LinkGoalBase
{
    tf::Vector3 target;
    double distance;
    MinDistanceGoal()
        : target(0, 0, 0)
        , distance(1)
    {
    }
    MinDistanceGoal(const std::string& link_name, const tf::Vector3& target, double distance, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , target(target)
        , distance(distance)
    {
    }
};

struct LineGoal : LinkGoalBase
{
    tf::Vector3 position;
    tf::Vector3 direction;
    LineGoal()
        : position(0, 0, 0)
        , direction(0, 0, 0)
    {
    }
    LineGoal(const std::string& link_name, const tf::Vector3& position, const tf::Vector3& direction, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(position)
        , direction(direction)
    {
    }
};

struct TouchGoal : LinkGoalBase
{
    tf::Vector3 position;
    tf::Vector3 normal;
    TouchGoal()
        : position(0, 0, 0)
        , normal(0, 0, 0)
    {
    }
    TouchGoal(const std::string& link_name, const tf::Vector3& position, const tf::Vector3& normal, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(position)
        , normal(normal)
    {
    }
};

struct AvoidJointLimitsGoal : GoalBase
{
    bool secondary;
    AvoidJointLimitsGoal(double weight = 1.0, bool secondary = true)
    {
        this->weight = weight;
        this->secondary = secondary;
    }
};

struct CenterJointsGoal : GoalBase
{
    bool secondary;
    CenterJointsGoal(double weight = 1.0, bool secondary = true)
    {
        this->weight = weight;
        this->secondary = secondary;
    }
};

struct MinimalDisplacementGoal : GoalBase
{
    bool secondary;
    MinimalDisplacementGoal(double weight = 1.0, bool secondary = true)
    {
        this->weight = weight;
        this->secondary = secondary;
    }
};

struct JointVariableGoal : GoalBase
{
    bool secondary;
    std::string variable_name;
    double variable_position;
    JointVariableGoal()
        : variable_position(0)
        , secondary(false)
    {
    }
    JointVariableGoal(const std::string& variable_name, double variable_position, double weight = 1.0, bool secondary = false)
        : variable_name(variable_name)
        , variable_position(variable_position)
    {
        this->weight = weight;
        this->secondary = secondary;
    }
};

struct JointFunctionGoal : GoalBase
{
    bool secondary;
    std::vector<std::string> variable_names;
    std::function<void(std::vector<double>&)> function;
    JointFunctionGoal()
        : secondary(false)
    {
    }
    JointFunctionGoal(const std::vector<std::string>& variable_names, const std::function<void(std::vector<double>&)>& function, double weight = 1.0, bool secondary = false)
        : variable_names(variable_names)
        , function(function)
    {
        this->weight = weight;
        this->secondary = secondary;
    }
};

struct BalanceGoal : GoalBase
{
    tf::Vector3 center, axis;
    BalanceGoal()
        : center(0, 0, 0)
        , axis(0, 0, 1)
    {
    }
    BalanceGoal(const tf::Vector3& center, double weight = 1.0)
        : center(center)
        , axis(0, 0, 1)
        , GoalBase(weight)
    {
    }
};

struct LinkFunctionGoal : LinkGoalBase
{
    std::function<double(const tf::Vector3&, const tf::Quaternion&)> function;
    LinkFunctionGoal() {}
    LinkFunctionGoal(const std::string& link_name, const std::function<double(const tf::Vector3&, const tf::Quaternion&)>& function, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , function(function)
    {
    }
};

struct SideGoal : LinkGoalBase
{
    tf::Vector3 axis;
    tf::Vector3 direction;
    SideGoal()
        : axis(0, 0, 1)
        , direction(0, 0, 1)
    {
    }
    SideGoal(const std::string& link_name, const tf::Vector3& axis, const tf::Vector3& direction, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , axis(axis)
        , direction(direction)
    {
    }
};

struct DirectionGoal : LinkGoalBase
{
    tf::Vector3 axis;
    tf::Vector3 direction;
    DirectionGoal()
        : axis(0, 0, 1)
        , direction(0, 0, 1)
    {
    }
    DirectionGoal(const std::string& link_name, const tf::Vector3& axis, const tf::Vector3& direction, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , axis(axis)
        , direction(direction)
    {
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
