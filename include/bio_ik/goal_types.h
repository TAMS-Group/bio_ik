// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#pragma once

#include "goal.h"

#include "robot_info.h"

#include <geometric_shapes/shapes.h>

#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

#include <map>
#include <unordered_set>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>

namespace bio_ik
{

class LinkGoalBase : public Goal
{
    std::string link_name_;

public:
    LinkGoalBase()
    {
        weight_ = 1;
        link_name_ = "";
    }
    LinkGoalBase(const std::string& link_name, double weight)
    {
        weight_ = weight;
        link_name_ = link_name;
    }
    virtual void describe(GoalContext& context) const
    {
        Goal::describe(context);
        context.addLink(link_name_);
    }
    void setLinkName(const std::string& link_name) { link_name_ = link_name; }
    const std::string& getLinkName() const { return link_name_; }
};

class PositionGoal : public LinkGoalBase
{
    tf::Vector3 position_;

public:
    PositionGoal()
        : position_(0, 0, 0)
    {
    }
    PositionGoal(const std::string& link_name, const tf::Vector3& position, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position_(position)
    {
    }
    inline const tf::Vector3& getPosition() const { return position_; }
    inline void setPosition(const tf::Vector3& position) { position_ = position; }
    virtual double evaluate(const GoalContext& context) const { return context.getLinkFrame().getPosition().distance2(getPosition()); }
};

class OrientationGoal : public LinkGoalBase
{
    tf::Quaternion orientation_;

public:
    OrientationGoal()
        : orientation_(0, 0, 0, 1)
    {
    }
    OrientationGoal(const std::string& link_name, const tf::Quaternion& orientation, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , orientation_(orientation.normalized())
    {
    }
    inline const tf::Quaternion& getOrientation() const { return orientation_; }
    inline void setOrientation(const tf::Quaternion& orientation) { orientation_ = orientation.normalized(); }
    virtual double evaluate(const GoalContext& context) const 
    { 
        //return getOrientation().distance2(context.getLinkFrame().getOrientation());
        //return (getOrientation() - getOrientation().nearest(context.getLinkFrame().getOrientation())).length2(); 
        return fmin( 
            (getOrientation() - context.getLinkFrame().getOrientation()).length2(),
            (getOrientation() + context.getLinkFrame().getOrientation()).length2()
        );
        /*return
            (getOrientation() - context.getLinkFrame().getOrientation()).length2() *
            (getOrientation() + context.getLinkFrame().getOrientation()).length2() * 0.5;*/
    }
};

class PoseGoal : public LinkGoalBase
{
    Frame frame_;
    double rotation_scale_;

public:
    PoseGoal()
        : rotation_scale_(0.5)
        , frame_(Frame::identity())
    {
    }
    PoseGoal(const std::string& link_name, const tf::Vector3& position, const tf::Quaternion& orientation, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , frame_(position, orientation.normalized())
    {
    }
    inline const tf::Vector3& getPosition() const { return frame_.getPosition(); }
    inline void setPosition(const tf::Vector3& position) { frame_.setPosition(position); }
    inline const tf::Quaternion& getOrientation() const { return frame_.getOrientation(); }
    inline void setOrientation(const tf::Quaternion& orientation) { frame_.setOrientation(orientation.normalized()); }
    inline double getRotationScale() const { return rotation_scale_; }
    inline void setRotationScale(double rotation_scale) { rotation_scale_ = rotation_scale; }
    virtual double evaluate(const GoalContext& context) const
    {
        double e = 0.0;
        e += context.getLinkFrame().getPosition().distance2(getPosition());
        
        /*e += 
            (getOrientation() - context.getLinkFrame().getOrientation()).length2() *
            (getOrientation() + context.getLinkFrame().getOrientation()).length2() * 
            (rotation_scale_ * rotation_scale_) * 0.5;*/
            
        e += fmin( 
            (getOrientation() - context.getLinkFrame().getOrientation()).length2(),
            (getOrientation() + context.getLinkFrame().getOrientation()).length2()
        ) * (rotation_scale_ * rotation_scale_);
        
        //e += (1.0 - getOrientation().dot(context.getLinkFrame().getOrientation())) * (rotation_scale_ * rotation_scale_);
        
        //e += (getOrientation() - context.getLinkFrame().getOrientation()).length2() * (rotation_scale_ * rotation_scale_);
        //ROS_ERROR("r %f", (getOrientation() - context.getLinkFrame().getOrientation()).length2());
        //e += (getOrientation() - getOrientation().nearest(context.getLinkFrame().getOrientation())).length2() * (rotation_scale_ * rotation_scale_);
        return e;
    }
};

class LookAtGoal : public LinkGoalBase
{
    tf::Vector3 axis_;
    tf::Vector3 target_;

public:
    LookAtGoal()
        : axis_(1, 0, 0)
        , target_(0, 0, 0)
    {
    }
    LookAtGoal(const std::string& link_name, const tf::Vector3& axis, const tf::Vector3& target, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , axis_(axis)
        , target_(target)
    {
    }
    const tf::Vector3& getAxis() const { return axis_; }
    const tf::Vector3& getTarget() const { return target_; }
    void setAxis(const tf::Vector3& axis) { axis_ = axis.normalized(); }
    void setTarget(const tf::Vector3& target) { target_ = target; }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        tf::Vector3 axis;
        quat_mul_vec(fb.getOrientation(), axis_, axis);
        return (target_ - fb.getPosition()).normalized().distance2(axis.normalized());
        //return (target_ - axis * axis.dot(target_ - fb.getPosition())).distance2(fb.getPosition());
    }
}; 

class MaxDistanceGoal : public LinkGoalBase
{
    tf::Vector3 target;
    double distance;

public:
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
    const tf::Vector3& getTarget() const { return target; }
    void setTarget(const tf::Vector3& t) { target = t; }
    double getDistance() const { return distance; }
    void setDistance(double d) { distance = d; }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        double d = fmax(0.0, fb.getPosition().distance(target) - distance);
        return d * d;
    }
};

class MinDistanceGoal : public LinkGoalBase
{
    tf::Vector3 target;
    double distance;

public:
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
    const tf::Vector3& getTarget() const { return target; }
    void setTarget(const tf::Vector3& t) { target = t; }
    double getDistance() const { return distance; }
    void setDistance(double d) { distance = d; }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        double d = fmax(0.0, distance - fb.getPosition().distance(target));
        return d * d;
    }
};

class LineGoal : public LinkGoalBase
{
    tf::Vector3 position;
    tf::Vector3 direction;

public:
    LineGoal()
        : position(0, 0, 0)
        , direction(0, 0, 0)
    {
    }
    LineGoal(const std::string& link_name, const tf::Vector3& position, const tf::Vector3& direction, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(position)
        , direction(direction.normalized())
    {
    }
    const tf::Vector3& getPosition() const { return position; }
    void setPosition(const tf::Vector3& p) { position = p; }
    const tf::Vector3& getDirection() const { return direction; }
    void setDirection(const tf::Vector3& d) { direction = d.normalized(); }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        return position.distance2(fb.getPosition() - direction * direction.dot(fb.getPosition() - position));
    }
};

class TouchGoal : public LinkGoalBase
{
    tf::Vector3 position;
    tf::Vector3 normal;
    struct CollisionShape
    {
        std::vector<Vector3> vertices;
        std::vector<fcl::Vec3f> points;
        std::vector<int> polygons;
        std::vector<fcl::Vec3f> plane_normals;
        std::vector<double> plane_dis;
        collision_detection::FCLGeometryConstPtr geometry;
        Frame frame;
        std::vector<std::vector<size_t>> edges;
    };
    struct CollisionLink
    {
        bool initialized;
        std::vector<std::shared_ptr<CollisionShape>> shapes;
        CollisionLink()
            : initialized(false)
        {
        }
    };
    struct CollisionModel
    {
        std::vector<CollisionLink> collision_links;
    };
    mutable CollisionModel* collision_model;
    mutable const moveit::core::LinkModel* link_model;
    
public:
    TouchGoal()
        : position(0, 0, 0)
        , normal(0, 0, 0)
    {
    }
    TouchGoal(const std::string& link_name, const tf::Vector3& position, const tf::Vector3& normal, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(position)
        , normal(normal.normalized())
    {
    }
    virtual void describe(GoalContext& context) const;
    virtual double evaluate(const GoalContext& context) const;
};

class AvoidJointLimitsGoal : public Goal
{
public:
    AvoidJointLimitsGoal(double weight = 1.0, bool secondary = true)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& info = context.getRobotInfo();
        double sum = 0.0;
        for(size_t i = 0; i < context.getProblemVariableCount(); i++)
        {
            size_t ivar = context.getProblemVariableIndex(i);
            if(info.getClipMax(ivar) == DBL_MAX) continue;
            double d = context.getProblemVariablePosition(i) - (info.getMin(ivar) + info.getMax(ivar)) * 0.5;
            d = fmax(0.0, fabs(d) * 2.0 - info.getSpan(ivar) * 0.5);
            d *= context.getProblemVariableWeight(i);
            sum += d * d;
        }
        return sum;
    }
};

class CenterJointsGoal : public Goal
{
public:
    CenterJointsGoal(double weight = 1.0, bool secondary = true)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& info = context.getRobotInfo();
        double sum = 0.0;
        for(size_t i = 0; i < context.getProblemVariableCount(); i++)
        {
            size_t ivar = context.getProblemVariableIndex(i);
            if(info.getClipMax(ivar) == DBL_MAX) continue;
            double d = context.getProblemVariablePosition(i) - (info.getMin(ivar) + info.getMax(ivar)) * 0.5;
            d *= context.getProblemVariableWeight(i);
            sum += d * d;
        }
        return sum;
    }
};

class MinimalDisplacementGoal : public Goal
{
public:
    MinimalDisplacementGoal(double weight = 1.0, bool secondary = true)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    virtual double evaluate(const GoalContext& context) const
    {
        double sum = 0.0;
        for(size_t i = 0; i < context.getProblemVariableCount(); i++)
        {
            double d = context.getProblemVariablePosition(i) - context.getProblemVariableInitialGuess(i);
            d *= context.getProblemVariableWeight(i);
            sum += d * d;
        }
        return sum;
    }
};

class JointVariableGoal : public Goal
{
    std::string variable_name;
    double variable_position;

public:
    JointVariableGoal()
        : variable_position(0)
    {
    }
    JointVariableGoal(const std::string& variable_name, double variable_position, double weight = 1.0, bool secondary = false)
        : variable_name(variable_name)
        , variable_position(variable_position)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    double getVariablePosition() const { return variable_position; }
    void setVariablePosition(double p) { variable_position = p; }
    const std::string& getVariableName() const { return variable_name; }
    void setVariableName(const std::string& n) { variable_name = n; }
    virtual void describe(GoalContext& context) const
    {
        Goal::describe(context);
        context.addVariable(variable_name);
    }
    virtual double evaluate(const GoalContext& context) const
    {
        double d = variable_position - context.getVariablePosition();
        return d * d;
    }
};

class JointFunctionGoal : public Goal
{
    std::vector<std::string> variable_names;
    std::function<void(std::vector<double>&)> function;

public:
    JointFunctionGoal() {}
    JointFunctionGoal(const std::vector<std::string>& variable_names, const std::function<void(std::vector<double>&)>& function, double weight = 1.0, bool secondary = false)
        : variable_names(variable_names)
        , function(function)
    {
        weight_ = weight;
        secondary_ = secondary;
    }
    void setJointVariableNames(const std::vector<std::string>& n) { variable_names = n; }
    void setJointVariableFunction(const std::function<void(std::vector<double>&)>& f) { function = f; }
    virtual void describe(GoalContext& context) const
    {
        Goal::describe(context);
        for(auto& variable_name : variable_names)
            context.addVariable(variable_name);
    }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& temp_vector = context.getTempVector();
        temp_vector.resize(variable_names.size());
        for(size_t i = 0; i < variable_names.size(); i++)
            temp_vector[i] = context.getVariablePosition(i);
        function(temp_vector);
        double sum = 0.0;
        for(size_t i = 0; i < variable_names.size(); i++)
        {
            double d = temp_vector[i] - context.getVariablePosition(i);
            sum += d * d;
        }
        return sum;
    }
};

class BalanceGoal : public Goal
{
    tf::Vector3 target_, axis_;
    struct BalanceInfo
    {
        tf::Vector3 center;
        double weight;
    };
    mutable std::vector<BalanceInfo> balance_infos;

public:
    BalanceGoal()
        : target_(0, 0, 0)
        , axis_(0, 0, 1)
    {
    }
    BalanceGoal(const tf::Vector3& target, double weight = 1.0)
        : target_(target)
        , axis_(0, 0, 1)
    {
        weight_ = weight;
    }
    const tf::Vector3& getTarget() const { return target_; }
    const tf::Vector3& getAxis() const { return axis_; }
    void setTarget(const tf::Vector3& target) { target_ = target; }
    void setAxis(const tf::Vector3& axis) { axis_ = axis; }
    virtual void describe(GoalContext& context) const;
    virtual double evaluate(const GoalContext& context) const;
};

class LinkFunctionGoal : public LinkGoalBase
{
    std::function<double(const tf::Vector3&, const tf::Quaternion&)> function;

public:
    LinkFunctionGoal() {}
    LinkFunctionGoal(const std::string& link_name, const std::function<double(const tf::Vector3&, const tf::Quaternion&)>& function, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , function(function)
    {
    }
    void setLinkFunction(const std::function<double(const tf::Vector3&, const tf::Quaternion&)>& f) { function = f; }
    virtual double evaluate(const GoalContext& context) const { return function(context.getLinkFrame().getPosition(), context.getLinkFrame().getOrientation()); }
};

class SideGoal : public LinkGoalBase
{
    tf::Vector3 axis;
    tf::Vector3 direction;

public:
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
    const tf::Vector3& getAxis() const { return axis; }
    const tf::Vector3& getDirection() const { return direction; }
    void setAxis(const tf::Vector3& a) { axis = a.normalized(); }
    void setDirection(const tf::Vector3& d) { direction = d.normalized(); }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        Vector3 v;
        quat_mul_vec(fb.getOrientation(), axis, v);
        double f = fmax(0.0, v.dot(direction));
        return f * f;
    }
};

class DirectionGoal : public LinkGoalBase
{
    tf::Vector3 axis;
    tf::Vector3 direction;

public:
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
    const tf::Vector3& getAxis() const { return axis; }
    const tf::Vector3& getDirection() const { return direction; }
    void setAxis(const tf::Vector3& a) { axis = a.normalized(); }
    void setDirection(const tf::Vector3& d) { direction = d.normalized(); }
    virtual double evaluate(const GoalContext& context) const
    {
        auto& fb = context.getLinkFrame();
        Vector3 v;
        quat_mul_vec(fb.getOrientation(), axis, v);
        return v.distance2(direction);
    }
};

class ConeGoal : public LinkGoalBase
{
    tf::Vector3 position;
    double position_weight;
    tf::Vector3 axis;
    tf::Vector3 direction;
    double angle;

public:
    const tf::Vector3& getPosition() const { return position; }
    double getPositionWeight() const { return position_weight; }
    const tf::Vector3& getAxis() const { return axis; }
    const tf::Vector3& getDirection() const { return direction; }
    double getAngle() const { return angle; }
    void setPosition(const tf::Vector3& p) { position = p; }
    void setPositionWeight(double w) { position_weight = w; }
    void setAxis(const tf::Vector3& a) { axis = a.normalized(); }
    void setDirection(const tf::Vector3& d) { direction = d.normalized(); }
    void setAngle(double a) { angle = a; }
    ConeGoal()
        : position(0, 0, 0)
        , position_weight(0)
        , axis(0, 0, 1)
        , direction(0, 0, 1)
        , angle(0)
    {
    }
    ConeGoal(const std::string& link_name, const tf::Vector3& axis, const tf::Vector3& direction, double angle, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(0, 0, 0)
        , position_weight(0)
        , axis(axis)
        , direction(direction)
        , angle(angle)
    {
    }
    ConeGoal(const std::string& link_name, const tf::Vector3& position, const tf::Vector3& axis, const tf::Vector3& direction, double angle, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(position)
        , position_weight(1)
        , axis(axis)
        , direction(direction)
        , angle(angle)
    {
    }
    ConeGoal(const std::string& link_name, const tf::Vector3& position, double position_weight, const tf::Vector3& axis, const tf::Vector3& direction, double angle, double weight = 1.0)
        : LinkGoalBase(link_name, weight)
        , position(position)
        , position_weight(position_weight)
        , axis(axis)
        , direction(direction)
        , angle(angle)
    {
    }
    virtual double evaluate(const GoalContext& context) const
    {
        double sum = 0.0;
        auto& fb = context.getLinkFrame();
        Vector3 v;
        quat_mul_vec(fb.getOrientation(), axis, v);
        double d = fmax(0.0, v.angle(direction) - angle);
        sum += d * d;
        double w = position_weight;
        sum += w * w * (position - fb.getPosition()).length2();
        return sum;
    }
};
}
