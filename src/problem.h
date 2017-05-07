#pragma once

#include "utils.h"
#include <vector>

#include "robot_info.h"

#include <geometric_shapes/shapes.h>

#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

#include <bio_ik/bio_ik.h>

namespace bio_ik
{

class Problem
{
private:
    std::vector<int> joint_usage;
    std::vector<ssize_t> link_tip_indices;
    std::vector<double> minimal_displacement_factors;
    std::vector<double> joint_transmission_goal_temp, joint_transmission_goal_temp2;
    MoveItRobotModelConstPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    ros::NodeHandle node_handle;
    RobotInfo modelInfo;
    double dpos, drot, dtwist;
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
    std::vector<CollisionLink> collision_links;

public:
    enum class GoalType;
    struct GoalInfo
    {
        const Goal* goal;
        GoalType goal_type;
        size_t tip_index;
        double weight;
        double weight_sq;
        double rotation_scale;
        double rotation_scale_sq;
        Frame frame;
        tf::Vector3 target;
        tf::Vector3 axis;
        double distance;
        ssize_t active_variable_index;
        double variable_position;
        std::vector<ssize_t> variable_indices;
        mutable size_t last_collision_vertex;
    };
    double timeout;
    std::vector<double> initial_guess;
    std::vector<size_t> active_variables;
    std::vector<size_t> tip_link_indices;
    std::vector<GoalInfo> goals;
    std::vector<GoalInfo> secondary_goals;
    void initialize(MoveItRobotModelConstPtr robot_model, const moveit::core::JointModelGroup* joint_model_group, ros::NodeHandle node_handle, const std::vector<const Goal*>& goals2);
    double computeGoalFitness(const GoalInfo& goal, const Frame* tip_frames, const double* active_variable_positions);
    double computeGoalFitness(const std::vector<GoalInfo>& goals, const Frame* tip_frames, const double* active_variable_positions);
    bool checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions);
};
}
