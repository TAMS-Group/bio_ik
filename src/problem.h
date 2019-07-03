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

#include "utils.h"
#include <vector>

#include <bio_ik/robot_info.h>

#include <geometric_shapes/shapes.h>

#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection_fcl/collision_robot_fcl.h>

#include <bio_ik/goal.h>

namespace bio_ik
{

class Problem
{
private:
    bool ros_params_initrd;
    std::vector<int> joint_usage;
    std::vector<ssize_t> link_tip_indices;
    std::vector<double> minimal_displacement_factors;
    std::vector<double> joint_transmission_goal_temp, joint_transmission_goal_temp2;
    moveit::core::RobotModelConstPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    IKParams params;
    RobotInfo modelInfo;
    double dpos, drot, dtwist;
#if (MOVEIT_FCL_VERSION < FCL_VERSION_CHECK(0, 6, 0))
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
#endif
    size_t addTipLink(const moveit::core::LinkModel* link_model);

public:
    /*enum class GoalType;
    struct BalanceGoalInfo
    {
        ssize_t tip_index;
        double mass;
        Vector3 center;
    };
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
        tf2::Vector3 target;
        tf2::Vector3 direction;
        tf2::Vector3 axis;
        double distance;
        ssize_t active_variable_index;
        double variable_position;
        std::vector<ssize_t> variable_indices;
        mutable size_t last_collision_vertex;
        std::vector<BalanceGoalInfo> balance_goal_infos;
    };*/
    enum class GoalType;
    // std::vector<const Frame*> temp_frames;
    // std::vector<double> temp_variables;
    struct GoalInfo
    {
        const Goal* goal;
        double weight_sq;
        double weight;
        GoalType goal_type;
        size_t tip_index;
        Frame frame;
        GoalContext goal_context;
    };
    double timeout;
    std::vector<double> initial_guess;
    std::vector<size_t> active_variables;
    std::vector<size_t> tip_link_indices;
    std::vector<GoalInfo> goals;
    std::vector<GoalInfo> secondary_goals;
    Problem();
    void initialize(moveit::core::RobotModelConstPtr robot_model, const moveit::core::JointModelGroup* joint_model_group, const IKParams& params, const std::vector<const Goal*>& goals2, const BioIKKinematicsQueryOptions* options);
    void initialize2();
    double computeGoalFitness(GoalInfo& goal, const Frame* tip_frames, const double* active_variable_positions);
    double computeGoalFitness(std::vector<GoalInfo>& goals, const Frame* tip_frames, const double* active_variable_positions);
    bool checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions);
};
}
