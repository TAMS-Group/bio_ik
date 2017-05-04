// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#include "ik_base.h"

#include <geometric_shapes/shapes.h>

namespace bio_ik
{

void IKRequest::updateActiveVariables(const IKParams& params)
{
    auto& robot_model = params.robot_model;
    auto* joint_model_group = params.joint_model_group;
    joint_usage.resize(robot_model->getJointModelCount());
    for(auto& u : joint_usage)
        u = 0;
    for(auto tip_index : tip_link_indices)
        for(auto* link_model = robot_model->getLinkModels()[tip_index]; link_model; link_model = link_model->getParentLinkModel())
            joint_usage[link_model->getParentJointModel()->getJointIndex()] = 1;
    active_variables.clear();
    for(auto* joint_model : joint_model_group->getActiveJointModels())
        if(joint_usage[joint_model->getJointIndex()] && !joint_model->getMimic())
            for(size_t ivar = joint_model->getFirstVariableIndex(); ivar < joint_model->getFirstVariableIndex() + joint_model->getVariableCount(); ivar++)
                active_variables.push_back(ivar);
}

void IKRequest::setGoals(const std::vector<const Goal*>& goals2, const IKParams& params)
{
    link_tip_indices.clear();
    link_tip_indices.resize(params.robot_model->getLinkModelCount(), -1);
    tip_link_indices.clear();

    goals.clear();
    secondary_goals.clear();
    for(auto& goal : goals2)
    {
        IKGoalInfo goal_info;

        goal_info.goal = goal;

        goal_info.weight = 1;
        goal_info.rotation_scale = 0.5;
        goal_info.frame = Frame::identity();
        goal_info.goal_type = GoalType::Null;

        bool secondary = false;

        if(auto* g = dynamic_cast<const GoalBase*>(goal))
        {
            goal_info.weight = g->weight;
            if(goal_info.weight <= 0) continue;
        }

        if(auto* g = dynamic_cast<const LinkGoalBase*>(goal))
        {
            auto* link_model = params.robot_model->getLinkModel(g->link_name);
            if(!link_model) ERROR("link not found", g->link_name);
            if(link_tip_indices[link_model->getLinkIndex()] < 0)
            {
                link_tip_indices[link_model->getLinkIndex()] = tip_link_indices.size();
                tip_link_indices.push_back(link_model->getLinkIndex());
            }
            goal_info.tip_index = link_tip_indices[link_model->getLinkIndex()];
        }

        if(auto* g = dynamic_cast<const PositionGoal*>(goal))
        {
            goal_info.goal_type = GoalType::Position;
            goal_info.frame.pos = g->position;
        }

        if(auto* g = dynamic_cast<const OrientationGoal*>(goal))
        {
            goal_info.goal_type = GoalType::Orientation;
            goal_info.frame.rot = g->orientation.normalized();
        }

        if(auto* g = dynamic_cast<const PoseGoal*>(goal))
        {
            goal_info.goal_type = GoalType::Pose;
            goal_info.frame.pos = g->position;
            goal_info.frame.rot = g->orientation.normalized();
            goal_info.rotation_scale = g->rotation_scale;
            if(!(goal_info.rotation_scale > 0)) goal_info.goal_type = GoalType::Position;

            /*goal_info.goal_type = GoalType::MaxDistance;
            goal_info.target = g->position;
            goal_info.distance = 0.2;*/
        }

        if(auto* g = dynamic_cast<const LookAtGoal*>(goal))
        {
            goal_info.goal_type = GoalType::LookAt;
            goal_info.target = g->target;
            goal_info.axis = g->axis.normalized();
        }

        if(auto* g = dynamic_cast<const MaxDistanceGoal*>(goal))
        {
            goal_info.goal_type = GoalType::MaxDistance;
            goal_info.target = g->target;
            goal_info.distance = g->distance;
        }

        if(auto* g = dynamic_cast<const MinDistanceGoal*>(goal))
        {
            goal_info.goal_type = GoalType::MinDistance;
            goal_info.target = g->target;
            goal_info.distance = g->distance;
        }

        if(auto* g = dynamic_cast<const LineGoal*>(goal))
        {
            goal_info.goal_type = GoalType::Line;
        }

        if(auto* g = dynamic_cast<const TouchGoal*>(goal))
        {
            goal_info.goal_type = GoalType::Touch;
        }

        if(auto* g = dynamic_cast<const AvoidJointLimitsGoal*>(goal))
        {
            goal_info.goal_type = GoalType::AvoidJointLimits;
            secondary = g->secondary;
        }

        if(auto* g = dynamic_cast<const CenterJointsGoal*>(goal))
        {
            goal_info.goal_type = GoalType::CenterJoints;
            secondary = g->secondary;
        }

        if(auto* g = dynamic_cast<const MinimalDisplacementGoal*>(goal))
        {
            goal_info.goal_type = GoalType::MinimalDisplacement;
            secondary = g->secondary;
        }

        if(auto* g = dynamic_cast<const JointVariableGoal*>(goal))
        {
            goal_info.goal_type = GoalType::JointVariable;
            goal_info.active_variable_index = -1;
            /*for(size_t i = 0; i < active_variables.size(); i++)
            {
                LOG(params.robot_model->getVariableNames()[active_variables[i]]);
                if(params.robot_model->getVariableNames()[active_variables[i]] == g->variable_name)
                {
                    goal_info.active_variable_index = i;
                }
            }
            if(goal_info.active_variable_index < 0)
            {
                //continue;
                ERROR("joint variable not found", g->variable_name);
            }*/
            goal_info.variable_position = g->variable_position;
            secondary = g->secondary;
        }

        if(auto* g = dynamic_cast<const JointFunctionGoal*>(goal))
        {
            goal_info.goal_type = GoalType::JointFunction;
            secondary = g->secondary;
        }

        goal_info.rotation_scale_sq = goal_info.rotation_scale * goal_info.rotation_scale;
        goal_info.weight_sq = goal_info.weight * goal_info.weight;

        // LOG_VAR(goal_info.tip_index);

        /*LOG_VAR(goal_info.rotation_scale);
        LOG_VAR(goal_info.tip_index);
        LOG_VAR(goal_info.weight);
        LOG("goal_info.frame.pos", goal_info.frame.pos.x(), goal_info.frame.pos.y(), goal_info.frame.pos.z());
        LOG("goal_info.frame.rot", goal_info.frame.rot.x(), goal_info.frame.rot.y(), goal_info.frame.rot.z(), goal_info.frame.rot.w());*/

        if(secondary)
            secondary_goals.push_back(goal_info);
        else
            goals.push_back(goal_info);
    }

    // std::sort(goals.begin(), goals.end(), [] (const IKGoalInfo& a, const IKGoalInfo& b) { return a.goal_type < b.goal_type; });

    updateActiveVariables(params);

    /*LOG_VAR(request.active_variables.size());
    for(auto& active_variable : request.active_variables) LOG_VAR(active_variable);

    LOG_VAR(request.tip_link_indices.size());
    for(auto& tli : request.tip_link_indices) LOG_VAR(tli);*/

    for(auto* pgg : {&goals, &secondary_goals})
    {
        for(auto& goal_info : *pgg)
        {
            if(goal_info.goal_type == GoalType::JointVariable)
            {
                for(size_t i = 0; i < active_variables.size(); i++)
                {
                    if(params.robot_model->getVariableNames()[active_variables[i]] == ((const JointVariableGoal*)goal_info.goal)->variable_name)
                    {
                        goal_info.active_variable_index = i;
                        break;
                    }
                }
            }

            if(goal_info.goal_type == GoalType::JointFunction)
            {
                auto* g = dynamic_cast<const JointFunctionGoal*>(goal_info.goal);
                for(auto& variable_name : g->variable_names)
                {
                    for(size_t i = 0; i < active_variables.size(); i++)
                    {
                        if(params.robot_model->getVariableNames()[active_variables[i]] == variable_name)
                        {
                            goal_info.variable_indices.push_back(i);
                            goto ok;
                        }
                    }

                    for(ssize_t i = 0; i < params.robot_model->getVariableCount(); i++)
                    {
                        if(params.robot_model->getVariableNames()[i] == variable_name)
                        {
                            goal_info.variable_indices.push_back(-1 - i);
                            goto ok;
                        }
                    }

                    ERROR("joint not found", variable_name);

                ok:;
                }
            }
        }
    }
}

double IKBase::computeGoalFitness(const IKGoalInfo& goal, const Frame* tip_frames, const double* active_variable_positions)
{
    double sum = 0.0;

    const auto& fa = goal.frame;
    const auto& fb = tip_frames[goal.tip_index];

    switch(goal.goal_type)
    {

    case GoalType::Position:
    {
        sum += goal.weight_sq * (fa.pos - fb.pos).length2();
        break;
    }

    case GoalType::Orientation:
    {
        sum += goal.weight_sq * (fa.rot - fa.rot.nearest(fb.rot)).length2();
        break;
    }

    case GoalType::Pose:
    {
        sum += goal.weight_sq * (fa.pos - fb.pos).length2();
        sum += goal.weight_sq * (fa.rot - fa.rot.nearest(fb.rot)).length2() * goal.rotation_scale_sq;
        break;
    }

    case GoalType::LookAt:
    {
        tf::Vector3 axis;
        quat_mul_vec(fb.rot, goal.axis, axis);
        // sum += (fb.pos + axis * axis.dot(goal.target - fb.pos)).distance2(goal.target) * goal.weight_sq / goal.target.distance2(fb.pos);
        sum += (goal.target - fb.pos).normalized().distance2(axis.normalized()) * goal.weight_sq;
        break;
    }

    case GoalType::MaxDistance:
    {
        double d = fmax(0.0, fb.pos.distance(goal.target) - goal.distance);
        sum += d * d * goal.weight_sq;
        break;
    }

    case GoalType::MinDistance:
    {
        double d = fmax(0.0, goal.distance - fb.pos.distance(goal.target));
        sum += d * d * goal.weight_sq;
        break;
    }

    case GoalType::Line:
    {
        const LineGoal* line_goal = (LineGoal*)goal.goal;
        sum += goal.weight_sq * line_goal->position.distance2(fb.pos - line_goal->direction * line_goal->direction.dot(fb.pos - line_goal->position));
        break;
    }

    case GoalType::Touch:
    {
        BLOCKPROFILER("touch goal");
        const TouchGoal* touch_goal = (TouchGoal*)goal.goal;
        auto goal_normal = touch_goal->normal;
        auto rot_inv = fb.rot.inverse();
        quat_mul_vec(rot_inv, goal_normal, goal_normal);
        double dmin = DBL_MAX;
        auto* link_model = params.robot_model->getLinkModel(request.tip_link_indices[goal.tip_index]);
        for(size_t shape_index = 0; shape_index < link_model->getShapes().size(); shape_index++)
        {
            // TODO:
            if(!link_model->areCollisionOriginTransformsIdentity()[shape_index]) ERROR("Collision Mesh Offsets NYI");
            // LOG("A");
            if(auto* mesh = dynamic_cast<const shapes::Mesh*>(link_model->getShapes()[shape_index].get()))
            {
                // LOG("MESH");
                // LOG(mesh->vertex_count);
                size_t array_index = 0;
                for(size_t vertex_index = 0; vertex_index < mesh->vertex_count; vertex_index++)
                {
                    /*tf::Vector3 vertex;
                    vertex.setX(mesh->vertices[vertex_index * 3 + 0]);
                    vertex.setY(mesh->vertices[vertex_index * 3 + 1]);
                    vertex.setZ(mesh->vertices[vertex_index * 3 + 2]);
                    double d = goal_normal.dot(vertex - goal_position);*/
                    double dot_x = mesh->vertices[array_index++] * goal_normal.x();
                    double dot_y = mesh->vertices[array_index++] * goal_normal.y();
                    double dot_z = mesh->vertices[array_index++] * goal_normal.z();
                    double d = dot_x + dot_y + dot_z;
                    if(d < dmin) dmin = d;
                    // break;
                }
            }
        }
        if(dmin < DBL_MAX)
        {
            // dmin -= goal_normal.dot(goal_position);
            dmin -= touch_goal->normal.dot(touch_goal->position - fb.pos);
            sum += dmin * dmin;
        }
        break;
    }

    case GoalType::JointVariable:
    {
        if(goal.active_variable_index < 0) break;
        double d = active_variable_positions[goal.active_variable_index] - goal.variable_position;
        sum += d * d * goal.weight_sq;
        break;
    }

    case GoalType::AvoidJointLimits:
    {
        for(size_t i = 0; i < active_variables.size(); i++)
        {
            size_t ivar = active_variables[i];
            if(modelInfo.getClipMax(ivar) == DBL_MAX) continue;
            double x = active_variable_positions[i] - (modelInfo.getMin(ivar) + modelInfo.getMax(ivar)) * 0.5;
            x = fmax(0.0, fabs(x) * 2.0 - modelInfo.getSpan(ivar) * 0.5);
            x *= minimal_displacement_factors[i];
            x *= goal.weight;
            sum += x * x;
        }
        break;
    }

    case GoalType::MinimalDisplacement:
    {
        for(size_t i = 0; i < active_variables.size(); i++)
        {
            // if(i >= 7) continue;
            size_t ivar = active_variables[i];
            double x = active_variable_positions[i] - request.initial_guess[ivar];
            x *= minimal_displacement_factors[i];
            x *= goal.weight;
            sum += x * x;
            // LOG(i, params.robot_model->getVariableNames()[active_variables[i]], minimal_displacement_factors[i]);
        }
        break;
    }

    case GoalType::CenterJoints:
    {
        for(size_t i = 0; i < active_variables.size(); i++)
        {
            size_t ivar = active_variables[i];
            if(modelInfo.getClipMax(ivar) == DBL_MAX) continue;
            double x = active_variable_positions[i] - (modelInfo.getMin(ivar) + modelInfo.getMax(ivar)) * 0.5;
            x *= minimal_displacement_factors[i];
            x *= goal.weight;
            sum += x * x;
        }
        break;
    }

    case GoalType::JointFunction:
    {
        joint_transmission_goal_temp.resize(goal.variable_indices.size());
        for(size_t i = 0; i < goal.variable_indices.size(); i++)
        {
            if(goal.variable_indices[i] >= 0)
                joint_transmission_goal_temp[i] = active_variable_positions[active_variables[goal.variable_indices[i]]];
            else
                joint_transmission_goal_temp[i] = request.initial_guess[-goal.variable_indices[i] - 1];
        }
        joint_transmission_goal_temp2 = joint_transmission_goal_temp;
        ((const JointFunctionGoal*)goal.goal)->function(joint_transmission_goal_temp2);
        for(size_t i = 0; i < goal.variable_indices.size(); i++)
        {
            double d = joint_transmission_goal_temp[i] - joint_transmission_goal_temp2[i];
            sum += d * d * goal.weight_sq;
        }
        break;
    }
    }

    return sum;
}

double IKBase::computeGoalFitness(const std::vector<IKGoalInfo>& goals, const Frame* tip_frames, const double* active_variable_positions)
{
    double sum = 0.0;
    for(auto& goal : goals)
        sum += computeGoalFitness(goal, tip_frames, active_variable_positions);
    return sum;
}

bool IKBase::checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
{
    for(auto& goal : request.goals)
    {
        const auto& fa = goal.frame;
        const auto& fb = tip_frames[goal.tip_index];

        switch(goal.goal_type)
        {

        case GoalType::Position:
        {
            if(dpos != DBL_MAX)
            {
                double p_dist = (fb.pos - fa.pos).length();
                if(!(p_dist <= dpos)) return false;
            }
            if(dtwist != DBL_MAX)
            {
                KDL::Frame fk_kdl, ik_kdl;
                frameToKDL(fa, fk_kdl);
                frameToKDL(fb, ik_kdl);
                KDL::Twist kdl_diff(fk_kdl.M.Inverse() * KDL::diff(fk_kdl.p, ik_kdl.p), fk_kdl.M.Inverse() * KDL::diff(fk_kdl.M, ik_kdl.M));
                if(!KDL::Equal(kdl_diff.vel, KDL::Twist::Zero().vel, dtwist)) return false;
            }
            continue;
        }

        case GoalType::Orientation:
        {
            if(drot != DBL_MAX)
            {
                double r_dist = fb.rot.angleShortestPath(fa.rot);
                r_dist = r_dist * 180 / M_PI;
                if(!(r_dist <= drot)) return false;
            }
            if(dtwist != DBL_MAX)
            {
                KDL::Frame fk_kdl, ik_kdl;
                frameToKDL(fa, fk_kdl);
                frameToKDL(fb, ik_kdl);
                KDL::Twist kdl_diff(fk_kdl.M.Inverse() * KDL::diff(fk_kdl.p, ik_kdl.p), fk_kdl.M.Inverse() * KDL::diff(fk_kdl.M, ik_kdl.M));
                if(!KDL::Equal(kdl_diff.rot, KDL::Twist::Zero().rot, dtwist)) return false;
            }
            continue;
        }

        case GoalType::Pose:
        {
            if(dpos != DBL_MAX || drot != DBL_MAX)
            {
                double p_dist = (fb.pos - fa.pos).length();
                double r_dist = fb.rot.angleShortestPath(fa.rot);
                r_dist = r_dist * 180 / M_PI;
                if(!(p_dist <= dpos)) return false;
                if(!(r_dist <= drot)) return false;
            }
            if(dtwist != DBL_MAX)
            {
                KDL::Frame fk_kdl, ik_kdl;
                frameToKDL(fa, fk_kdl);
                frameToKDL(fb, ik_kdl);
                KDL::Twist kdl_diff(fk_kdl.M.Inverse() * KDL::diff(fk_kdl.p, ik_kdl.p), fk_kdl.M.Inverse() * KDL::diff(fk_kdl.M, ik_kdl.M));
                if(!KDL::Equal(kdl_diff, KDL::Twist::Zero(), dtwist)) return false;
            }
            continue;
        }

        default:
        {
            double dmax = DBL_MAX;
            dmax = std::fmin(dmax, dpos);
            dmax = std::fmin(dmax, dtwist);
            double d = computeGoalFitness(goal, tip_frames.data(), active_variable_positions);
            if(!(d < dmax * dmax)) return false;
        }
        }
    }

    return true;
}
}
