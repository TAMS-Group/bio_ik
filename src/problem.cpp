// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#include "ik_base.h"

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>

#include <unordered_set>

#include <mutex>

namespace bio_ik
{

enum class Problem::GoalType
{
    Null,
    Position,
    Orientation,
    Pose,
    LookAt,
    MaxDistance,
    MinDistance,
    Touch,
    Line,
    AvoidJointLimits,
    MinimalDisplacement,
    JointVariable,
    CenterJoints,
    JointFunction,
    Balance,
    LinkFunction,
    Side,
    Direction,
    Cone,
};

size_t Problem::addTipLink(const moveit::core::LinkModel* link_model)
{
    if(link_tip_indices[link_model->getLinkIndex()] < 0)
    {
        link_tip_indices[link_model->getLinkIndex()] = tip_link_indices.size();
        tip_link_indices.push_back(link_model->getLinkIndex());
    }
    return link_tip_indices[link_model->getLinkIndex()];
}

Problem::Problem()
    : ros_params_initrd(false)
{
}

void Problem::initialize(MoveItRobotModelConstPtr robot_model, const moveit::core::JointModelGroup* joint_model_group, ros::NodeHandle node_handle, const std::vector<const Goal*>& goals2, const BioIKKinematicsQueryOptions* options)
{
    if(robot_model != this->robot_model)
    {
        modelInfo = RobotInfo(robot_model);
        collision_links.clear();
        collision_links.resize(robot_model->getLinkModelCount());
    }

    this->robot_model = robot_model;
    this->joint_model_group = joint_model_group;
    this->node_handle = node_handle;

    if(!ros_params_initrd)
    {
        ros_params_initrd = true;
        auto& n = node_handle;
        n.param("dpos", dpos, DBL_MAX);
        n.param("drot", drot, DBL_MAX);
        n.param("dtwist", dtwist, 1e-5);
        if(dpos < 0.0 || dpos >= FLT_MAX || !std::isfinite(dpos)) dpos = DBL_MAX;
        if(drot < 0.0 || drot >= FLT_MAX || !std::isfinite(drot)) drot = DBL_MAX;
        if(dtwist < 0.0 || dtwist >= FLT_MAX || !std::isfinite(dtwist)) dtwist = DBL_MAX;
    }

    link_tip_indices.clear();
    link_tip_indices.resize(robot_model->getLinkModelCount(), -1);
    tip_link_indices.clear();
    
    active_variables.clear();
    auto addActiveVariable = [this, robot_model, joint_model_group, options] (const std::string& name) -> ssize_t
    {
        if(options)
        {
            auto& joint_name = robot_model->getJointOfVariable(name)->getName();
            for(auto& fixed_joint_name : options->fixed_joints)
            {
                if(fixed_joint_name == joint_name)
                {
                    return (ssize_t)-1 - (ssize_t)robot_model->getVariableIndex(name);
                }
            }
        }
        for(size_t i = 0; i < active_variables.size(); i++)
            if(name == robot_model->getVariableNames()[active_variables[i]])
                return i;
        for(auto& n : joint_model_group->getVariableNames())
        {
            if(n == name)
            {
                active_variables.push_back(robot_model->getVariableIndex(name));
                return active_variables.size() - 1;
            }
        }
        ERROR("joint variable not found", name);
    };

    goals.clear();
    secondary_goals.clear();
    for(auto& goal : goals2)
    {
        GoalInfo goal_info;

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
            auto* link_model = robot_model->getLinkModel(g->link_name);
            if(!link_model) ERROR("link not found", g->link_name);
            goal_info.tip_index = addTipLink(link_model);
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
            goal_info.last_collision_vertex = 0;
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
            goal_info.active_variable_index = addActiveVariable(g->variable_name);
            goal_info.variable_position = g->variable_position;
            secondary = g->secondary;
        }

        if(auto* g = dynamic_cast<const JointFunctionGoal*>(goal))
        {
            goal_info.goal_type = GoalType::JointFunction;
            for(auto& variable_name : g->variable_names)
                goal_info.variable_indices.push_back(addActiveVariable(variable_name));
            secondary = g->secondary;
        }

        if(auto* g = dynamic_cast<const BalanceGoal*>(goal))
        {
            // LOG("a");
            goal_info.goal_type = GoalType::Balance;
            goal_info.balance_goal_infos.resize(robot_model->getLinkModelCount());
            for(auto& b : goal_info.balance_goal_infos)
            {
                b.tip_index = -1;
                b.center = Vector3(0.0, 0.0, 0.0);
                b.mass = 0.0;
            }
            // LOG("1");
            for(auto link_model : robot_model->getLinkModels())
            {
                auto link_urdf = robot_model->getURDF()->getLink(link_model->getName());
                if(!link_urdf) continue;
                if(!link_urdf->inertial) continue;
                const auto& center_urdf = link_urdf->inertial->origin.position;
                tf::Vector3 center(center_urdf.x, center_urdf.y, center_urdf.z);
                double mass = link_urdf->inertial->mass;
                if(!(mass > 0)) continue;
                goal_info.balance_goal_infos[link_model->getLinkIndex()].tip_index = addTipLink(link_model);
                goal_info.balance_goal_infos[link_model->getLinkIndex()].center = center;
                goal_info.balance_goal_infos[link_model->getLinkIndex()].mass = mass;
            }
            // LOG("2");
            goal_info.balance_goal_infos.erase(std::remove_if(goal_info.balance_goal_infos.begin(), goal_info.balance_goal_infos.end(), [](const BalanceGoalInfo& b) { return b.tip_index < 0; }), goal_info.balance_goal_infos.end());
            double mass_sum = 0.0;
            for(auto& info : goal_info.balance_goal_infos)
                mass_sum += info.mass;
            for(auto& info : goal_info.balance_goal_infos)
                info.mass /= mass_sum;
            goal_info.target = g->center;
            goal_info.axis = g->axis;
            // LOG("b");
        }
        
        if(auto* g = dynamic_cast<const LinkFunctionGoal*>(goal))
        {
            goal_info.goal_type = GoalType::LinkFunction;
        }
        
        if(auto* g = dynamic_cast<const SideGoal*>(goal))
        {
            goal_info.goal_type = GoalType::Side;
            goal_info.axis = g->axis.normalized();
            goal_info.direction = g->direction.normalized();
        }
        
        if(auto* g = dynamic_cast<const DirectionGoal*>(goal))
        {
            goal_info.goal_type = GoalType::Direction;
            goal_info.axis = g->axis.normalized();
            goal_info.direction = g->direction.normalized();
        }
        
        if(auto* g = dynamic_cast<const ConeGoal*>(goal))
        {
            goal_info.goal_type = GoalType::Cone;
            goal_info.axis = g->axis.normalized();
            goal_info.direction = g->direction.normalized();
            goal_info.distance = g->angle;
            goal_info.frame.pos = g->position;
        }

        goal_info.rotation_scale_sq = goal_info.rotation_scale * goal_info.rotation_scale;
        goal_info.weight_sq = goal_info.weight * goal_info.weight;

        if(secondary)
            secondary_goals.push_back(goal_info);
        else
            goals.push_back(goal_info);
    }

    // update active variables from active subtree
    joint_usage.resize(robot_model->getJointModelCount());
    for(auto& u : joint_usage)
        u = 0;
    for(auto tip_index : tip_link_indices)
        for(auto* link_model = robot_model->getLinkModels()[tip_index]; link_model; link_model = link_model->getParentLinkModel())
            joint_usage[link_model->getParentJointModel()->getJointIndex()] = 1;
    if(options)
        for(auto& fixed_joint_name : options->fixed_joints)
            joint_usage[robot_model->getJointModel(fixed_joint_name)->getJointIndex()] = 0;
    for(auto* joint_model : joint_model_group->getActiveJointModels())
        if(joint_usage[joint_model->getJointIndex()] && !joint_model->getMimic())
            for(auto& n : joint_model->getVariableNames())
                addActiveVariable(n);
    
    // init weights for minimal displacement goals
    {
        minimal_displacement_factors.resize(active_variables.size());
        double s = 0;
        for(auto ivar : active_variables)
            s += modelInfo.getMaxVelocityRcp(ivar);
        if(s > 0)
        {
            for(size_t i = 0; i < active_variables.size(); i++)
            {
                auto ivar = active_variables[i];
                minimal_displacement_factors[i] = modelInfo.getMaxVelocityRcp(ivar) / s;
            }
        }
        else
        {
            for(size_t i = 0; i < active_variables.size(); i++)
                minimal_displacement_factors[i] = 1.0 / active_variables.size();
        }
    }
}

double Problem::computeGoalFitness(const GoalInfo& goal, const Frame* tip_frames, const double* active_variable_positions)
{
    double sum = 0.0;

    // LOG("computeGoalFitness", tip_frames[0]);

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
        double offset = 10000;
        fcl::Sphere shape1(offset);
        const TouchGoal* touch_goal = (TouchGoal*)goal.goal;
        double dmin = DBL_MAX;
        size_t link_index = tip_link_indices[goal.tip_index];
        auto* link_model = robot_model->getLinkModel(link_index);
        auto touch_goal_normal = touch_goal->normal.normalized();
        // auto fbrot = fb.rot.normalized();
        auto& collision_link = collision_links[link_index];
        if(!collision_link.initialized)
        {
            collision_link.initialized = true;
            collision_link.shapes.resize(link_model->getShapes().size());
            for(size_t shape_index = 0; shape_index < link_model->getShapes().size(); shape_index++)
            {
                collision_link.shapes[shape_index] = std::make_shared<CollisionShape>();
                auto& s = *collision_link.shapes[shape_index];
                s.frame = Frame(link_model->getCollisionOriginTransforms()[shape_index]);
                auto* shape = link_model->getShapes()[shape_index].get();
                // LOG(link_model->getName(), shape_index, link_model->getShapes().size(), typeid(*shape).name());
                if(auto* mesh = dynamic_cast<const shapes::Mesh*>(shape))
                {
                    struct : bodies::ConvexMesh
                    {
                        std::vector<fcl::Vec3f> points;
                        std::vector<int> polygons;
                        std::vector<fcl::Vec3f> plane_normals;
                        std::vector<double> plane_dis;
                        void init(const shapes::Shape* shape)
                        {
                            type_ = shapes::MESH;
                            scaled_vertices_ = 0;
                            {
                                static std::mutex mutex;
                                std::lock_guard<std::mutex> lock(mutex);
                                setDimensions(shape);
                            }
                            for(auto& v : mesh_data_->vertices_)
                                points.emplace_back(v.x(), v.y(), v.z());
                            for(size_t triangle_index = 0; triangle_index < mesh_data_->triangles_.size() / 3; triangle_index++)
                            {
                                polygons.push_back(3);
                                polygons.push_back(mesh_data_->triangles_[triangle_index * 3 + 0]);
                                polygons.push_back(mesh_data_->triangles_[triangle_index * 3 + 1]);
                                polygons.push_back(mesh_data_->triangles_[triangle_index * 3 + 2]);
                            }
                            for(size_t triangle_index = 0; triangle_index < mesh_data_->triangles_.size() / 3; triangle_index++)
                            {
                                auto plane_index = mesh_data_->plane_for_triangle_[triangle_index];
                                auto plane = mesh_data_->planes_[plane_index];
                                plane_normals.emplace_back(plane.x(), plane.y(), plane.z());
                                plane_dis.push_back(plane.w());
                            }
                        }
                    } convex;
                    convex.init(mesh);
                    s.points = convex.points;
                    s.polygons = convex.polygons;
                    s.plane_normals = convex.plane_normals;
                    s.plane_dis = convex.plane_dis;

                    //auto* fcl = new fcl::Convex(s.plane_normals.data(), s.plane_dis.data(), s.plane_normals.size(), s.points.data(), s.points.size(), s.polygons.data());

                    // workaround for fcl::Convex initialization bug
                    auto* fcl = (fcl::Convex*) ::operator new(sizeof(fcl::Convex));
                    fcl->num_points = s.points.size();
                    fcl = new(fcl) fcl::Convex(s.plane_normals.data(), s.plane_dis.data(), s.plane_normals.size(), s.points.data(), s.points.size(), s.polygons.data());

                    s.geometry = decltype(s.geometry)(new collision_detection::FCLGeometry(fcl, link_model, shape_index));
                    s.edges.resize(s.points.size());
                    std::vector<std::unordered_set<size_t>> edge_sets(s.points.size());
                    for(size_t edge_index = 0; edge_index < fcl->num_edges; edge_index++)
                    {
                        auto edge = fcl->edges[edge_index];
                        if(edge_sets[edge.first].find(edge.second) == edge_sets[edge.first].end())
                        {
                            edge_sets[edge.first].insert(edge.second);
                            s.edges[edge.first].push_back(edge.second);
                        }
                        if(edge_sets[edge.second].find(edge.first) == edge_sets[edge.second].end())
                        {
                            edge_sets[edge.second].insert(edge.first);
                            s.edges[edge.second].push_back(edge.first);
                        }
                    }
                    for(auto& p : s.points)
                        s.vertices.emplace_back(p[0], p[1], p[2]);
                }
                else
                {
                    s.geometry = collision_detection::createCollisionGeometry(link_model->getShapes()[shape_index], link_model, shape_index);
                }
                // LOG("b");
            }
            // getchar();
        }
        BLOCKPROFILER("touch goal");
        for(size_t shape_index = 0; shape_index < link_model->getShapes().size(); shape_index++)
        {
            if(!collision_link.shapes[shape_index]->geometry) continue;
            auto* shape = link_model->getShapes()[shape_index].get();
            // LOG(shape_index, typeid(*shape).name());
            if(auto* mesh = dynamic_cast<const shapes::Mesh*>(shape))
            {
                auto& s = collision_link.shapes[shape_index];
                double d = DBL_MAX;
                auto goal_normal = touch_goal->normal;
                quat_mul_vec(fb.rot.inverse(), goal_normal, goal_normal);
                quat_mul_vec(s->frame.rot.inverse(), goal_normal, goal_normal);
                /*{
                    size_t array_index = 0;
                    for(size_t vertex_index = 0; vertex_index < mesh->vertex_count; vertex_index++)
                    {
                        double dot_x = mesh->vertices[array_index++] * goal_normal.x();
                        double dot_y = mesh->vertices[array_index++] * goal_normal.y();
                        double dot_z = mesh->vertices[array_index++] * goal_normal.z();
                        double e = dot_x + dot_y + dot_z;
                        if(e < d) d = e;
                    }
                }*/
                if(mesh->vertex_count > 0)
                {
                    size_t vertex_index = goal.last_collision_vertex;
                    double vertex_dot_normal = goal_normal.dot(s->vertices[vertex_index]);
                    // size_t loops = 0;
                    while(true)
                    {
                        bool repeat = false;
                        for(auto vertex_index_2 : s->edges[vertex_index])
                        {
                            auto vertex_dot_normal_2 = goal_normal.dot(s->vertices[vertex_index_2]);
                            if(vertex_dot_normal_2 < vertex_dot_normal)
                            {
                                vertex_index = vertex_index_2;
                                vertex_dot_normal = vertex_dot_normal_2;
                                repeat = true;
                                break;
                            }
                        }
                        if(!repeat) break;
                        // loops++;
                    }
                    // LOG_VAR(loops);
                    d = vertex_dot_normal;
                    goal.last_collision_vertex = vertex_index;
                }
                d -= touch_goal->normal.dot(touch_goal->position - fb.pos);
                if(d < dmin) dmin = d;
            }
            else
            {
                fcl::DistanceRequest request;
                fcl::DistanceResult result;
                auto pos1 = touch_goal->position - touch_goal_normal * offset * 2;
                auto* shape2 = collision_link.shapes[shape_index]->geometry->collision_geometry_.get();
                auto frame2 = Frame(fb.pos, fb.rot.normalized()) * collision_link.shapes[shape_index]->frame;
                double d = fcl::distance(&shape1, fcl::Transform3f(fcl::Vec3f(pos1.x(), pos1.y(), pos1.z())), shape2, fcl::Transform3f(fcl::Quaternion3f(frame2.rot.w(), frame2.rot.x(), frame2.rot.y(), frame2.rot.z()), fcl::Vec3f(frame2.pos.x(), frame2.pos.y(), frame2.pos.z())), request, result);
                d -= offset;
                // LOG(link_model->getName(), shape_index, d);
                if(d < dmin) dmin = d;
            }
        }
        if(dmin < DBL_MAX)
        {
            // dmin -= touch_goal_normal.dot(touch_goal->position);
            sum += dmin * dmin * goal.weight_sq;
        }
        break;
    }

    case GoalType::JointVariable:
    {
        if(goal.active_variable_index < 0) break;
        double v;
        if(goal.active_variable_index >= 0)
            v = active_variable_positions[goal.active_variable_index];
        else
            v = initial_guess[-1 - goal.active_variable_index];
        double d = v - goal.variable_position;
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
            double x = active_variable_positions[i] - initial_guess[ivar];
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
        //for(size_t i = 0; i < goal.variable_indices.size(); i++)
        //    joint_transmission_goal_temp[i] = active_variable_positions[goal.variable_indices[i]];
        for(size_t i = 0; i < goal.variable_indices.size(); i++)
        {
            ssize_t vi = goal.variable_indices[i];
            if(vi >= 0)
                joint_transmission_goal_temp[i] = active_variable_positions[vi];
            else
                joint_transmission_goal_temp[i] = initial_guess[-1 - vi];
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

    case GoalType::Balance:
    {
        Vector3 center;
        for(auto& info : goal.balance_goal_infos)
        {
            auto& frame = tip_frames[info.tip_index];
            auto c = info.center;
            quat_mul_vec(frame.rot, c, c);
            c += frame.pos;
            center += c * info.mass;
        }
        center -= goal.target;
        center -= goal.axis * goal.axis.dot(center);
        sum += center.length2() * goal.weight_sq;
        break;
    }
    
    case GoalType::LinkFunction:
    {
        sum += ((LinkFunctionGoal*)goal.goal)->function(fb.pos, fb.rot) * goal.weight_sq;
        break;
    }
    
    case GoalType::Side:
    {
        Vector3 v;
        quat_mul_vec(fb.rot, goal.axis, v);
        double f = fmax(0.0, v.dot(goal.direction));
        sum += f * f * goal.weight_sq;
        break;
    }
    
    case GoalType::Direction:
    {
        Vector3 v;
        quat_mul_vec(fb.rot, goal.axis, v);
        sum += v.distance2(goal.direction) * goal.weight_sq;
        break;
    }
    
    case GoalType::Cone:
    {
        Vector3 v;
        quat_mul_vec(fb.rot, goal.axis, v);
        double d = fmax(0.0, v.angle(goal.direction) - goal.distance);
        sum += d * d * goal.weight_sq;
        double w = ((const ConeGoal*)goal.goal)->position_weight;
        sum += goal.weight_sq * w * w * (fa.pos - fb.pos).length2();
    }

    default:
    {
        break;
    }
    }

    // LOG("computeGoalFitness", sum);

    return sum;
}

double Problem::computeGoalFitness(const std::vector<GoalInfo>& goals, const Frame* tip_frames, const double* active_variable_positions)
{
    double sum = 0.0;
    for(auto& goal : goals)
        sum += computeGoalFitness(goal, tip_frames, active_variable_positions);
    return sum;
}

bool Problem::checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
{
    for(auto& goal : goals)
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

    // LOG("checkSolutionActiveVariables true");

    return true;
}
}
