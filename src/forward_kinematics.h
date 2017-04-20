// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#pragma once

#include "robot_helpers.h"

#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <emmintrin.h>
#include <immintrin.h>
#include <x86intrin.h>

namespace bio_ik
{

// computes and caches local joint frames
class RobotJointEvaluator
{
private:
    std::vector<double> joint_cache_variables;
    std::vector<Frame> joint_cache_frames;
    std::vector<Frame> link_frames;
protected:
    std::vector<Vector3> joint_axis_list;
    MoveItRobotModelConstPtr robot_model;
    std::vector<double> variables;
public:
    inline void getJointFrame(const moveit::core::JointModel* joint_model, const double* variables, Frame& frame)
    {
        auto joint_type = joint_model->getType();
        if(joint_type == moveit::core::JointModel::FIXED)
        {
            frame = Frame::identity();
            return;
        }
        size_t joint_index = joint_model->getJointIndex();
        switch(joint_type)
        {
        case moveit::core::JointModel::REVOLUTE:
        {
            auto axis = joint_axis_list[joint_index];

            auto v = variables[joint_model->getFirstVariableIndex()];

            /*v *= 1.0 / (2 * M_PI);
            v -= std::floor(v);
            v *= (2 * M_PI);*/

            auto half_angle = v * 0.5;

            // TODO: optimize sin cos

            auto fcos = cos(half_angle);
            auto fsin = sin(half_angle);

            //auto fsin = half_angle;
            //auto fcos = 1.0 - half_angle * half_angle * 0.5;

            frame = Frame(Vector3(0.0, 0.0, 0.0), Quaternion(axis.x() * fsin, axis.y() * fsin, axis.z() * fsin, fcos));

            break;
        }
        case moveit::core::JointModel::PRISMATIC:
        {
            auto axis = joint_axis_list[joint_index];
            auto v = variables[joint_model->getFirstVariableIndex()];
            frame = Frame(axis * v, Quaternion(0.0, 0.0, 0.0, 1.0));
            break;
        }
        case moveit::core::JointModel::FLOATING:
        {
            auto* vv = variables + joint_model->getFirstVariableIndex();
            frame.pos = Vector3(vv[0], vv[1], vv[2]);
            frame.rot = Quaternion(vv[3], vv[4], vv[5], vv[6]).normalized();
            //LOG("floating", joint_model->getFirstVariableIndex(), vv[0], vv[1], vv[2], vv[3], vv[4], vv[5], vv[6]);
            break;
        }
        default:
        {
            auto* joint_variables = variables + joint_model->getFirstVariableIndex();
            Eigen::Affine3d joint_transform;
            joint_model->computeTransform(joint_variables, joint_transform);
            frame = Frame(joint_transform);
            break;
        }
        }

        //LOG("local joint frame", joint_model->getType(), joint_model->getName(), frame);
    }
    inline void getJointFrame(const moveit::core::JointModel* joint_model, const std::vector<double>& variables, Frame& frame)
    {
        getJointFrame(joint_model, variables.data(), frame);
    }
protected:
    inline const Frame& getLinkFrame(const moveit::core::LinkModel* link_model)
    {
        return link_frames[link_model->getLinkIndex()];
    }



    inline bool checkJointMoved(const moveit::core::JointModel* joint_model)
    {
        size_t i0 = joint_model->getFirstVariableIndex();
        size_t cnt = joint_model->getVariableCount();
        if(cnt == 0) return true;
        if(cnt == 1) return !(variables[i0] == joint_cache_variables[i0]);
        for(size_t i = i0; i < i0 + cnt; i++)
            if(!(variables[i] == joint_cache_variables[i]))
                return true;
        return false;
    }
    inline void putJointCache(const moveit::core::JointModel* joint_model, const Frame& frame)
    {
        joint_cache_frames[joint_model->getJointIndex()] = frame;
        size_t i0 = joint_model->getFirstVariableIndex();
        size_t cnt = joint_model->getVariableCount();
        for(size_t i = i0; i < i0 + cnt; i++)
            joint_cache_variables[i] = variables[i];
    }
    inline const Frame& getJointFrame(const moveit::core::JointModel* joint_model)
    {
        size_t joint_index = joint_model->getJointIndex();

        if(!checkJointMoved(joint_model)) return joint_cache_frames[joint_index];

        getJointFrame(joint_model, variables, joint_cache_frames[joint_index]);

        size_t cnt = joint_model->getVariableCount();
        if(cnt)
        {
            size_t i0 = joint_model->getFirstVariableIndex();
            if(cnt == 1)
                joint_cache_variables[i0] = variables[i0];
            else
                for(size_t i = i0; i < i0 + cnt; i++)
                    joint_cache_variables[i] = variables[i];
        }

        // TODO: optimize copy

        /*size_t cnt = joint_model->getVariableCount();
        size_t i0 = joint_model->getFirstVariableIndex();
        memcpy(joint_cache_variables.data() + i0, variables.data() + i0, cnt * 8);*/

        return joint_cache_frames[joint_index];
    }

public:
    RobotJointEvaluator(MoveItRobotModelConstPtr model) : robot_model(model)
    {
        joint_cache_variables.clear();
        joint_cache_variables.resize(model->getVariableCount(), DBL_MAX);

        joint_cache_frames.clear();
        joint_cache_frames.resize(model->getJointModelCount());

        link_frames.clear();
        for(auto* link_model : model->getLinkModels())
            link_frames.push_back(Frame(link_model->getJointOriginTransform()));

        joint_axis_list.clear();
        joint_axis_list.resize(robot_model->getJointModelCount());
        for(size_t i = 0; i < joint_axis_list.size(); i++)
        {
            auto* joint_model = robot_model->getJointModel(i);
            if(auto* j = dynamic_cast<const moveit::core::RevoluteJointModel*>(joint_model))
                joint_axis_list[i] = Vector3(j->getAxis().x(), j->getAxis().y(), j->getAxis().z());
            if(auto* j = dynamic_cast<const moveit::core::PrismaticJointModel*>(joint_model))
                joint_axis_list[i] = Vector3(j->getAxis().x(), j->getAxis().y(), j->getAxis().z());
        }
    }
};



// fast tree fk
class RobotFK_Fast_Base : protected RobotJointEvaluator
{
protected:
    std::vector<std::string> tip_names;
    std::vector<Frame> tip_frames;
    std::vector<const moveit::core::LinkModel*> tip_links;
    std::vector<const moveit::core::LinkModel*> link_schedule;
    std::vector<Frame> global_frames;
    std::vector<std::vector<const moveit::core::LinkModel*>> link_chains;
    std::vector<size_t> active_variables;
    std::vector<std::pair<size_t, size_t>> variable_to_link_map;
    std::vector<size_t> variable_to_link_map_2;
    std::vector<std::pair<size_t, size_t>> variable_to_link_chain_map;
    inline void updateMimic(std::vector<double>& values)
    {
        for(auto* joint : robot_model->getMimicJointModels())
        {
            auto src = joint->getMimic()->getFirstVariableIndex();
            auto dest = joint->getFirstVariableIndex();

            // TODO: more dimensions ???
            values[dest] = values[src] * joint->getMimicFactor() + joint->getMimicOffset();

            // moveit doesn't seem to support multiple mimic dimensions either... ???

            // TODO: ok???
            //for(size_t base = 0; base < joint->getVariableCount(); base++)
            //    values[base + dest] = values[base + src] * joint->getMimicFactor() + joint->getMimicOffset();
        }
    }
public:
    RobotFK_Fast_Base(MoveItRobotModelConstPtr model) : RobotJointEvaluator(model)
    {
    }
    void initialize(const std::vector<size_t>& tip_link_indices)
    {

        tip_names.resize(tip_link_indices.size());
        for(size_t i = 0; i < tip_link_indices.size(); i++)
            tip_names[i] = robot_model->getLinkModelNames()[tip_link_indices[i]];

        tip_frames.resize(tip_names.size());

        tip_links.clear();
        for(const auto& n : tip_names)
            tip_links.push_back(robot_model->getLinkModel(n));

        global_frames.resize(robot_model->getLinkModelCount());

        link_chains.clear();
        link_schedule.clear();
        for(auto* tip_link : tip_links)
        {
            std::vector<const moveit::core::LinkModel*> chain;
            for(auto* link = tip_link; link; link = link->getParentLinkModel())
                chain.push_back(link);
            reverse(chain.begin(), chain.end());
            link_chains.push_back(chain);
            for(auto* link : chain)
            {
                if(find(link_schedule.begin(), link_schedule.end(), link) != link_schedule.end()) continue;
                link_schedule.push_back(link);
            }
        }

        active_variables.clear();
        for(auto* link_model : link_schedule)
        {
            auto link_index = link_model->getLinkIndex();
            auto* joint_model = link_model->getParentJointModel();
            size_t vstart = joint_model->getFirstVariableIndex();
            size_t vcount = joint_model->getVariableCount();
            for(size_t vi = vstart; vi < vstart + vcount; vi++)
            {
                variable_to_link_map.push_back(std::make_pair(vi, link_index));

                if(find(active_variables.begin(), active_variables.end(), vi) != active_variables.end()) continue;
                    active_variables.push_back(vi);
            }
        }

        variable_to_link_map_2.resize(robot_model->getVariableCount());
        for(auto* link_model : link_schedule)
        {
            auto link_index = link_model->getLinkIndex();
            auto* joint_model = link_model->getParentJointModel();
            size_t vstart = joint_model->getFirstVariableIndex();
            size_t vcount = joint_model->getVariableCount();
            for(size_t vi = vstart; vi < vstart + vcount; vi++)
            {
                variable_to_link_map_2[vi] = link_index;
            }
        }

        variable_to_link_chain_map.resize(robot_model->getVariableCount());
        for(size_t chain_index = 0; chain_index < link_chains.size(); chain_index++)
        {
            auto& link_chain = link_chains[chain_index];
            for(size_t ipos = 0; ipos < link_chain.size(); ipos++)
            {
                auto* link_model = link_chain[ipos];
                auto* joint_model = link_model->getParentJointModel();
                size_t vstart = joint_model->getFirstVariableIndex();
                size_t vcount = joint_model->getVariableCount();
                for(size_t vi = vstart; vi < vstart + vcount; vi++)
                {
                    variable_to_link_chain_map[vi] = std::make_pair(chain_index, ipos);
                }
            }
        }
    }
    void applyConfiguration(const std::vector<double>& jj0)
    {
        FNPROFILER();
        variables = jj0;
        updateMimic(variables);
        for(auto* link_model : link_schedule)
        {
            auto* joint_model = link_model->getParentJointModel();
            auto* parent_link_model = joint_model->getParentLinkModel();
            if(parent_link_model)
            {
                concat(
                    global_frames[parent_link_model->getLinkIndex()],
                    getLinkFrame(link_model),
                    getJointFrame(joint_model),
                    global_frames[link_model->getLinkIndex()]);
            }
            else
            {
                concat(
                    getLinkFrame(link_model),
                    getJointFrame(joint_model),
                    global_frames[link_model->getLinkIndex()]);
            }
            //if(linkModel->getParentLinkModel() && linkModel->getParentLinkModel()->getLinkIndex() > linkModel->getLinkIndex()) { LOG("wrong link order"); throw runtime_error("wrong link order"); }
        }
        for(size_t itip = 0; itip < tip_links.size(); itip++)
        {
            tip_frames[itip] = global_frames[tip_links[itip]->getLinkIndex()];
        }
    }
    inline const Frame& getTipFrame(size_t fi) const
    {
        return tip_frames[fi];
    }
    inline const std::vector<Frame>& getTipFrames() const
    {
        return tip_frames;
    }
    inline void incrementalBegin(const std::vector<double>& jj)
    {
        applyConfiguration(jj);
    }
    inline void incrementalEnd()
    {
    }
    inline const Frame& getJointVariableFrame(size_t variable_index)
    {
        return global_frames[variable_to_link_map_2[variable_index]];
    }
};



// incremental tip frame updates without full recomputation
class RobotFK_Fast : public RobotFK_Fast_Base
{
protected:
    std::vector<std::vector<Frame>> chain_cache;
    std::vector<double> vars, variables0;
    std::vector<Frame> test_tips;
    std::vector<uint8_t> links_changed;
    bool use_incremental;
    bool last_use_incremental;
    void updateIncremental(const std::vector<double>& vars0)
    {
        FNPROFILER();

        //PROFILER_COUNTER("incremental update count");

        // init variable lists
        vars = vars0;
        updateMimic(vars);
        for(auto& vi : active_variables)
        {
            //vars[vi] = vars0[vi];
            variables0[vi] = variables[vi];
        }

        // find moved links
        for(auto* link_model : link_schedule)
            links_changed[link_model->getLinkIndex()] = false;
        for(auto& x : variable_to_link_map)
        {
            auto variable_index = x.first;
            auto link_index = x.second;
            if(vars[variable_index] != variables[variable_index]) links_changed[link_index] = true;
        }

        // update
        for(size_t ichain = 0; ichain < link_chains.size(); ichain++)
        {
            auto& link_chain = link_chains[ichain];
            auto& cache_chain = chain_cache[ichain];

            size_t iposmax = 0;
            for(size_t ipos = 0; ipos < link_chain.size(); ipos++)
            {
                auto* link_model = link_chain[ipos];
                bool changed = links_changed[link_model->getLinkIndex()];
                if(changed) iposmax = ipos;
            }

            for(size_t ipos = 0; ipos <= iposmax; ipos++)
            {
                auto* link_model = link_chain[ipos];
                auto* joint_model = link_model->getParentJointModel();

                bool changed = links_changed[link_model->getLinkIndex()];

                if(cache_chain.size() <= ipos || changed)
                {
                    Frame before, after;

                    if(ipos < cache_chain.size())
                    {
                        before = cache_chain[ipos];
                    }
                    else
                    {
                        if(ipos > 0)
                        {
                            concat(cache_chain[ipos - 1], getLinkFrame(link_model), getJointFrame(joint_model), before);
                        }
                        else
                        {
                            concat(getLinkFrame(link_model), getJointFrame(joint_model), before);
                        }
                    }

                    chain_cache[ichain].resize(ipos + 1, Frame::identity());

                    if(changed)
                    {
                        if(ichain > 0 && ipos < link_chains[ichain - 1].size() && link_chains[ichain][ipos] == link_chains[ichain - 1][ipos])
                        {
                            after = chain_cache[ichain - 1][ipos];
                        }
                        else
                        {
                            size_t vstart = joint_model->getFirstVariableIndex();
                            size_t vcount = joint_model->getVariableCount();

                            if(vcount == 1)
                                variables[vstart] = vars[vstart];
                            else
                                for(size_t vi = vstart; vi < vstart + vcount; vi++)
                                    variables[vi] = vars[vi];

                            if(ipos > 0)
                            {
                                concat(cache_chain[ipos - 1], getLinkFrame(link_model), getJointFrame(joint_model), after);
                            }
                            else
                            {
                                concat(getLinkFrame(link_model), getJointFrame(joint_model), after);
                            }

                            if(vcount == 1)
                                variables[vstart] = variables0[vstart];
                            else
                                for(size_t vi = vstart; vi < vstart + vcount; vi++)
                                    variables[vi] = variables0[vi];

                            //PROFILER_COUNTER("incremental update transforms");
                        }
                        chain_cache[ichain][ipos] = after;

                        //tipFrames[ichain] = inverse(before) * tipFrames[ichain];
                        //tipFrames[ichain] = after * tipFrames[ichain];

                        /*Frame before_inverse;
                        invert(before, before_inverse);

                        //tipFrames[ichain] = after * before_inverse * tipFrames[ichain];
                        {
                            concat(after, before_inverse, tip_frames[ichain], tip_frames[ichain]);
                        }*/

                        change(after, before, tip_frames[ichain], tip_frames[ichain]);
                    }
                    else
                    {
                        after = before;
                        chain_cache[ichain][ipos] = after;
                    }
                }
            }
        }

        // set new vars
        //variables = vars;
        for(auto& vi : active_variables) variables[vi] = vars[vi];

        // test
        if(0)
        {
            test_tips = tip_frames;
            RobotFK_Fast_Base::applyConfiguration(vars);
            for(size_t i = 0; i < tip_frames.size(); i++)
            {
                auto dist = tip_frames[i].pos.distance(test_tips[i].pos);
                LOG_VAR(dist);
                if(dist > 0.001)
                {
                    LOG(tip_frames[i].pos.x(), tip_frames[i].pos.y(), tip_frames[i].pos.z());
                    LOG(test_tips[i].pos.x(), test_tips[i].pos.y(), test_tips[i].pos.z());
                    ERROR("incremental update error");
                }
            }
        }
    }
public:
    RobotFK_Fast(MoveItRobotModelConstPtr model)
        : RobotFK_Fast_Base(model), use_incremental(false)
    {
    }
    inline void incrementalBegin(const std::vector<double>& jj)
    {
        applyConfiguration(jj);
        chain_cache.clear();
        chain_cache.resize(tip_frames.size());
        links_changed.resize(robot_model->getLinkModelCount());
        vars.resize(jj.size());
        variables0.resize(jj.size());
        use_incremental = true;
    }
    inline void incrementalEnd()
    {
        use_incremental = false;
        //applyConfiguration(variables);
    }
    inline void applyConfiguration(const std::vector<double>& jj)
    {
        if(use_incremental)
            updateIncremental(jj);
        else
            RobotFK_Fast_Base::applyConfiguration(jj);
    }
};








// fast tree fk jacobian
class RobotFK_Jacobian : public RobotFK_Fast
{
    typedef RobotFK_Fast Base;
protected:
    std::vector<std::vector<const moveit::core::JointModel*>> joint_dependencies;
    std::vector<int> tip_dependencies;
public:
    RobotFK_Jacobian(MoveItRobotModelConstPtr model) : RobotFK_Fast(model)
    {
    }
    void initialize(const std::vector<size_t>& tip_link_indices)
    {
        Base::initialize(tip_link_indices);
        auto tip_count = tip_names.size();
        joint_dependencies.resize(robot_model->getJointModelCount());
        for(auto& x : joint_dependencies) x.clear();
        for(auto* link_model : link_schedule)
        {
            auto* joint_model = link_model->getParentJointModel();
            joint_dependencies[joint_model->getJointIndex()].push_back(joint_model);
        }
        for(auto* link_model : link_schedule)
        {
            auto* joint_model = link_model->getParentJointModel();
            if(auto* mimic = joint_model->getMimic())
            {
                while(mimic->getMimic() && mimic->getMimic() != joint_model) mimic = mimic->getMimic();
                joint_dependencies[mimic->getJointIndex()].push_back(joint_model);
            }
        }
        tip_dependencies.resize(robot_model->getJointModelCount() * tip_count);
        for(auto& x : tip_dependencies) x = 0;
        for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
        {
            for(auto* link_model = tip_links[tip_index]; link_model; link_model = link_model->getParentLinkModel())
            {
                auto* joint_model = link_model->getParentJointModel();
                tip_dependencies[joint_model->getJointIndex() * tip_count + tip_index] = 1;
            }
        }
    }
    void computeJacobian(const std::vector<size_t>& variable_indices, Eigen::MatrixXd& jacobian)
    {
        double step_size = 0.00001;
        double half_step_size = step_size * 0.5;
        double inv_step_size = 1.0 / step_size;
        auto tip_count = tip_frames.size();
        jacobian.resize(tip_count * 6, variable_indices.size());
        for(size_t icol = 0; icol < variable_indices.size(); icol++)
        {
            for(size_t itip = 0; itip < tip_count; itip++)
            {
                jacobian(itip * 6 + 0, icol) = 0;
                jacobian(itip * 6 + 1, icol) = 0;
                jacobian(itip * 6 + 2, icol) = 0;
                jacobian(itip * 6 + 3, icol) = 0;
                jacobian(itip * 6 + 4, icol) = 0;
                jacobian(itip * 6 + 5, icol) = 0;
            }
        }
        for(size_t icol = 0; icol < variable_indices.size(); icol++)
        {
            auto ivar = variable_indices[icol];
            auto* var_joint_model = robot_model->getJointOfVariable(ivar);
            if(var_joint_model->getMimic()) continue;
            for(auto* joint_model : joint_dependencies[var_joint_model->getJointIndex()])
            {
                double scale = 1;
                for(auto* m = joint_model; m->getMimic() && m->getMimic() != joint_model; m = m->getMimic())
                {
                    scale *= m->getMimicFactor();
                }
                auto* link_model = joint_model->getChildLinkModel();
                switch(joint_model->getType())
                {
                case moveit::core::JointModel::FIXED:
                {
                    // fixed joint: zero gradient (do nothing)
                    continue;
                }
                case moveit::core::JointModel::REVOLUTE:
                {
                    auto& link_frame = global_frames[link_model->getLinkIndex()];
                    for(size_t itip = 0; itip < tip_count; itip++)
                    {
                        if(!tip_dependencies[joint_model->getJointIndex() * tip_count + itip]) continue;

                        auto& tip_frame = tip_frames[itip];

                        auto q = link_frame.rot.inverse() * tip_frame.rot;
                        q = q.inverse();

                        auto rot = joint_axis_list[joint_model->getJointIndex()];
                        quat_mul_vec(q, rot, rot);

                        auto vel = link_frame.pos - tip_frame.pos;
                        quat_mul_vec(tip_frame.rot.inverse(), vel, vel);

                        vel = vel.cross(rot);

                        jacobian(itip * 6 + 0, icol) += vel.x() * scale;
                        jacobian(itip * 6 + 1, icol) += vel.y() * scale;
                        jacobian(itip * 6 + 2, icol) += vel.z() * scale;

                        jacobian(itip * 6 + 3, icol) += rot.x() * scale;
                        jacobian(itip * 6 + 4, icol) += rot.y() * scale;
                        jacobian(itip * 6 + 5, icol) += rot.z() * scale;

                        // LOG("a", vel.x(), vel.y(), vel.z(), rot.x(), rot.y(), rot.z());
                    }
                    continue;
                }
                case moveit::core::JointModel::PRISMATIC:
                {
                    auto& link_frame = global_frames[link_model->getLinkIndex()];
                    for(size_t itip = 0; itip < tip_count; itip++)
                    {
                        if(!tip_dependencies[joint_model->getJointIndex() * tip_count + itip]) continue;

                        auto& tip_frame = tip_frames[itip];

                        auto q = link_frame.rot.inverse() * tip_frame.rot;
                        q = q.inverse();

                        auto axis = joint_axis_list[joint_model->getJointIndex()];
                        auto v = axis;
                        quat_mul_vec(q, axis, v);

                        jacobian(itip * 6 + 0, icol) += v.x() * scale;
                        jacobian(itip * 6 + 1, icol) += v.y() * scale;
                        jacobian(itip * 6 + 2, icol) += v.z() * scale;

                        // LOG("a", v.x(), v.y(), v.z(), 0, 0, 0);
                    }
                    continue;
                }
                default:
                {
                    // numeric differentiation for joint types that are not yet implemented / or joint types that might be added to moveit in the future
                    auto ivar2 = ivar;
                    if(joint_model->getMimic())
                        ivar2 = ivar2 - var_joint_model->getFirstVariableIndex() + joint_model->getFirstVariableIndex();
                    auto& link_frame_1 = global_frames[link_model->getLinkIndex()];
                    auto v0 = variables[ivar2];
                    //auto joint_frame_1 = getJointFrame(joint_model);
                    variables[ivar2] = v0 + step_size;
                    auto joint_frame_2 = getJointFrame(joint_model);
                    variables[ivar2] = v0;
                    Frame link_frame_2;
                    if(auto* parent_link_model = joint_model->getParentLinkModel())
                        concat(global_frames[parent_link_model->getLinkIndex()], getLinkFrame(link_model), joint_frame_2, link_frame_2);
                    else
                        concat(getLinkFrame(link_model), joint_frame_2, link_frame_2);
                    for(size_t itip = 0; itip < tip_count; itip++)
                    {
                        if(!tip_dependencies[joint_model->getJointIndex() * tip_count + itip]) continue;
                        auto tip_frame_1 = tip_frames[itip];
                        Frame tip_frame_2;
                        change(link_frame_2, link_frame_1, tip_frame_1, tip_frame_2);
                        auto twist = frameTwist(tip_frame_1, tip_frame_2);
                        jacobian(itip * 6 + 0, icol) += twist.vel.x() * inv_step_size * scale;
                        jacobian(itip * 6 + 1, icol) += twist.vel.y() * inv_step_size * scale;
                        jacobian(itip * 6 + 2, icol) += twist.vel.z() * inv_step_size * scale;
                        jacobian(itip * 6 + 3, icol) += twist.rot.x() * inv_step_size * scale;
                        jacobian(itip * 6 + 4, icol) += twist.rot.y() * inv_step_size * scale;
                        jacobian(itip * 6 + 5, icol) += twist.rot.z() * inv_step_size * scale;
                    }
                    continue;
                }
                }
            }
        }
    }
};














// fast linear fk approximation

/*
class RobotFK_Mutator : public RobotFK_Jacobian
{
    typedef RobotFK_Jacobian Base;
    Eigen::MatrixXd mutation_approx_jacobian;
    aligned_vector<mutation_approx_frames<Frame>> mutation_approx_frames;
public:
    RobotFK_Mutator(MoveItRobotModelConstPtr model) : RobotFK_Jacobian(model)
    {
    }


    void initializeMutationApproximator(const std::vector<size_t>& variable_indices)
    {
        FNPROFILER();

        auto tip_count = tip_names.size();

        mutation_approx_frames.resize(tip_count);
        for(size_t itip = 0; itip < tip_count; itip++)
            mutation_approx_frames[itip].resize(robot_model->getVariableCount());

        computeJacobian(variable_indices, mutation_approx_jacobian);

        for(size_t icol = 0; icol < variable_indices.size(); icol++)
        {
            size_t ivar = variable_indices[icol];
            for(size_t itip = 0; itip < tip_count; itip++)
            {
                {
                    Vector3 t;
                    t.setX(mutation_approx_jacobian(itip * 6 + 0, icol));
                    t.setY(mutation_approx_jacobian(itip * 6 + 1, icol));
                    t.setZ(mutation_approx_jacobian(itip * 6 + 2, icol));
                    quat_mul_vec(tip_frames[itip].rot, t, t);
                    mutation_approx_frames[itip][ivar].pos = t;
                }

                {
                    Quaternion q;
                    q.setX(mutation_approx_jacobian(itip * 6 + 3, icol) * 0.5);
                    q.setY(mutation_approx_jacobian(itip * 6 + 4, icol) * 0.5);
                    q.setZ(mutation_approx_jacobian(itip * 6 + 5, icol) * 0.5);
                    q.setW(1.0);
                    quat_mul_quat(tip_frames[itip].rot, q, q);
                    q -= tip_frames[itip].rot;
                    mutation_approx_frames[itip][ivar].rot = q;
                }
            }
        }
    }


    void computeApproximateMutation1(
        size_t variable_index,
        double variable_delta,
        const std::vector<Frame>& input,
        std::vector<Frame>& output) const
    {
        //BLOCKPROFILER("computeApproximateMutations C");
        auto tip_count = tip_names.size();
        output.resize(tip_count);
        for(size_t itip = 0; itip < tip_count; itip++)
        {
            auto& joint_delta = mutation_approx_frames[itip][variable_index];

            const Frame& tip_frame = input[itip];

            double px = tip_frame.pos.x();
            double py = tip_frame.pos.y();
            double pz = tip_frame.pos.z();

            double rx = tip_frame.rot.x();
            double ry = tip_frame.rot.y();
            double rz = tip_frame.rot.z();
            double rw = tip_frame.rot.w();

            px += joint_delta.pos.x() * variable_delta;
            py += joint_delta.pos.y() * variable_delta;
            pz += joint_delta.pos.z() * variable_delta;

            rx += joint_delta.rot.x() * variable_delta;
            ry += joint_delta.rot.y() * variable_delta;
            rz += joint_delta.rot.z() * variable_delta;
            rw += joint_delta.rot.w() * variable_delta;

            auto& tip_mutation = output[itip];

            tip_mutation.pos.setX(px);
            tip_mutation.pos.setY(py);
            tip_mutation.pos.setZ(pz);

            tip_mutation.rot.setX(rx);
            tip_mutation.rot.setY(ry);
            tip_mutation.rot.setZ(rz);
            tip_mutation.rot.setW(rw);
        }
    }

  // GCC >= 4.8: function multiversioning
  // GCC >= 4.9: AVX and FMA intrinsics
#if (__GNUC__ > 4 || (__GNUC__ == 4 && __GNUC_MINOR__ >= 9))
    #define FUNCTION_MULTIVERSIONING 1
#endif


#ifndef __SSE2_MATH__
    __attribute__ ((hot))
    __attribute__ ((noinline))
#ifdef FUNCTION_MULTIVERSIONING
    __attribute__ ((target ("default")))
#endif
    void computeApproximateMutations(
        size_t variable_count,
        const size_t* variable_indices,
        size_t mutation_count,
        const double * const * mutation_values,
        std::vector<std::vector<Frame>>& tip_frame_mutations) const
    {
        BLOCKPROFILER("computeApproximateMutations C");

        //FNPROFILER();
        const double* p_variables = variables.data();
        auto tip_count = tip_names.size();
        //while(tip_frame_mutations.size() < mutation_count) tip_frame_mutations.emplace_back(tip_count);
        tip_frame_mutations.resize(mutation_count);
        for(auto& m : tip_frame_mutations) m.resize(tip_count);
        for(size_t itip = 0; itip < tip_count; itip++)
        {
            auto& joint_deltas = mutation_approx_frames[itip];

            const Frame& tip_frame = tip_frames[itip];
            for(size_t imutation = 0; imutation < mutation_count; imutation++)
            {
                double px = tip_frame.pos.x();
                double py = tip_frame.pos.y();
                double pz = tip_frame.pos.z();

                double rx = tip_frame.rot.x();
                double ry = tip_frame.rot.y();
                double rz = tip_frame.rot.z();
                double rw = tip_frame.rot.w();

                for(size_t vii = 0; vii < variable_count; vii++)
                {
                    size_t variable_index = variable_indices[vii];
                    double variable_delta = mutation_values[imutation][vii] - p_variables[variable_index];

                    px += joint_deltas[variable_index].pos.x() * variable_delta;
                    py += joint_deltas[variable_index].pos.y() * variable_delta;
                    pz += joint_deltas[variable_index].pos.z() * variable_delta;

                    rx += joint_deltas[variable_index].rot.x() * variable_delta;
                    ry += joint_deltas[variable_index].rot.y() * variable_delta;
                    rz += joint_deltas[variable_index].rot.z() * variable_delta;
                    rw += joint_deltas[variable_index].rot.w() * variable_delta;
                }

                auto& tip_mutation = tip_frame_mutations[imutation][itip];

                tip_mutation.pos.setX(px);
                tip_mutation.pos.setY(py);
                tip_mutation.pos.setZ(pz);

                tip_mutation.rot.setX(rx);
                tip_mutation.rot.setY(ry);
                tip_mutation.rot.setZ(rz);
                tip_mutation.rot.setW(rw);
            }
        }
    }
#endif



#ifdef __SSE2_MATH__
    __attribute__ ((hot))
    __attribute__ ((noinline))
#ifdef FUNCTION_MULTIVERSIONING
    __attribute__ ((target ("default")))
#endif
    void computeApproximateMutations(
        size_t variable_count,
        const size_t* __restrict__ variable_indices,
        size_t mutation_count,
        const double * const * mutation_values,
        std::vector<std::vector<Frame>>& tip_frame_mutations) const
    {
        BLOCKPROFILER("computeApproximateMutations SSE2");

        auto* __restrict__ variables_ptr = variables.data();
        auto tip_count = tip_names.size();
        //while(tip_frame_mutations.size() < mutation_count) tip_frame_mutations.emplace_back(tip_count);
        tip_frame_mutations.resize(mutation_count);
        for(auto& m : tip_frame_mutations) m.resize(tip_count);
        for(size_t itip = 0; itip < tip_count; itip++)
        {
            auto* __restrict__ joint_deltas = mutation_approx_frames[itip].data();
            const Frame& tip_frame = tip_frames[itip];
            const double* tip_frame_ptr = (const double*)&tip_frame;
            __m128d pxy0 = _mm_load_pd(tip_frame_ptr + 0);
            __m128d pzw0 = _mm_load_pd(tip_frame_ptr + 2);
            __m128d rxy0 = _mm_load_pd(tip_frame_ptr + 4);
            __m128d rzw0 = _mm_load_pd(tip_frame_ptr + 6);
            for(size_t imutation = 0; imutation < mutation_count; imutation++)
            {
                auto* __restrict__ mutation_ptr = mutation_values[imutation];
                __m128d pxy = pxy0;
                __m128d pzw = pzw0;
                __m128d rxy = rxy0;
                __m128d rzw = rzw0;
                for(size_t vii = 0; vii < variable_count; vii++)
                {
                    auto variable_index = variable_indices[vii];
                    double variable_delta = mutation_ptr[vii] - variables_ptr[variable_index];
                    __m128d ff = _mm_set1_pd(variable_delta);
                    auto joint_delta_ptr = (const double* __restrict__)&(joint_deltas[variable_index]);
                    pxy = _mm_add_pd(pxy, _mm_mul_pd(ff, _mm_load_pd(joint_delta_ptr + 0)));
                    pzw = _mm_add_pd(pzw, _mm_mul_pd(ff, _mm_load_pd(joint_delta_ptr + 2)));
                    rxy = _mm_add_pd(rxy, _mm_mul_pd(ff, _mm_load_pd(joint_delta_ptr + 4)));
                    rzw = _mm_add_pd(rzw, _mm_mul_pd(ff, _mm_load_pd(joint_delta_ptr + 6)));
                }
                auto& tip_mutation = tip_frame_mutations[imutation][itip];
                double* __restrict__ tip_mutation_ptr = (double*)&tip_mutation;
                _mm_store_pd(tip_mutation_ptr + 0, pxy);
                _mm_store_pd(tip_mutation_ptr + 2, pzw);
                _mm_store_pd(tip_mutation_ptr + 4, rxy);
                _mm_store_pd(tip_mutation_ptr + 6, rzw);
            }

        }
    }
#endif



#ifdef FUNCTION_MULTIVERSIONING
    __attribute__ ((hot))
    __attribute__ ((noinline))
    __attribute__ ((target ("avx")))
    void computeApproximateMutations(
        size_t variable_count,
        const size_t* __restrict__ variable_indices,
        size_t mutation_count,
        const double * const * mutation_values,
        std::vector<std::vector<Frame>>& tip_frame_mutations) const
    {
        BLOCKPROFILER("computeApproximateMutations AVX");

        auto* __restrict__ variables_ptr = variables.data();
        auto tip_count = tip_names.size();
        //while(tip_frame_mutations.size() < mutation_count) tip_frame_mutations.emplace_back(tip_count);
        tip_frame_mutations.resize(mutation_count);
        for(auto& m : tip_frame_mutations) m.resize(tip_count);
        for(size_t itip = 0; itip < tip_count; itip++)
        {
            auto* __restrict__ joint_deltas = mutation_approx_frames[itip].data();
            const Frame& tip_frame = tip_frames[itip];
            const double* tip_frame_ptr = (const double*)&tip_frame;
            __m256d p0 = _mm256_loadu_pd(tip_frame_ptr + 0);
            __m256d r0 = _mm256_loadu_pd(tip_frame_ptr + 4);
            for(size_t imutation = 0; imutation < mutation_count; imutation++)
            {
                auto* __restrict__ mutation_ptr = mutation_values[imutation];
                __m256d p = p0;
                __m256d r = r0;
                for(size_t vii = 0; vii < variable_count; vii++)
                {
                    auto variable_index = variable_indices[vii];
                    double variable_delta = mutation_ptr[vii] - variables_ptr[variable_index];
                    __m256d ff = _mm256_set1_pd(variable_delta);
                    auto joint_delta_ptr = (const double* __restrict__)&(joint_deltas[variable_index]);
                    p = _mm256_add_pd(p, _mm256_mul_pd(ff, _mm256_load_pd(joint_delta_ptr + 0)));
                    r = _mm256_add_pd(r, _mm256_mul_pd(ff, _mm256_load_pd(joint_delta_ptr + 4)));
                }
                auto& tip_mutation = tip_frame_mutations[imutation][itip];
                double* __restrict__ tip_mutation_ptr = (double*)&tip_mutation;
                _mm256_storeu_pd(tip_mutation_ptr + 0, p);
                _mm256_storeu_pd(tip_mutation_ptr + 4, r);
            }
        }
    }

    __attribute__ ((hot))
    __attribute__ ((noinline))
    __attribute__ ((target ("fma", "avx")))
    void computeApproximateMutations(
        size_t variable_count,
        const size_t* __restrict__ variable_indices,
        size_t mutation_count,
        const double * const * mutation_values,
        std::vector<std::vector<Frame>>& tip_frame_mutations) const
    {
        BLOCKPROFILER("computeApproximateMutations FMA AVX");

        //LOG("fma");
        auto* __restrict__ variables_ptr = variables.data();
        auto tip_count = tip_names.size();
        //while(tip_frame_mutations.size() < mutation_count) tip_frame_mutations.emplace_back(tip_count);
        tip_frame_mutations.resize(mutation_count);
        for(auto& m : tip_frame_mutations) m.resize(tip_count);
        for(size_t itip = 0; itip < tip_count; itip++)
        {
            auto* __restrict__ joint_deltas = mutation_approx_frames[itip].data();
            const Frame& tip_frame = tip_frames[itip];
            const double* tip_frame_ptr = (const double*)&tip_frame;
            __m256d p0 = _mm256_loadu_pd(tip_frame_ptr + 0);
            __m256d r0 = _mm256_loadu_pd(tip_frame_ptr + 4);
            for(size_t imutation = 0; imutation < mutation_count; imutation++)
            {
                auto* __restrict__ mutation_ptr = mutation_values[imutation];
                __m256d p = p0;
                __m256d r = r0;
                for(size_t vii = 0; vii < variable_count; vii++)
                {
                    auto variable_index = variable_indices[vii];
                    double variable_delta = mutation_ptr[vii] - variables_ptr[variable_index];
                    __m256d ff = _mm256_set1_pd(variable_delta);
                    auto joint_delta_ptr = (const double* __restrict__)&(joint_deltas[variable_index]);
                    p = _mm256_fmadd_pd(ff, _mm256_load_pd(joint_delta_ptr + 0), p);
                    r = _mm256_fmadd_pd(ff, _mm256_load_pd(joint_delta_ptr + 4), r);
                }
                auto& tip_mutation = tip_frame_mutations[imutation][itip];
                double* __restrict__ tip_mutation_ptr = (double*)&tip_mutation;
                _mm256_storeu_pd(tip_mutation_ptr + 0, p);
                _mm256_storeu_pd(tip_mutation_ptr + 4, r);
            }
            //LOG_VAR(itip);
        }
    }
#endif


};
*/











class RobotFK_Mutator : public RobotFK_Jacobian
{
    typedef RobotFK_Jacobian Base;
    Eigen::MatrixXd mutation_approx_jacobian;
    aligned_vector<aligned_vector<Frame>> mutation_approx_frames;
    aligned_vector<aligned_vector<Vector3>> mutation_approx_quadratics;
public:
    RobotFK_Mutator(MoveItRobotModelConstPtr model) : RobotFK_Jacobian(model)
    {
    }

    void initializeMutationApproximator(const std::vector<size_t>& variable_indices)
    {
        FNPROFILER();

        auto tip_count = tip_names.size();

        // init first order approximators
        {
            mutation_approx_frames.resize(tip_count);
            for(size_t itip = 0; itip < tip_count; itip++)
                mutation_approx_frames[itip].resize(robot_model->getVariableCount());

            for(size_t itip = 0; itip < tip_count; itip++)
                for(auto ivar : variable_indices)
                    mutation_approx_frames[itip][ivar] = Frame::identity();

            computeJacobian(variable_indices, mutation_approx_jacobian);

            for(size_t icol = 0; icol < variable_indices.size(); icol++)
            {
                size_t ivar = variable_indices[icol];
                for(size_t itip = 0; itip < tip_count; itip++)
                {
                    {
                        Vector3 t;
                        t.setX(mutation_approx_jacobian(itip * 6 + 0, icol));
                        t.setY(mutation_approx_jacobian(itip * 6 + 1, icol));
                        t.setZ(mutation_approx_jacobian(itip * 6 + 2, icol));
                        quat_mul_vec(tip_frames[itip].rot, t, t);
                        mutation_approx_frames[itip][ivar].pos = t;
                    }

                    {
                        Quaternion q;
                        q.setX(mutation_approx_jacobian(itip * 6 + 3, icol) * 0.5);
                        q.setY(mutation_approx_jacobian(itip * 6 + 4, icol) * 0.5);
                        q.setZ(mutation_approx_jacobian(itip * 6 + 5, icol) * 0.5);
                        q.setW(1.0);
                        quat_mul_quat(tip_frames[itip].rot, q, q);
                        q -= tip_frames[itip].rot;
                        mutation_approx_frames[itip][ivar].rot = q;
                    }
                }
            }
        }

        // init second order approximators
        {
            mutation_approx_quadratics.resize(tip_count);
            for(size_t itip = 0; itip < tip_count; itip++)
                mutation_approx_quadratics[itip].resize(robot_model->getVariableCount());
            for(size_t itip = 0; itip < tip_count; itip++)
                for(auto ivar : variable_indices)
                    mutation_approx_quadratics[itip][ivar] = Vector3(0, 0, 0);
            for(auto ivar : variable_indices)
            {
                auto* var_joint_model = robot_model->getJointOfVariable(ivar);
                if(var_joint_model->getMimic()) continue;
                for(auto* joint_model : joint_dependencies[var_joint_model->getJointIndex()])
                {
                    double scale = 1;
                    for(auto* m = joint_model; m->getMimic() && m->getMimic() != joint_model; m = m->getMimic())
                    {
                        scale *= m->getMimicFactor();
                    }
                    auto* link_model = joint_model->getChildLinkModel();
                    switch(joint_model->getType())
                    {
                        case moveit::core::JointModel::REVOLUTE:
                        {
                            auto& link_frame = global_frames[link_model->getLinkIndex()];

                            for(size_t itip = 0; itip < tip_count; itip++)
                            {
                                if(!tip_dependencies[joint_model->getJointIndex() * tip_count + itip]) continue;

                                auto& tip_frame = tip_frames[itip];

                                auto axis = joint_axis_list[joint_model->getJointIndex()];
                                quat_mul_vec(link_frame.rot, axis, axis);

                                auto v = link_frame.pos - tip_frame.pos;
                                v -= axis * v.dot(axis);

                                mutation_approx_quadratics[itip][ivar] = v * 0.5 * (scale * scale);
                            }

                            continue;
                        }
                    }
                }
            }
        }
    }


    void computeApproximateMutation1(
        size_t variable_index,
        double variable_delta,
        const std::vector<Frame>& input,
        std::vector<Frame>& output) const
    {
        //BLOCKPROFILER("computeApproximateMutations C");
        auto tip_count = tip_names.size();
        output.resize(tip_count);
        for(size_t itip = 0; itip < tip_count; itip++)
        {
            auto& joint_delta = mutation_approx_frames[itip][variable_index];

            const Frame& tip_frame = input[itip];

            double px = tip_frame.pos.x();
            double py = tip_frame.pos.y();
            double pz = tip_frame.pos.z();

            double rx = tip_frame.rot.x();
            double ry = tip_frame.rot.y();
            double rz = tip_frame.rot.z();
            double rw = tip_frame.rot.w();

            px += joint_delta.pos.x() * variable_delta;
            py += joint_delta.pos.y() * variable_delta;
            pz += joint_delta.pos.z() * variable_delta;

            double variable_delta_sq = variable_delta * variable_delta;
            px += mutation_approx_quadratics[itip][variable_index].x() * variable_delta_sq;
            py += mutation_approx_quadratics[itip][variable_index].y() * variable_delta_sq;
            pz += mutation_approx_quadratics[itip][variable_index].z() * variable_delta_sq;

            rx += joint_delta.rot.x() * variable_delta;
            ry += joint_delta.rot.y() * variable_delta;
            rz += joint_delta.rot.z() * variable_delta;
            rw += joint_delta.rot.w() * variable_delta;

            auto& tip_mutation = output[itip];

            tip_mutation.pos.setX(px);
            tip_mutation.pos.setY(py);
            tip_mutation.pos.setZ(pz);

            tip_mutation.rot.setX(rx);
            tip_mutation.rot.setY(ry);
            tip_mutation.rot.setZ(rz);
            tip_mutation.rot.setW(rw);
        }
    }

    void computeApproximateMutations(
        size_t variable_count,
        const size_t* variable_indices,
        size_t mutation_count,
        const double * const * mutation_values,
        std::vector<std::vector<Frame>>& tip_frame_mutations) const
    {
        BLOCKPROFILER("computeApproximateMutations C");

        //FNPROFILER();
        const double* p_variables = variables.data();
        auto tip_count = tip_names.size();
        //while(tip_frame_mutations.size() < mutation_count) tip_frame_mutations.emplace_back(tip_count);
        tip_frame_mutations.resize(mutation_count);
        for(auto& m : tip_frame_mutations) m.resize(tip_count);
        for(size_t itip = 0; itip < tip_count; itip++)
        {
            auto& joint_deltas = mutation_approx_frames[itip];

            const Frame& tip_frame = tip_frames[itip];
            for(size_t imutation = 0; imutation < mutation_count; imutation++)
            {
                double px = tip_frame.pos.x();
                double py = tip_frame.pos.y();
                double pz = tip_frame.pos.z();

                double rx = tip_frame.rot.x();
                double ry = tip_frame.rot.y();
                double rz = tip_frame.rot.z();
                double rw = tip_frame.rot.w();

                for(size_t vii = 0; vii < variable_count; vii++)
                {
                    size_t variable_index = variable_indices[vii];
                    double variable_delta = mutation_values[imutation][vii] - p_variables[variable_index];

                    px += joint_deltas[variable_index].pos.x() * variable_delta;
                    py += joint_deltas[variable_index].pos.y() * variable_delta;
                    pz += joint_deltas[variable_index].pos.z() * variable_delta;

                    double variable_delta_sq = variable_delta * variable_delta;
                    px += mutation_approx_quadratics[itip][variable_index].x() * variable_delta_sq;
                    py += mutation_approx_quadratics[itip][variable_index].y() * variable_delta_sq;
                    pz += mutation_approx_quadratics[itip][variable_index].z() * variable_delta_sq;

                    rx += joint_deltas[variable_index].rot.x() * variable_delta;
                    ry += joint_deltas[variable_index].rot.y() * variable_delta;
                    rz += joint_deltas[variable_index].rot.z() * variable_delta;
                    rw += joint_deltas[variable_index].rot.w() * variable_delta;
                }

                auto& tip_mutation = tip_frame_mutations[imutation][itip];

                tip_mutation.pos.setX(px);
                tip_mutation.pos.setY(py);
                tip_mutation.pos.setZ(pz);

                tip_mutation.rot.setX(rx);
                tip_mutation.rot.setY(ry);
                tip_mutation.rot.setZ(rz);
                tip_mutation.rot.setW(rw);
            }
        }
    }


};




























typedef RobotFK_Mutator RobotFK;


















// for comparison
class RobotFK_MoveIt
{
    MoveItRobotModelConstPtr robot_model;
    moveit::core::RobotState robot_state;
    std::vector<Frame> tipFrames;
    std::vector<const moveit::core::LinkModel*> tipLinks;
    std::vector<double> jj;
public:
    RobotFK_MoveIt(MoveItRobotModelConstPtr model)
        : robot_model(model), robot_state(model)
    {
    }
    void initialize(const std::vector<size_t>& tip_link_indices)
    {
        tipFrames.resize(tip_link_indices.size());
        tipLinks.resize(tip_link_indices.size());
        for(size_t i = 0; i < tip_link_indices.size(); i++)
            tipLinks[i] = robot_model->getLinkModel(tip_link_indices[i]);
    }
    void applyConfiguration(const std::vector<double>& jj0)
    {
        BLOCKPROFILER("RobotFK_MoveIt applyConfiguration");
        jj = jj0;
        robot_model->interpolate(jj0.data(), jj0.data(), 0.5, jj.data()); // force mimic update
        robot_state.setVariablePositions(jj);
        robot_state.update();
        for(size_t i = 0; i < tipFrames.size(); i++)
            tipFrames[i] = Frame(robot_state.getGlobalLinkTransform(tipLinks[i]));
    }
    inline void incrementalBegin(const std::vector<double>& jj)
    {
    }
    inline void incrementalEnd()
    {
    }
    const Frame& getTipFrame(size_t fi) const
    {
        return tipFrames[fi];
    }
    const std::vector<Frame>& getTipFrames() const
    {
        return tipFrames;
    }
};
















}
