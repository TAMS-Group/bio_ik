// Bio IK for ROS
// Philipp Ruppel

#include "frame.h"
#include "utils.h"
#include "robot_helpers.h"
#include "forward_kinematics.h"
#include "ik_base.h"
#include "ik_parallel.h"

#include <pluginlib/class_list_macros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <urdf/model.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf_model/model.h>
#include <srdfdom/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>

#include <random>
#include <type_traits>
#include <atomic>
#include <tuple>



using namespace bio_ik;

namespace bio_ik_kinematics_plugin
{     

typedef IKParallel PluginIKSolver;

struct BioIKKinematicsPlugin : kinematics::KinematicsBase
{
    uint8_t test;

    std::vector<std::string> joint_names, link_names;
    MoveItRobotModelConstPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    mutable std::unique_ptr<PluginIKSolver> ik;
    mutable std::vector<double> state, temp;
    mutable std::unique_ptr<moveit::core::RobotState> temp_state;
    mutable std::vector<Frame> tipFrames;
    RobotInfo robot_info;

    BioIKKinematicsPlugin()
    {
        test = 0;
    }

    virtual const std::vector<std::string>& getJointNames() const
    {
        LOG_FNC();
        return joint_names;
    }
    
    virtual const std::vector<std::string>& getLinkNames() const
    {
        LOG_FNC();
        return link_names;
    }
    
    virtual bool getPositionFK(
                        const std::vector<std::string>& link_names, 
                        const std::vector<double>& joint_angles, 
                        std::vector<geometry_msgs::Pose>& poses) const
    {
        LOG_FNC();
        return false;
    }
    
    virtual bool getPositionIK(
                        const geometry_msgs::Pose& ik_pose, 
                        const std::vector<double>& ik_seed_state, 
                        std::vector<double>& solution, 
                        moveit_msgs::MoveItErrorCodes& error_code, 
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }

    std::vector<Eigen::Affine3d> tip_reference_frames;

    IKParams ikparams;
    
    bool load(std::string robot_description, std::string group_name)
    {
        LOG_FNC();
        
        LOG_VAR(robot_description);
        LOG_VAR(group_name);

        rdf_loader::RDFLoader rdf_loader(robot_description_);
        auto srdf = rdf_loader.getSRDF();
        auto urdf_model = rdf_loader.getURDF();

        if (!urdf_model || !srdf)
        {
            LOG("URDF and SRDF must be loaded for KDL kinematics solver to work.");
            return false;
        }
        
        robot_model.reset(new robot_model::RobotModel(urdf_model, srdf));
        
        joint_model_group = robot_model->getJointModelGroup(group_name);
        if (!joint_model_group)
        {
            LOG("failed to get joint model group");
            return false;
        }

        joint_names.clear();
        
        for(auto* joint_model : joint_model_group->getJointModels())
            if(joint_model->getName() != base_frame_ && joint_model->getType() != moveit::core::JointModel::UNKNOWN && joint_model->getType() != moveit::core::JointModel::FIXED)
                joint_names.push_back(joint_model->getName());

        auto tips2 = tip_frames_;
        joint_model_group->getEndEffectorTips(tips2);
        if(!tips2.empty()) tip_frames_ = tips2;

        link_names = tip_frames_; 

        for(auto& n : joint_names) LOG("joint", n);
        for(auto& n : link_names) LOG("link", n);

        moveit::core::RobotState robot_state(robot_model);
        tip_reference_frames.clear();
        for(auto& tip : tip_frames_)
        {
            auto ref = robot_state.getGlobalLinkTransform(getBaseFrame());
            tip_reference_frames.push_back(ref);
        }
        
        ros::NodeHandle node_handle("~");
        std::string rdesc;
        node_handle.searchParam(robot_description_, rdesc);
        node_handle = ros::NodeHandle(rdesc + "_kinematics/" + group_name_);
        
        bool enable_profiler;
        node_handle.param("profiler", enable_profiler, false);
        if(enable_profiler) Profiler::start();

        std::vector<size_t> mutable_genes;
        RobotInfo model_info(robot_model, tip_frames_);

        for(size_t j : model_info.getActiveVariables())
        {
            for(auto& vname : joint_model_group->getVariableNames())
            {
                size_t i = robot_model->getVariableIndex(vname);
                if(i == j)
                {
                    mutable_genes.push_back(i);
                }
            }
        }
        
        robot_info = RobotInfo(robot_model, tip_frames_);
        
        ikparams.robot_model = robot_model;
        ikparams.node_handle = node_handle;
        ikparams.tip_frames = tip_frames_;
        ikparams.active_variables = mutable_genes;
        
        ikparams.tip_infos.clear();
        
        HeuristicErrorTree heuristic_error_tree(robot_model, tip_frames_);
        
        auto load_tip = [&] (const std::string& name)
        {
            LOG("load tip", name);
        
            ros::NodeHandle node_handle("~");
            std::string rdesc;
            node_handle.searchParam(robot_description_, rdesc);
            node_handle = ros::NodeHandle(rdesc + "_kinematics/" + name);
            
            size_t tip_index = ikparams.tip_infos.size();
    
            ikparams.tip_infos.emplace_back();
            
            //ikparams.tip_infos.back().position_only_ik = node_handle.param("position_only_ik", false);
            //ikparams.tip_infos.back().weight = node_handle.param("weight", 1.0);
            
            node_handle.param("position_only_ik", ikparams.tip_infos.back().position_only_ik, false);
            node_handle.param("weight", ikparams.tip_infos.back().weight, 1.0);
            
            //ikparams.tip_infos.back().rotation_scale = heuristic_error_tree.getChainLength(tip_index) * (0.5 / M_PI);
            ikparams.tip_infos.back().rotation_scale = 0.5;
            node_handle.param("rotation_scale", ikparams.tip_infos.back().rotation_scale, ikparams.tip_infos.back().rotation_scale);
            if(ikparams.tip_infos.back().position_only_ik) ikparams.tip_infos.back().rotation_scale = 0;
            ikparams.tip_infos.back().rotation_scale_sq = ikparams.tip_infos.back().rotation_scale * ikparams.tip_infos.back().rotation_scale;
            
            LOG_VAR(ikparams.tip_infos.back().position_only_ik);
        };
        
        // TODO: per-tip config ???
        
        for(auto& tip_name : tip_frames_)
            LOG("tip", tip_name);
        
        for(auto& tip_name : tip_frames_)
            load_tip(group_name);
        
        /*std::vector<const moveit::core::JointModelGroup*> groups;
        joint_model_group->getSubgroups(groups);
        groups.push_back(joint_model_group);
        for(auto& tip_name : tip_frames_)
        {
            LOG_VAR(tip_name);
            for(auto* group : groups)
            {
                auto n = group->getEndEffectorName();
                LOG_VAR(group->getEndEffectorName());
                LOG_VAR(group->getName());
                if(!n.empty() && n == tip_name)
                {
                    load_tip(group->getName());
                goto _finish;
                }
            }
            for(auto* group : robot_model->getJointModelGroups())
            {
                std::vector<std::string> tips;
                group->getEndEffectorTips(tips);
                for(auto& tip : tips)
                {
                    LOG_VAR(tip);
                    if(tip == tip_name)
                    {
                        load_tip(group->getName());
                goto _finish;
                    }
                }
            }
            ERROR("tip not found", tip_name);
        }
    _finish:
        0;*/
        
        /*for(auto& tip_name : tip_frames_)
        {
            LOG_VAR(tip_name);
            for(auto& end_effector : robot_model->getSRDF()->getEndEffectors())
            {
                LOG_VAR(end_effector.name_);
                LOG_VAR(end_effector.component_group_);
                LOG_VAR(end_effector.parent_group_);
                LOG_VAR(end_effector.parent_link_);
                if(tip_name == end_effector.parent_link_)
                {
                    load_tip(end_effector.component_group_);
                }
            }
        }*/
        
        LOG_VAR(ikparams.tip_infos.size());
        
        //ERROR("");

        temp_state.reset(new moveit::core::RobotState(robot_model));
        
        ik.reset(new PluginIKSolver(ikparams));

        LOG("init ready");

        return true;
    }

    virtual bool initialize(const std::string &robot_description, const std::string &group_name, const std::string &base_frame, const std::string &tip_frame, double search_discretization)
    {
        LOG_FNC();
        std::vector<std::string> tip_frames;
        tip_frames.push_back(tip_frame);
        initialize(robot_description, group_name, base_frame, tip_frames, search_discretization);
        return true;
    }
    
    virtual bool initialize(const std::string& robot_description,
                          const std::string& group_name,
                          const std::string& base_frame,
                          const std::vector<std::string>& tip_frames,
                          double search_discretization)
    {
        LOG_FNC();
        setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);
        load(robot_description, group_name);
        return true;
    }

    // TODO: implement ?
    virtual bool searchPositionIK(
        const geometry_msgs::Pose& ik_pose, 
        const std::vector<double>& ik_seed_state, 
        double timeout, std::vector<double>& solution, 
        moveit_msgs::MoveItErrorCodes& error_code, 
        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }
    
    virtual bool searchPositionIK(
        const geometry_msgs::Pose& ik_pose, 
        const std::vector<double>& ik_seed_state, 
        double timeout, 
        const std::vector<double> &consistency_limits, 
        std::vector<double>& solution, 
        moveit_msgs::MoveItErrorCodes &error_code, 
        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }
    
    virtual bool searchPositionIK(
        const geometry_msgs::Pose& ik_pose, 
        const std::vector<double>& ik_seed_state,
        double timeout, 
        std::vector<double> &solution, 
        const IKCallbackFn& solution_callback, 
        moveit_msgs::MoveItErrorCodes& error_code, 
        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }
    
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector< double > &ik_seed_state, double timeout, const std::vector< double > &consistency_limits, std::vector< double > &solution, const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code, const kinematics::KinematicsQueryOptions &options=kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }
    
    /*struct OptMod : kinematics::KinematicsQueryOptions
    {
        int test;
    };*/

    virtual bool searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                const std::vector<double>& ik_seed_state,
                                double timeout,
                                const std::vector<double>& consistency_limits,
                                std::vector<double>& solution,
                                const IKCallbackFn& solution_callback,
                                moveit_msgs::MoveItErrorCodes& error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions(),
                                const moveit::core::RobotState* context_state = NULL) const
    {
        double t0 = ros::Time::now().toSec();

        LOG_FNC();

        FNPROFILER();
        
        //LOG(typeid(options).name());
        //LOG(((OptMod*)&options)->test);

        // get variable default positions / context state
        state.resize(robot_model->getVariableCount());
        robot_model->getVariableDefaultPositions(state);
        if(context_state)
            for(size_t i = 0; i < robot_model->getVariableCount(); i++)
                state[i] = context_state->getVariablePositions()[i];
                
        // overwrite used variables with seed state
        solution = ik_seed_state;
        {
            int i = 0;
            for(auto& joint_name : getJointNames())
            {
                auto* joint_model = robot_model->getJointModel(joint_name);
                if(!joint_model) continue;
                for(size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
                    state.at(joint_model->getFirstVariableIndex() + vi) = solution.at(i++);
            }
        }
        
        // transform tips to baseframe
        tipFrames.clear();
        for(size_t i = 0; i < ik_poses.size(); i++)
        {
            Eigen::Affine3d p, r;
            tf::poseMsgToEigen(ik_poses[i], p);
            if(context_state)
                r = context_state->getGlobalLinkTransform(getBaseFrame());
            else
                r = tip_reference_frames[i];
            tipFrames.emplace_back(r * p);
        }

        // run ik solver
        //if(!test)
        ik->solve(state, tipFrames, t0 + timeout);
        
        // get solution
        //if(!test)
        state = ik->getSolution();
        
        //if(test) for(auto ivar : ikparams.active_variables) state[ivar] = 0;
        
        //for(auto ivar : ikparams.active_variables) LOG(ivar, state[ivar]);
        
        // wrap angles
        for(auto ivar : ikparams.active_variables)
        {
            auto v = state[ivar];
            
            //LOG(ivar, v, modelInfo.getMin(ivar), modelInfo.getMax(ivar));
            
            /*v /= 2 * M_PI;
            v -= std::floor(v);
            v *= 2 * M_PI;
            if(v > M_PI) v -= 2 * M_PI;*/
            
            //LOG_VAR(ivar);
            
            if(robot_info.isRevolute(ivar) && robot_info.getClipMax(ivar) == DBL_MAX)
            {
                v *= (1.0 / (2 * M_PI));
                v -= std::floor(v);
                v += std::floor(robot_info.getMin(ivar) * (1.0 / (2 * M_PI)));
                if(v < robot_info.getMin(ivar)) v += 2 * M_PI;
                v *= (2 * M_PI);
            }
            
            state[ivar] = v;
        }

        // wrap angles
        robot_model->enforcePositionBounds(state.data());

        // map result to jointgroup variables
        {
            solution.clear();
            for(auto& joint_name : getJointNames())
            {
                auto* joint_model = robot_model->getJointModel(joint_name);
                if(!joint_model) continue;
                for(size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
                    solution.push_back(state.at(joint_model->getFirstVariableIndex() + vi));
            }
        }

        // run callback
        {
            if(!solution_callback.empty())
                solution_callback(ik_poses.front(), solution, error_code);
            else
                error_code.val = error_code.SUCCESS;
        }
        
        return error_code.val == error_code.SUCCESS;
    }
    
    // MoveIt version compatability
    // API changed from "const bool" to "bool"
    // Automatically select correct return type
    typedef decltype(((kinematics::KinematicsBase*)0)->supportsGroup(0)) supportsGroup_Result; 
    
    //virtual const bool supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out = NULL) const
    virtual supportsGroup_Result supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out = 0) const
    {
        LOG_FNC();
        //LOG_VAR(jmg->getName());
        return true;
    }
    
};

}

#undef LOG
#undef ERROR

PLUGINLIB_EXPORT_CLASS(bio_ik_kinematics_plugin::BioIKKinematicsPlugin, kinematics::KinematicsBase);




