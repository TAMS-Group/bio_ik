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




// implement BioIKKinematicsQueryOptions

namespace bio_ik
{

std::unordered_set<const void*> idBioIKKinematicsQueryOptions;

BioIKKinematicsQueryOptions::BioIKKinematicsQueryOptions() : replace(false)
{
    idBioIKKinematicsQueryOptions.insert(this);
}

BioIKKinematicsQueryOptions::~BioIKKinematicsQueryOptions()
{
    idBioIKKinematicsQueryOptions.erase(this);
}

bool isBioIKKinematicsQueryOptions(const void* ptr)
{
    return idBioIKKinematicsQueryOptions.find(ptr) != idBioIKKinematicsQueryOptions.end();
}

const BioIKKinematicsQueryOptions* toBioIKKinematicsQueryOptions(const void* ptr)
{
    if(isBioIKKinematicsQueryOptions(ptr))
        return (const BioIKKinematicsQueryOptions*)ptr;
    else
        return 0;
}

}




// BioIK Kinematics Plugin

namespace bio_ik_kinematics_plugin
{


typedef IKParallel PluginIKSolver;

struct BioIKKinematicsPlugin : kinematics::KinematicsBase
{
    std::vector<std::string> joint_names, link_names;
    MoveItRobotModelConstPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    mutable std::unique_ptr<PluginIKSolver> ik;
    mutable std::vector<double> state, temp;
    mutable std::unique_ptr<moveit::core::RobotState> temp_state;
    mutable std::vector<Frame> tipFrames;
    RobotInfo robot_info;
    bool enable_profiler;

    BioIKKinematicsPlugin()
    {
        enable_profiler = false;
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

    mutable std::vector<std::unique_ptr<Goal>> default_goals;

    mutable std::vector<const bio_ik::Goal*> all_goals;

    IKParams ikparams;

    mutable IKRequest ikrequest;



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

        ros::NodeHandle node_handle("~");
        std::string rdesc;
        node_handle.searchParam(robot_description_, rdesc);
        node_handle = ros::NodeHandle(rdesc + "_kinematics/" + group_name_);

        //bool enable_profiler;
        node_handle.param("profiler", enable_profiler, false);
        //if(enable_profiler) Profiler::start();

        robot_info = RobotInfo(robot_model);

        ikparams.robot_model = robot_model;
        ikparams.node_handle = node_handle;
        ikparams.joint_model_group = joint_model_group;

        temp_state.reset(new moveit::core::RobotState(robot_model));

        ik.reset(new PluginIKSolver(ikparams));



        {

            BLOCKPROFILER("default ik goals");

            default_goals.clear();

            for(size_t i = 0; i < tip_frames_.size(); i++)
            {
                PoseGoal* goal = new PoseGoal();

                goal->link_name = tip_frames_[i];

                //LOG_VAR(goal->link_name);

                goal->rotation_scale = 0.5;

                node_handle.param("rotation_scale", goal->rotation_scale, goal->rotation_scale);

                bool position_only_ik = false;
                node_handle.param("position_only_ik", position_only_ik, position_only_ik);
                if(position_only_ik) goal->rotation_scale = 0;

                default_goals.emplace_back(goal);
            }

            /*{
                double weight = 0;
                node_handle.param("avoid_joint_limits_weight", weight, weight);
                if(weight > 0.0)
                {
                    auto* avoid_joint_limits_goal = new bio_ik::AvoidJointLimitsGoal();
                    avoid_joint_limits_goal->weight = weight;
                    default_goals.emplace_back(avoid_joint_limits_goal);
                }
            }

            {
                double weight = 0;
                node_handle.param("minimal_displacement_weight", weight, weight);
                if(weight > 0.0)
                {
                    auto* minimal_displacement_goal = new bio_ik::MinimalDisplacementGoal();
                    minimal_displacement_goal->weight = weight;
                    default_goals.emplace_back(minimal_displacement_goal);
                }
            }*/

            {
                typedef XmlRpc::XmlRpcValue var;

                var goals;
                node_handle.param("goals", goals, goals);

                if(goals.getType() == var::TypeArray)
                {
                    for(int i = 0; i < goals.size(); i++)
                    {
                        if(goals[i].getType() == var::TypeStruct)
                        {
                            auto d = XmlRpcReader(goals[i]);

                            std::string type;
                            d.param("type", type);
                            LOG("goal", "type", type);

                            if(type == "LookAt")
                            {
                                auto* g = new LookAtGoal();
                                d.param("weight", g->weight);
                                d.param("link", g->link_name);
                                d.param("axis", g->axis);
                                d.param("target", g->target);
                                default_goals.emplace_back(g);
                                continue;
                            }

                            if(type == "Position")
                            {
                                auto* g = new PositionGoal();
                                d.param("weight", g->weight);
                                d.param("link", g->link_name);
                                d.param("position", g->position);
                                default_goals.emplace_back(g);
                                continue;
                            }

                            if(type == "Orientation")
                            {
                                auto* g = new OrientationGoal();
                                d.param("weight", g->weight);
                                d.param("link", g->link_name);
                                d.param("orientation", g->orientation);
                                default_goals.emplace_back(g);
                                continue;
                            }

                            if(type == "Pose")
                            {
                                auto* g = new PoseGoal();
                                d.param("weight", g->weight);
                                d.param("link", g->link_name);
                                d.param("position", g->position);
                                d.param("orientation", g->orientation);
                                default_goals.emplace_back(g);
                                continue;
                            }

                            if(type == "JointVariable")
                            {
                                auto* g = new JointVariableGoal();
                                d.param("weight", g->weight);
                                d.param("variable", g->variable_name);
                                d.param("position", g->variable_position);
                                d.param("secondary", g->secondary);
                                default_goals.emplace_back(g);
                                continue;
                            }

                            if(type == "MaxDistance")
                            {
                                auto* g = new MaxDistanceGoal();
                                d.param("weight", g->weight);
                                d.param("link", g->link_name);
                                d.param("target", g->target);
                                d.param("distance", g->distance);
                                default_goals.emplace_back(g);
                                continue;
                            }

                            if(type == "AvoidJointLimits")
                            {
                                auto* g = new AvoidJointLimitsGoal();
                                d.param("weight", g->weight);
                                default_goals.emplace_back(g);
                                continue;
                            }

                            if(type == "MinimalDisplacement")
                            {
                                auto* g = new MinimalDisplacementGoal();
                                d.param("weight", g->weight);
                                default_goals.emplace_back(g);
                                continue;
                            }

                            ERROR("invalid goal type", type);
                        }
                    }
                }
            }
        }


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
                                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
                                const moveit::core::RobotState* context_state = NULL) const
    {
        double t0 = ros::Time::now().toSec();

        //timeout = 0.1;

        //LOG("a");

        if(enable_profiler) Profiler::start();

        auto* bio_ik_options = toBioIKKinematicsQueryOptions(&options);

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

        if(!bio_ik_options || !bio_ik_options->replace)
        {
            // transform tips to baseframe
            tipFrames.clear();
            for(size_t i = 0; i < ik_poses.size(); i++)
            {
                Eigen::Affine3d p, r;
                tf::poseMsgToEigen(ik_poses[i], p);
                if(context_state)
                {
                    r = context_state->getGlobalLinkTransform(getBaseFrame());
                }
                else
                {
                    if(i == 0) temp_state->setToDefaultValues();
                    r = temp_state->getGlobalLinkTransform(getBaseFrame());
                }
                tipFrames.emplace_back(r * p);
            }
        }

        // init ik

        ikrequest.timeout = t0 + timeout;
        ikrequest.initial_guess = state;

        //for(auto& v : state) LOG("var", &v - &state.front(), v);

        //ikrequest.tip_objectives = tipFrames;

        /*for(size_t i = 0; i < ikrequest.goals.size(); i++)
        {
            ikrequest.goals[i].frame = tipFrames[i];
        }*/

        //LOG("---");

        /*{
            BLOCKPROFILER("ik goals");
            std::vector<std::unique_ptr<Goal>> goals;
            for(size_t i = 0; i < tip_frames_.size(); i++)
            {
                //if(rand() % 2) break;
                PoseGoal* goal = new PoseGoal();
                goal->link_name = tip_frames_[i];
                goal->position = tipFrames[i].pos;
                goal->orientation = tipFrames[i].rot;
                goals.emplace_back(goal);
                //if(rand() % 20) break;
            }
            //std::random_shuffle(goals.begin(), goals.end());
            //LOG_VAR(goals.size());
            setRequestGoals(ikrequest, goals, ikparams);
        }*/

        {
            if(!bio_ik_options || !bio_ik_options->replace)
            {
                for(size_t i = 0; i < tip_frames_.size(); i++)
                {
                    auto* goal = (PoseGoal*)default_goals[i].get();
                    goal->position = tipFrames[i].pos;
                    goal->orientation = tipFrames[i].rot;
                }
            }

            all_goals.clear();

            if(!bio_ik_options || !bio_ik_options->replace)
                for(auto& goal : default_goals)
                    all_goals.push_back(goal.get());

            if(bio_ik_options)
                for(auto& goal : bio_ik_options->goals)
                    all_goals.push_back(goal.get());

            ikrequest.setGoals(all_goals, ikparams);
            //ikrequest.setGoals(default_goals, ikparams);
        }

        {
            BLOCKPROFILER("ik init");
            ik->initialize(ikrequest);
        }

        // run ik solver
        ik->solve();

        // get solution
        state = ik->getSolution();

        // wrap angles
        for(auto ivar : ikrequest.active_variables)
        {
            auto v = state[ivar];
            // TODO: correct ???
            // TODO: force close to initial_guess ???
            if(robot_info.isRevolute(ivar))
            {
                if(v < robot_info.getMin(ivar))
                {
                    v += std::ceil((robot_info.getMin(ivar) - v) * (1.0 / (2.0 * M_PI))) * (2.0 * M_PI);
                }
                if(v > robot_info.getMax(ivar))
                {
                    v += std::floor((robot_info.getMax(ivar) - v) * (1.0 / (2.0 * M_PI))) * (2.0 * M_PI);
                }
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

        // return an error if an accurate solution was requested, but no accurate solution was found
        if(!ik->getSuccess() && !options.return_approximate_solution)
        {
            error_code.val = error_code.NO_IK_SOLUTION;
            return false;
        }

        // callback?
        if(!solution_callback.empty())
        {
            // run callback
            solution_callback(ik_poses.front(), solution, error_code);

            // return success if callback has accepted the solution
            return error_code.val == error_code.SUCCESS;
        }
        else
        {
            // return success
            error_code.val = error_code.SUCCESS;
            return true;
        }

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







// register plugin

#undef LOG
#undef ERROR
PLUGINLIB_EXPORT_CLASS(bio_ik_kinematics_plugin::BioIKKinematicsPlugin, kinematics::KinematicsBase);
