// Bio IK for ROS
// Philipp Ruppel

#pragma once

#include <bio_ik/bio_ik.h>

#include "utils.h"
#include "frame.h"
#include "robot_helpers.h"
#include "forward_kinematics.h"

#include <mutex>
 
namespace bio_ik
{















struct IKParams
{
    MoveItRobotModelConstPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    ros::NodeHandle node_handle;
    size_t thread_index;
};








enum class GoalType : uint32_t
{
    Null,
    Position, 
    Orientation, 
    Pose, 
    LookAt, 
    MaxDistance,
    AvoidJointLimits,
    MinimalDisplacement,
};

struct IKGoalInfo
{
    GoalType goal_type;
    size_t tip_index;
    double weight;
    double weight_sq;
    double rotation_scale;
    double rotation_scale_sq;
    Frame frame;
    tf::Vector3 target;
    tf::Vector3 axis;
};

struct IKRequest
{
    double timeout;
    std::vector<double> initial_guess;
    std::vector<size_t> active_variables;
    std::vector<size_t> tip_link_indices;
    std::vector<IKGoalInfo> goals;
    
private:
    std::vector<int> joint_usage;
    std::vector<ssize_t> link_tip_indices;
    void updateActiveVariables(const IKParams& params)
    {
        auto& robot_model = params.robot_model;
        auto* joint_model_group = params.joint_model_group;
        joint_usage.resize(robot_model->getJointModelCount());
        for(auto& u : joint_usage) u = 0;
        for(auto tip_index : tip_link_indices)
            for(auto* link_model = robot_model->getLinkModels()[tip_index]; link_model; link_model = link_model->getParentLinkModel())
                joint_usage[link_model->getParentJointModel()->getJointIndex()] = 1;
        active_variables.clear();
        for(auto* joint_model : joint_model_group->getActiveJointModels())
        //for(auto* joint_model : robot_model->getJointModels())
            if(joint_usage[joint_model->getJointIndex()] && !joint_model->getMimic())
                for(size_t ivar = joint_model->getFirstVariableIndex(); ivar < joint_model->getFirstVariableIndex() + joint_model->getVariableCount(); ivar++)
                    active_variables.push_back(ivar);
    }
    
public:
    //template<class GOAL_PTR>
    //void setGoals(const std::vector<std::unique_ptr<Goal>>& goals2, const IKParams& params)
    //void setGoals(const std::vector<GOAL_PTR>& goals2, const IKParams& params)
    void setGoals(const std::vector<const Goal*>& goals2, const IKParams& params)
    {
        link_tip_indices.clear();
        link_tip_indices.resize(params.robot_model->getLinkModelCount(), -1);
        tip_link_indices.clear();
        
        goals.clear();
        for(auto& goal : goals2)
        {
            IKGoalInfo goal_info;
            
            goal_info.weight = 1;
            goal_info.rotation_scale = 0.5;
            goal_info.frame = Frame::identity();
            goal_info.goal_type = GoalType::Null;
            
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
                goal_info.frame.rot = g->orientation;
            }

            if(auto* g = dynamic_cast<const PoseGoal*>(goal))
            {
                goal_info.goal_type = GoalType::Pose;
                goal_info.frame.pos = g->position;
                goal_info.frame.rot = g->orientation;
                goal_info.rotation_scale = g->rotation_scale;
            }
            
            if(auto* g = dynamic_cast<const LookAtGoal*>(goal))
            {
                goal_info.goal_type = GoalType::LookAt;
                goal_info.target = g->target;
                goal_info.axis = g->axis;
            }
            
            if(auto* g = dynamic_cast<const AvoidJointLimitsGoal*>(goal))
            {
                goal_info.goal_type = GoalType::AvoidJointLimits;
            }
            
            if(auto* g = dynamic_cast<const MinimalDisplacementGoal*>(goal))
            {
                goal_info.goal_type = GoalType::MinimalDisplacement;
            }
            
            goal_info.rotation_scale_sq = goal_info.rotation_scale * goal_info.rotation_scale;
            goal_info.weight_sq = goal_info.weight * goal_info.weight;
            
            //LOG_VAR(goal_info.tip_index);
            
            /*LOG_VAR(goal_info.rotation_scale);
            LOG_VAR(goal_info.tip_index);
            LOG_VAR(goal_info.weight);
            LOG("goal_info.frame.pos", goal_info.frame.pos.x(), goal_info.frame.pos.y(), goal_info.frame.pos.z());
            LOG("goal_info.frame.rot", goal_info.frame.rot.x(), goal_info.frame.rot.y(), goal_info.frame.rot.z(), goal_info.frame.rot.w());*/
            
            goals.push_back(goal_info);
        }
        
        std::sort(goals.begin(), goals.end(), [] (const IKGoalInfo& a, const IKGoalInfo& b) { return a.goal_type < b.goal_type; });
        
        updateActiveVariables(params);
        
        /*LOG_VAR(request.active_variables.size());
        for(auto& active_variable : request.active_variables) LOG_VAR(active_variable);
        
        LOG_VAR(request.tip_link_indices.size());
        for(auto& tli : request.tip_link_indices) LOG_VAR(tli);*/
    }
};


















struct IKBase2
{
    volatile int canceled;
    virtual void initialize(const IKRequest& request) = 0;
    virtual void step() = 0;
    virtual size_t concurrency() const = 0;
    virtual const std::vector<double>& getSolution() const = 0;
    virtual ~IKBase2() { }
};

struct RandomBase
{
    //std::random_device rdev;
    
    //std::mt19937 rng;
    std::minstd_rand rng;
    //std::ranlux24 rng;
    //std::knuth_b rng;
    //std::default_random_engine rng;


    inline double random() { return std::uniform_real_distribution<double>(0, 1)(rng); }

    inline std::size_t random_index(std::size_t s) { return std::uniform_int_distribution<size_t>(0, s - 1)(rng); }
    
    std::normal_distribution<double> normal_distribution;
    inline double random_gauss() { return normal_distribution(rng); }
    
    inline double random(double min, double max) { return random() * (max - min) + min; }

    template<class e>
    inline e& random_element(std::vector<e>& l) { return l[random_index(l.size())]; }
    
    template<class e>
    inline const e& random_element(const std::vector<e>& l) { return l[random_index(l.size())]; }

    
    XORShift64 _xorshift;
    inline size_t fast_random_index(size_t mod)
    {
        return _xorshift() % mod;
    }
    template<class T>
    inline const T& fast_random_element(const std::vector<T>& v)
    {
        return v[fast_random_index(v.size())];
    }
    
    static const size_t random_buffer_size = 1024 * 1024 * 8;
    
    const double* make_random_buffer()
    {
        static std::vector<double> buf;
        buf.resize(random_buffer_size);
        for(auto& r : buf) 
            r = random();
        return buf.data();
    }
    const double* random_buffer;
    size_t random_buffer_index;
    inline double fast_random()
    {
        double r = random_buffer[random_buffer_index & (random_buffer_size - 1)];
        random_buffer_index++;
        return r;
    }
    
    const double* make_random_gauss_buffer()
    {
        static std::vector<double> buf;
        buf.resize(random_buffer_size);
        for(auto& r : buf) 
            r = random_gauss();
        return buf.data();
    }
    const double* random_gauss_buffer;
    size_t random_gauss_index;
    inline double fast_random_gauss()
    {
        double r = random_gauss_buffer[random_gauss_index & (random_buffer_size - 1)];
        random_gauss_index++;
        return r;
    }
    inline const double* fast_random_gauss_n(size_t n)
    {
        size_t i = random_gauss_index;
        random_gauss_index += n;
        if(random_gauss_index >= random_buffer_size) i = 0, random_gauss_index = n;
        return random_gauss_buffer + i;
    }


    RandomBase() :
        rng(std::random_device()())
    {
        random_buffer = make_random_buffer();
        random_buffer_index = _xorshift();
        random_gauss_buffer = make_random_gauss_buffer();
        random_gauss_index = _xorshift();
    }
};

struct IKBase : IKBase2, RandomBase
{
    IKParams params;
    RobotFK model;
    RobotInfo modelInfo;
    //HeuristicErrorTree heuristicErrorTree;
    //std::vector<Frame> tipObjectives;
    std::vector<size_t> active_variables;
    const std::vector<size_t>& getGenes() const { return active_variables; }
    
    bool opt_angular_scale_full_circle;
    double fitness_randomization;

    int thread_index;

    void setParams(const IKParams& p)
    {
        auto& n = p.node_handle;
        n.param("full_circle", opt_angular_scale_full_circle, true);
        n.param("fitness_randomization", fitness_randomization, 0.0);
    }

    IKBase(const IKParams& p) : 
        model(p.robot_model), 
        modelInfo(p.robot_model),
        //heuristicErrorTree(p.robot_model, p.tip_frames),
        params(p)
    {
        setParams(p);
        thread_index = p.thread_index;
    }
    
    virtual ~IKBase()
    {
    }
    
    IKRequest request;
    
    virtual void initialize(const IKRequest& request)
    {
        this->request = request;
        //tipObjectives = request.tip_objectives;
        model.initialize(request.tip_link_indices);
        //modelInfo.initialize(request.tip_link_indices);
        active_variables = request.active_variables;
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    double computeFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        FNPROFILER();
    
        double fitness_sum = 0.0;

        for(auto& goal : request.goals)
        {
            const auto& fa = goal.frame;
            const auto& fb = tip_frames[goal.tip_index];
            
            switch(goal.goal_type)
            {
            
            case GoalType::Position:
            {
                fitness_sum += goal.weight_sq * (fa.pos - fb.pos).length2();
                continue;
            }
                
            case GoalType::Orientation:
            {
                fitness_sum += goal.weight_sq * (fa.rot - fa.rot.nearest(fb.rot)).length2();
                continue;
            }
                
            case GoalType::Pose:
            {
                fitness_sum += goal.weight_sq * (fa.pos - fb.pos).length2();
                fitness_sum += goal.weight_sq * (fa.rot - fa.rot.nearest(fb.rot)).length2() * goal.rotation_scale_sq;
                continue;
            }
                
            case GoalType::LookAt:
            {
                tf::Vector3 axis;
                quat_mul_vec(fb.rot, goal.axis, axis);
                fitness_sum += (fb.pos + axis * axis.dot(goal.target - fb.pos)).distance2(goal.target);
                continue;
            }
            
            /*case GoalType::AvoidJointLimits:
            {
                for(size_t i = 0; i < active_variables.size(); i++)
                {
                    size_t ivar = active_variables[i];
                    if(modelInfo.getClipMax(ivar) == DBL_MAX) continue;
                    double x = (active_variable_positions[i] - modelInfo.getMin(ivar)) / modelInfo.getSpan(ivar);
                    x = x * 2 - 1;
                    x *= goal.weight;
                    fitness_sum += x * x;
                }
            }*/
            
            case GoalType::AvoidJointLimits:
            {
                double s = 0.0;
                for(size_t i = 0; i < active_variables.size(); i++)
                {
                    size_t ivar = active_variables[i];
                    if(modelInfo.getClipMax(ivar) == DBL_MAX) continue;
                    double x = (active_variable_positions[i] - modelInfo.getMin(ivar)) / modelInfo.getSpan(ivar);
                    x = x * 2 - 1;
                    //x = 1.0 / (1.0 + 1.0 / (goal.weight * goal.weight) - x * x);
                    //x = 1 + x * x * goal.weight_sq;
                    //fitness_sum *= x;
                    s += x * x;
                }
                fitness_sum *= 1 + s * goal.weight_sq;
                continue;
            }
            
            case GoalType::MinimalDisplacement:
            {
                double s = 0.0;
                for(size_t i = 0; i < active_variables.size(); i++)
                {
                    size_t ivar = active_variables[i];
                    double x = active_variable_positions[i] - request.initial_guess[ivar];
                    x = x * 2 - 1;
                    s += x * x;
                }
                fitness_sum *= 1 + s * goal.weight_sq;
                continue;
            }
            
            }
        }
        
        /*for(size_t i = 0; i < active_variables.size(); i++)
        {
            size_t ivar = active_variables[i];
            if(modelInfo.getClipMax(ivar) == DBL_MAX) continue;
            double x = (active_variable_positions[i] - modelInfo.getMin(ivar)) / modelInfo.getSpan(ivar);
            x = x * 2 - 1;
            //x = 1.0 / (1.0 + 1.0 / (goal.weight * goal.weight) - x * x);
            x = 1 + x * x * 1;
            fitness_sum *= x;
        }*/

        return fitness_sum;
    }
    
    std::vector<double> temp_active_variable_positions;
    
    double computeFitness(const std::vector<double>& variable_positions, const std::vector<Frame>& tip_frames)
    {
        temp_active_variable_positions.resize(active_variables.size());
        for(size_t i = 0; i < temp_active_variable_positions.size(); i++)
            temp_active_variable_positions[i] = variable_positions[active_variables[i]];
        return computeFitnessActiveVariables(tip_frames, temp_active_variable_positions.data());
    }

    double computeFitness(const std::vector<double>& variable_positions)
    {
        //FNPROFILER();
        model.applyConfiguration(variable_positions);
        return computeFitness(variable_positions, model.getTipFrames());
    }
    
    
    
    
    
    

    
    
    
    
    
    
    
    
    


    
    
    
    
    
     
    





    virtual size_t concurrency() const { return 1; }
    
};


typedef IKBase IKSolver;



typedef Factory<IKSolver, const IKParams&> IKFactory;










}



