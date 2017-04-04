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
    JointVariable,
    CenterJoints,
};

struct IKGoalInfo
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
};

struct IKRequest
{
    double timeout;
    std::vector<double> initial_guess;
    std::vector<size_t> active_variables;
    std::vector<size_t> tip_link_indices;
    std::vector<IKGoalInfo> goals;
    std::vector<IKGoalInfo> secondary_goals;
    
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
                goal_info.frame.rot = g->orientation;
            }

            if(auto* g = dynamic_cast<const PoseGoal*>(goal))
            {
                goal_info.goal_type = GoalType::Pose;
                goal_info.frame.pos = g->position;
                goal_info.frame.rot = g->orientation;
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
                goal_info.axis = g->axis;
            }
            
            if(auto* g = dynamic_cast<const MaxDistanceGoal*>(goal))
            {
                goal_info.goal_type = GoalType::MaxDistance;
                goal_info.target = g->target;
                goal_info.distance = g->distance;
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
            
            goal_info.rotation_scale_sq = goal_info.rotation_scale * goal_info.rotation_scale;
            goal_info.weight_sq = goal_info.weight * goal_info.weight;
            
            //LOG_VAR(goal_info.tip_index);
            
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
        
        //std::sort(goals.begin(), goals.end(), [] (const IKGoalInfo& a, const IKGoalInfo& b) { return a.goal_type < b.goal_type; });
        
        updateActiveVariables(params);
        
        /*LOG_VAR(request.active_variables.size());
        for(auto& active_variable : request.active_variables) LOG_VAR(active_variable);
        
        LOG_VAR(request.tip_link_indices.size());
        for(auto& tli : request.tip_link_indices) LOG_VAR(tli);*/
        
        for(auto* pgg : { &goals, &secondary_goals })
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
            }
        }
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
    
    double dpos, drot, dtwist;

    void setParams(const IKParams& p)
    {
        auto& n = p.node_handle;
        
        n.param("full_circle", opt_angular_scale_full_circle, true);
        n.param("fitness_randomization", fitness_randomization, 0.0);
        
        n.param("dpos", dpos, DBL_MAX);
        n.param("drot", drot, DBL_MAX);
        n.param("dtwist", dtwist, 1e-5);
        if(dpos < 0.0 || dpos >= FLT_MAX) dpos = DBL_MAX;
        if(drot < 0.0 || drot >= FLT_MAX) drot = DBL_MAX;
        if(dtwist < 0.0 || dtwist >= FLT_MAX) dtwist = DBL_MAX;
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
    
    
    
    
    
    
    std::vector<double> minimal_displacement_factors;
    
    virtual void initialize(const IKRequest& request)
    {
        this->request = request;
        //tipObjectives = request.tip_objectives;
        model.initialize(request.tip_link_indices);
        //modelInfo.initialize(request.tip_link_indices);
        active_variables = request.active_variables;
        
        
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
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    double computeGoalFitness(const std::vector<IKGoalInfo>& goals, const Frame* tip_frames, const double* active_variable_positions)
    {
        double sum = 0.0;

        for(auto& goal : goals)
        {
            const auto& fa = goal.frame;
            const auto& fb = tip_frames[goal.tip_index];
            
            switch(goal.goal_type)
            {
            
                case GoalType::Position:
                {
                    sum += goal.weight_sq * (fa.pos - fb.pos).length2();
                    continue;
                }
                    
                case GoalType::Orientation:
                {
                    sum += goal.weight_sq * (fa.rot - fa.rot.nearest(fb.rot)).length2();
                    continue;
                }
                    
                case GoalType::Pose:
                {
                    sum += goal.weight_sq * (fa.pos - fb.pos).length2();
                    sum += goal.weight_sq * (fa.rot - fa.rot.nearest(fb.rot)).length2() * goal.rotation_scale_sq;
                    continue;
                }
                    
                case GoalType::LookAt:
                {
                    tf::Vector3 axis;
                    quat_mul_vec(fb.rot, goal.axis, axis);
                    //sum += (fb.pos + axis * axis.dot(goal.target - fb.pos)).distance2(goal.target) * goal.weight_sq / goal.target.distance2(fb.pos);
                    sum += (goal.target - fb.pos).normalized().distance2(axis.normalized()) * goal.weight_sq;
                    continue;
                }
                
                case GoalType::MaxDistance:
                {
                    double d = fmax(0.0, fb.pos.distance(goal.target) - goal.distance);
                    sum += d * d * goal.weight_sq;
                    continue;
                }
                
                case GoalType::JointVariable:
                {
                    if(goal.active_variable_index < 0) continue;
                    double d = active_variable_positions[goal.active_variable_index] - goal.variable_position;
                    sum += d * d * goal.weight_sq;
                    continue;
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
                    continue;
                }
                
                case GoalType::MinimalDisplacement:
                {
                    for(size_t i = 0; i < active_variables.size(); i++)
                    {
                        size_t ivar = active_variables[i];
                        double x = active_variable_positions[i] - request.initial_guess[ivar];
                        x *= minimal_displacement_factors[i];
                        x *= goal.weight;
                        sum += x * x;
                        //LOG(i, params.robot_model->getVariableNames()[active_variables[i]], minimal_displacement_factors[i]);
                    }
                    continue;
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
                    continue;
                }
            }
        }

        return sum;
    }
    
    
    
    
    
    double computeSecondaryFitnessActiveVariables(const double* active_variable_positions)
    {
        return computeGoalFitness(request.secondary_goals, 0, active_variable_positions);
    }
    
    double computeSecondaryFitnessAllVariables(const std::vector<double>& variable_positions)
    {
        return computeSecondaryFitnessActiveVariables(extractActiveVariables(variable_positions));
    }
    
    double computeFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        return computeGoalFitness(request.goals, tip_frames.data(), active_variable_positions);
    }
    
    
    
    
    
    bool checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
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
                
                case GoalType::LookAt:
                {
                    tf::Vector3 axis;
                    quat_mul_vec(fb.rot, goal.axis, axis);
                    double angle = axis.angle(goal.target - fb.pos);
                    if(drot != DBL_MAX)
                    {
                        double r_dist = angle;
                        r_dist = r_dist * 180 / M_PI;
                        if(!(r_dist <= drot)) return false;
                    }
                    if(dtwist != DBL_MAX)
                    {
                        if(!(angle <= dtwist)) return false;
                    }
                    continue;
                }
                
                case GoalType::MaxDistance:
                {
                    double d = fmax(0.0, fb.pos.distance(goal.target) - goal.distance);
                    if(dpos != DBL_MAX)
                    {
                        if(!(d <= dpos)) return false;
                    }
                    if(dtwist != DBL_MAX)
                    {
                        if(!(d <= dtwist)) return false;
                    }
                    continue;
                }
                
                case GoalType::JointVariable:
                {
                    if(goal.active_variable_index < 0) continue;
                    double d = active_variable_positions[goal.active_variable_index] - goal.variable_position;
                    d = fabs(d);
                    if(dpos != DBL_MAX && modelInfo.isPrismatic(active_variables[goal.active_variable_index]))
                    {
                        if(!(d <= dpos)) return false;
                    }
                    if(dpos != DBL_MAX && modelInfo.isRevolute(active_variables[goal.active_variable_index]))
                    {
                        if(!(d <= drot)) return false;
                    }
                    if(dtwist != DBL_MAX)
                    {
                        if(!(d <= dtwist)) return false;
                    }
                    continue;
                }
                
                default:
                {
                    return false;
                }
                
            }
        
        }
        
        return true;
    }
    
    bool checkSolution(const std::vector<double>& variable_positions, const std::vector<Frame>& tips)
    {
        return checkSolutionActiveVariables(tips, extractActiveVariables(variable_positions));
    }
    
    
    /*
    double computeFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        FNPROFILER();
    
        double sum = 0.0;
        double gain = 1.0;

        for(auto& goal : request.goals)
        {
            const auto& fa = goal.frame;
            const auto& fb = tip_frames[goal.tip_index];
            
            switch(goal.goal_type)
            {
            
            case GoalType::Position:
            {
                sum += goal.weight_sq * (fa.pos - fb.pos).length2();
                continue;
            }
                
            case GoalType::Orientation:
            {
                sum += goal.weight_sq * (fa.rot - fa.rot.nearest(fb.rot)).length2();
                continue;
            }
                
            case GoalType::Pose:
            {
                sum += goal.weight_sq * (fa.pos - fb.pos).length2();
                sum += goal.weight_sq * (fa.rot - fa.rot.nearest(fb.rot)).length2() * goal.rotation_scale_sq;
                continue;
            }
                
            case GoalType::LookAt:
            {
                tf::Vector3 axis;
                quat_mul_vec(fb.rot, goal.axis, axis);
                sum += (fb.pos + axis * axis.dot(goal.target - fb.pos)).distance2(goal.target);
                continue;
            }

            case GoalType::AvoidJointLimits:
            {
                for(size_t i = 0; i < active_variables.size(); i++)
                {
                    size_t ivar = active_variables[i];
                    if(modelInfo.getClipMax(ivar) == DBL_MAX) continue;
                    double x = (active_variable_positions[i] - modelInfo.getMin(ivar)) / modelInfo.getSpan(ivar) * 2 - 1;
                    
                    x = x * x;
                    
                    //x = 1.0 / (1.0 + 0.1 - x * x) - 1.0;
                    //x = 1 + x * x * goal.weight_sq;
                    //fitness_sum *= x;
                    
                    //x = 1.0 - sqrt(fmax(0.0, 1.0 - x * x));
                    
                    //x *= modelInfo.getMaxVelocityRcp(ivar) / modelInfo.getSpan(ivar);
                    
                    x *= goal.weight;
                    gain += x * x;
                }
                continue;
            }
            
            case GoalType::MinimalDisplacement:
            {
                for(size_t i = 0; i < active_variables.size(); i++)
                {
                    size_t ivar = active_variables[i];
                    double x = active_variable_positions[i] - request.initial_guess[ivar];
                    //LOG(i, minimal_displacement_factors[i]);
                    x *= minimal_displacement_factors[i];
                    x *= goal.weight;
                    //gain += x * x;
                    gain += fabs(x);
                    //sum += x * x;
                }
                continue;
            }
            
            }
        }

        return sum * gain;
    }
    */
    
    std::vector<double> temp_active_variable_positions;
    
    double* extractActiveVariables(const std::vector<double>& variable_positions)
    {
        temp_active_variable_positions.resize(active_variables.size());
        for(size_t i = 0; i < temp_active_variable_positions.size(); i++)
            temp_active_variable_positions[i] = variable_positions[active_variables[i]];
        return temp_active_variable_positions.data();
    }
    
    double computeFitness(const std::vector<double>& variable_positions, const std::vector<Frame>& tip_frames)
    {
        return computeFitnessActiveVariables(tip_frames, extractActiveVariables(variable_positions));
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



