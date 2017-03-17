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










struct IKGoalInfo
{
    size_t tip_index;
    double weight;
    double rotation_scale;
    double rotation_scale_sq;
    Frame frame;
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
    void setGoals(const std::vector<std::unique_ptr<Goal>>& goals2, const IKParams& params)
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

            if(auto* g = dynamic_cast<const LinkGoalBase*>(goal.get()))
            {
                auto* link_model = params.robot_model->getLinkModel(g->link_name);
                if(!link_model) ERROR("link not found", g->link_name);
                if(link_tip_indices[link_model->getLinkIndex()] < 0)
                {
                    link_tip_indices[link_model->getLinkIndex()] = tip_link_indices.size();
                    tip_link_indices.push_back(link_model->getLinkIndex());
                }
                goal_info.tip_index = link_tip_indices[link_model->getLinkIndex()];
                goal_info.weight = g->weight;
            }
            
            if(auto* g = dynamic_cast<const PositionGoal*>(goal.get()))
            {
                goal_info.frame.pos = g->position;
            }
            
            if(auto* g = dynamic_cast<const OrientationGoal*>(goal.get()))
            {
                goal_info.frame.rot = g->orientation;
            }
            
            if(auto* g = dynamic_cast<const PoseGoal*>(goal.get()))
            {
                goal_info.frame.pos = g->position;
                goal_info.frame.rot = g->orientation;
                goal_info.rotation_scale = g->rotation_scale;
            }
            
            goal_info.rotation_scale_sq = goal_info.rotation_scale * goal_info.rotation_scale;
            
            //LOG_VAR(goal_info.tip_index);
            
            /*LOG_VAR(goal_info.rotation_scale);
            LOG_VAR(goal_info.tip_index);
            LOG_VAR(goal_info.weight);
            LOG("goal_info.frame.pos", goal_info.frame.pos.x(), goal_info.frame.pos.y(), goal_info.frame.pos.z());
            LOG("goal_info.frame.rot", goal_info.frame.rot.x(), goal_info.frame.rot.y(), goal_info.frame.rot.z(), goal_info.frame.rot.w());*/
            
            goals.push_back(goal_info);
        }
        
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
    
    
    
    
    
    
    
    
    
    




    double computeFitness(const std::vector<Frame>& tip_frames, bool balanced = true)
    {
        FNPROFILER();
    
        double fitness_sum = 0.0;
        
        //LOG_VAR(request.goals.size());
        //LOG_VAR(tip_frames.size());

        for(auto& goal : request.goals)
        {
            //LOG_VAR(goal.tip_index);
        
            const auto& fa = goal.frame;
            const auto& fb = tip_frames[goal.tip_index];
            
            //LOG_VAR(fa);
            //LOG_VAR(fb);
            
            double tdist = (fa.pos - fb.pos).length2();

            double rdist = (fa.rot - fa.rot.nearest(fb.rot)).length2();

            rdist *= goal.rotation_scale_sq;

            double f = tdist + rdist;
            
            double w = goal.weight;
            f *= w * w;
            
            fitness_sum += f;
        }

        return fitness_sum;
    }

    
    /*
    double computeFitness(const std::vector<Frame>& frames, bool balanced = true)
    {
        FNPROFILER();
    
        double fitness_sum = 0.0;

        for(size_t tip_index = 0; tip_index < tipObjectives.size(); tip_index++)
        {
            const auto& fa = tipObjectives[tip_index];
            const auto& fb = frames[tip_index];
            
            double tdist = (fa.pos - fb.pos).length2();

            double rdist = (fa.rot - fa.rot.nearest(fb.rot)).length2();

            rdist *= request.tip_infos[tip_index].rotation_scale_sq;

            double f = tdist + rdist;
            
            double w = request.tip_infos[tip_index].weight;
            f *= w * w;
            
            fitness_sum += f;
        }

        return fitness_sum;
    }
    */
    
    double computeFitness(const std::vector<double>& genes, bool balanced = true)
    {
        //FNPROFILER();
        model.applyConfiguration(genes);
        return computeFitness(model.getTipFrames(), balanced);
    }
    
    
    
    
    
    

    
    
    
    
    
    
    
    
    


    
    
    
    
    
     
    





    virtual size_t concurrency() const { return 1; }
    
};


typedef IKBase IKSolver;



typedef Factory<IKSolver, const IKParams&> IKFactory;










}



