// Bio IK for ROS
// Philipp Ruppel

#pragma once

#include "utils.h"
#include "frame.h"
#include "robot_helpers.h"
#include "forward_kinematics.h"

#include <mutex>
 
namespace bio_ik
{

struct IKTipInfo
{
    bool position_only_ik;
    double weight;
    IKTipInfo() :
        position_only_ik(0),
        weight(1)
    {
    }
};

struct IKParams
{
    MoveItRobotModelConstPtr robot_model;
    std::vector<std::string> tip_frames;
    std::vector<size_t> active_variables;
    ros::NodeHandle node_handle;
    std::vector<IKTipInfo> tip_infos;
    size_t thread_index;
};


struct IKBase2
{
    volatile int canceled;
    virtual void initialize(const std::vector<double>& initial_guess, const std::vector<Frame>& tipObjectives) = 0;
    virtual void step() = 0;
    virtual size_t concurrency() const = 0;
    virtual const std::vector<double>& getSolution() const = 0;
    virtual ~IKBase2() { }
};


struct IKBase : IKBase2
{
    IKParams params;
    RobotFK model;
    RobotInfo modelInfo;
    HeuristicErrorTree heuristicErrorTree;
    std::vector<Frame> tipObjectives;
    std::vector<size_t> active_variables;
    const std::vector<size_t>& getGenes() const { return active_variables; }
    
    //std::random_device rdev;
    
    //std::mt19937 rng;
    std::minstd_rand rng;
    //std::ranlux24 rng;
    //std::knuth_b rng;
    //std::default_random_engine rng;
    
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
        model(p.robot_model, p.tip_frames), 
        //rdev(), 
        //rng(rdev()), 
        rng(std::random_device()()), 
        modelInfo(p.robot_model, p.tip_frames),
        heuristicErrorTree(p.robot_model, p.tip_frames),
        params(p)
    {
        active_variables = p.active_variables;
        setParams(p);
        thread_index = p.thread_index;
    }
    
    virtual ~IKBase()
    {
    }
    
    virtual void initialize(const std::vector<double>& initial_guess, const std::vector<Frame>& tipObjectives)
    {
        this->tipObjectives = tipObjectives;
    }
    
    
    
    
    
    
    
    
    
    inline double random() { return std::uniform_real_distribution<double>(0, 1)(rng); }

    inline std::size_t random_index(std::size_t s) { return std::uniform_int_distribution<size_t>(0, s - 1)(rng); }
    
    std::normal_distribution<double> normal_distribution;
    inline double random_gauss() { return normal_distribution(rng); }
    
    inline double random(double min, double max) { return random() * (max - min) + min; }

    template<class e>
    inline e& random_element(std::vector<e>& l) { return l[random_index(l.size())]; }
    
    template<class e>
    inline const e& random_element(const std::vector<e>& l) { return l[random_index(l.size())]; }





    
    double computeFitness(const std::vector<Frame>& frames, bool balanced = true)
    {
        FNPROFILER();
    
        double fitness_sum = 0.0;

        for(size_t tip_index = 0; tip_index < tipObjectives.size(); tip_index++)
        {
            const auto& fa = tipObjectives[tip_index];
            const auto& fb = frames[tip_index];
            
            double tdist = (fa.pos - fb.pos).length2();
            
            //double rdist = (fa.rot - fb.rot).length2();
            
            /*Quaternion qa = fa.rot;
            Quaternion qb = fb.rot;
            if(qa.w() < 0) qa = -qa;
            if(qb.w() < 0) qb = -qb;
            double rdist = (qa - qb).length2();*/
            
            //double rdist = fa.rot.angle(fb.rot);
            
            /*double rdist = fa.rot.angleShortestPath(fb.rot);
            rdist *= rdist;*/
                
            double rdist = (fa.rot - fa.rot.nearest(fb.rot)).length2();
            
            double rscale = heuristicErrorTree.getChainLength(tip_index) * (0.5 / M_PI);
            rdist *= rscale * rscale;
            
            double f = tdist + rdist;
            
            double w = params.tip_infos[tip_index].weight;
            f *= w * w;
            
            fitness_sum += f;
        }
        
        //if(!std::isfinite(fitness_sum)) ERROR("fitness inf");
        
        return fitness_sum;
    }

    
    double computeFitness(const std::vector<double>& genes, bool balanced = true)
    {
        //FNPROFILER();
        model.applyConfiguration(genes);
        return computeFitness(model.getTipFrames(), balanced);
    }
    
    
    
    
    
    

    
    
    
    
    
    
    
    
    


    
    
    
    
    
     
    
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
    
    const size_t random_buffer_size = 1024 * 1024 * 8;
    
    const double* make_random_buffer()
    {
        static std::vector<double> buf;
        buf.resize(random_buffer_size);
        for(auto& r : buf) 
            r = random();
        return buf.data();
    }
    const double* random_buffer = make_random_buffer();
    size_t random_buffer_index = _xorshift();
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
    const double* random_gauss_buffer = make_random_gauss_buffer();
    size_t random_gauss_index = _xorshift();
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

    










    virtual size_t concurrency() const { return 1; }
    
};


typedef IKBase IKSolver;



typedef Factory<IKSolver, const IKParams&> IKFactory;










}



