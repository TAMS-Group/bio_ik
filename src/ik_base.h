// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#pragma once

#include <bio_ik/bio_ik.h>

#include "forward_kinematics.h"
#include "frame.h"
#include "robot_helpers.h"
#include "utils.h"

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
    MinDistance,
    Touch,
    Line,
    AvoidJointLimits,
    MinimalDisplacement,
    JointVariable,
    CenterJoints,
    JointFunction,
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
    std::vector<ssize_t> variable_indices;
};

class IKRequest
{
private:
    std::vector<int> joint_usage;
    std::vector<ssize_t> link_tip_indices;
    void updateActiveVariables(const IKParams& params);

public:
    double timeout;
    std::vector<double> initial_guess;
    std::vector<size_t> active_variables;
    std::vector<size_t> tip_link_indices;
    std::vector<IKGoalInfo> goals;
    std::vector<IKGoalInfo> secondary_goals;
    void setGoals(const std::vector<const Goal*>& goals2, const IKParams& params);
};

struct IKBase2
{
    volatile int canceled;
    virtual void initialize(const IKRequest& request) = 0;
    virtual void step() = 0;
    virtual size_t concurrency() const = 0;
    virtual const std::vector<double>& getSolution() const = 0;
    virtual ~IKBase2() {}
};

struct RandomBase
{
    // std::mt19937 rng;
    std::minstd_rand rng;
    // std::ranlux24 rng;
    // std::knuth_b rng;
    // std::default_random_engine rng;

    inline double random() { return std::uniform_real_distribution<double>(0, 1)(rng); }

    inline std::size_t random_index(std::size_t s) { return std::uniform_int_distribution<size_t>(0, s - 1)(rng); }

    std::normal_distribution<double> normal_distribution;
    inline double random_gauss() { return normal_distribution(rng); }

    inline double random(double min, double max) { return random() * (max - min) + min; }

    template <class e> inline e& random_element(std::vector<e>& l) { return l[random_index(l.size())]; }

    template <class e> inline const e& random_element(const std::vector<e>& l) { return l[random_index(l.size())]; }

    XORShift64 _xorshift;
    inline size_t fast_random_index(size_t mod) { return _xorshift() % mod; }
    template <class T> inline const T& fast_random_element(const std::vector<T>& v) { return v[fast_random_index(v.size())]; }

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

    RandomBase()
        : rng(std::random_device()())
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

    IKBase(const IKParams& p)
        : model(p.robot_model)
        , modelInfo(p.robot_model)
        , params(p)
    {
        setParams(p);
        thread_index = p.thread_index;
    }

    IKRequest request;

    std::vector<Frame> null_tip_frames;

    std::vector<double> minimal_displacement_factors;

    virtual void initialize(const IKRequest& request)
    {
        this->request = request;
        model.initialize(request.tip_link_indices);
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

        null_tip_frames.resize(request.tip_link_indices.size());
    }

    double computeGoalFitness(const IKGoalInfo& goal, const Frame* tip_frames, const double* active_variable_positions);

    double computeGoalFitness(const std::vector<IKGoalInfo>& goals, const Frame* tip_frames, const double* active_variable_positions);

    std::vector<double> joint_transmission_goal_temp, joint_transmission_goal_temp2;

    double computeSecondaryFitnessActiveVariables(const double* active_variable_positions) { return computeGoalFitness(request.secondary_goals, null_tip_frames.data(), active_variable_positions); }

    double computeSecondaryFitnessAllVariables(const std::vector<double>& variable_positions) { return computeSecondaryFitnessActiveVariables(extractActiveVariables(variable_positions)); }

    double computeFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions) { return computeGoalFitness(request.goals, tip_frames.data(), active_variable_positions); }

    double computeCombinedFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        double ret = 0.0;
        ret += computeGoalFitness(request.goals, tip_frames.data(), active_variable_positions);
        ret += computeGoalFitness(request.secondary_goals, null_tip_frames.data(), active_variable_positions);
        return ret;
    }

    bool checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions);

    bool checkSolution(const std::vector<double>& variable_positions, const std::vector<Frame>& tips) { return checkSolutionActiveVariables(tips, extractActiveVariables(variable_positions)); }

    std::vector<double> temp_active_variable_positions;

    double* extractActiveVariables(const std::vector<double>& variable_positions)
    {
        temp_active_variable_positions.resize(active_variables.size());
        for(size_t i = 0; i < temp_active_variable_positions.size(); i++)
            temp_active_variable_positions[i] = variable_positions[active_variables[i]];
        return temp_active_variable_positions.data();
    }

    double computeFitness(const std::vector<double>& variable_positions, const std::vector<Frame>& tip_frames) { return computeFitnessActiveVariables(tip_frames, extractActiveVariables(variable_positions)); }

    double computeFitness(const std::vector<double>& variable_positions)
    {
        model.applyConfiguration(variable_positions);
        return computeFitness(variable_positions, model.getTipFrames());
    }

    virtual ~IKBase() {}

    virtual size_t concurrency() const { return 1; }
};

typedef IKBase IKSolver;

typedef Factory<IKSolver, const IKParams&> IKFactory;
}
