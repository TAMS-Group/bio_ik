// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#pragma once

#include <bio_ik/bio_ik.h>

#include "forward_kinematics.h"
#include "frame.h"
#include "problem.h"
#include "robot_info.h"
#include "utils.h"

namespace bio_ik
{

struct IKParams
{
    MoveItRobotModelConstPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    ros::NodeHandle node_handle;
};

struct Random
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
        // LOG("make_random_gauss_buffer");
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

    Random()
        : rng(std::random_device()())
    {
        random_buffer = make_random_buffer();
        random_buffer_index = _xorshift();
        random_gauss_buffer = make_random_gauss_buffer();
        random_gauss_index = _xorshift();
    }
};

struct IKBase : Random
{
    IKParams params;
    RobotFK model;
    RobotInfo modelInfo;
    int thread_index;
    Problem problem;
    std::vector<Frame> null_tip_frames;
    volatile int canceled;

    virtual void step() = 0;

    virtual const std::vector<double>& getSolution() const = 0;

    virtual void setParams(const IKParams& p) {}

    IKBase(const IKParams& p)
        : model(p.robot_model)
        , modelInfo(p.robot_model)
        , params(p)
    {
        setParams(p);
    }
    virtual ~IKBase() {}

    virtual void initialize(const Problem& problem)
    {
        this->problem = problem;
        model.initialize(problem.tip_link_indices);
        // active_variables = problem.active_variables;
        null_tip_frames.resize(problem.tip_link_indices.size());
    }

    double computeSecondaryFitnessActiveVariables(const double* active_variable_positions) { return problem.computeGoalFitness(problem.secondary_goals, null_tip_frames.data(), active_variable_positions); }

    double computeSecondaryFitnessAllVariables(const std::vector<double>& variable_positions) { return computeSecondaryFitnessActiveVariables(extractActiveVariables(variable_positions)); }

    double computeFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions) { return problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions); }

    double computeCombinedFitnessActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions)
    {
        double ret = 0.0;
        ret += problem.computeGoalFitness(problem.goals, tip_frames.data(), active_variable_positions);
        ret += problem.computeGoalFitness(problem.secondary_goals, null_tip_frames.data(), active_variable_positions);
        return ret;
    }

    bool checkSolutionActiveVariables(const std::vector<Frame>& tip_frames, const double* active_variable_positions) { return problem.checkSolutionActiveVariables(tip_frames, active_variable_positions); }

    bool checkSolution(const std::vector<double>& variable_positions, const std::vector<Frame>& tips) { return checkSolutionActiveVariables(tips, extractActiveVariables(variable_positions)); }

    std::vector<double> temp_active_variable_positions;

    double* extractActiveVariables(const std::vector<double>& variable_positions)
    {
        temp_active_variable_positions.resize(problem.active_variables.size());
        for(size_t i = 0; i < temp_active_variable_positions.size(); i++)
            temp_active_variable_positions[i] = variable_positions[problem.active_variables[i]];
        return temp_active_variable_positions.data();
    }

    double computeFitness(const std::vector<double>& variable_positions, const std::vector<Frame>& tip_frames) { return computeFitnessActiveVariables(tip_frames, extractActiveVariables(variable_positions)); }

    double computeFitness(const std::vector<double>& variable_positions)
    {
        model.applyConfiguration(variable_positions);
        return computeFitness(variable_positions, model.getTipFrames());
    }

    virtual size_t concurrency() const { return 1; }
};

typedef IKBase IKSolver;

typedef Factory<IKSolver, const IKParams&> IKFactory;
}
