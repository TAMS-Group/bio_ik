// Bio IK for ROS
// Philipp Ruppel

#pragma once

#include "ik_base.h"

#include "ik_evolution_1.h"

namespace bio_ik
{

// pseudoinverse jacobian solver
// (mainly for testing RobotFK_Jacobian::computeJacobian)
template<class BASE>
struct IKJacobianBase : BASE
{
    using BASE::modelInfo;
    using BASE::model;
    using BASE::params;
    using BASE::tipObjectives;
    using BASE::computeFitness;
    using BASE::getGenes;
    using BASE::active_variables;
    
    Eigen::VectorXd tip_diffs;
    Eigen::VectorXd joint_diffs;
    Eigen::MatrixXd jacobian;
    std::vector<Frame> tip_frames_temp;

    IKJacobianBase(const IKParams& p) : BASE(p)
    {
    }
    
    void optimizeJacobian(std::vector<double>& solution)
    {
        FNPROFILER();

        int tip_count = params.tip_frames.size();
        tip_diffs.resize(tip_count * 6);
        joint_diffs.resize(getGenes().size());

        // compute fk
        model.applyConfiguration(solution);
        
        double translational_scale = 1;
        double rotational_scale = 1;
        
        // compute goal diffs
        tip_frames_temp = model.getTipFrames();
        for(int itip = 0; itip < tip_count; itip++)
        {
            auto twist = frameTwist(tip_frames_temp[itip], tipObjectives[itip]);
            tip_diffs(itip * 6 + 0) = twist.vel.x() * translational_scale;
            tip_diffs(itip * 6 + 1) = twist.vel.y() * translational_scale;
            tip_diffs(itip * 6 + 2) = twist.vel.z() * translational_scale;
            tip_diffs(itip * 6 + 3) = twist.rot.x() * rotational_scale;
            tip_diffs(itip * 6 + 4) = twist.rot.y() * rotational_scale;
            tip_diffs(itip * 6 + 5) = twist.rot.z() * rotational_scale;
        }
        
        // compute jacobian
        {
            model.computeJacobian(active_variables, jacobian);
            int icol = 0;
            for(auto ivar : getGenes())
            {
                for(size_t itip = 0; itip < tip_count; itip++)
                {
                    jacobian(itip * 6 + 0, icol) *= translational_scale;
                    jacobian(itip * 6 + 1, icol) *= translational_scale;
                    jacobian(itip * 6 + 2, icol) *= translational_scale;
                    jacobian(itip * 6 + 3, icol) *= rotational_scale;
                    jacobian(itip * 6 + 4, icol) *= rotational_scale;
                    jacobian(itip * 6 + 5, icol) *= rotational_scale;
                }
                icol++;
            }
        }

        // get pseudoinverse and multiply
        joint_diffs = jacobian.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(tip_diffs);
        //joint_diffs = (jacobian.transpose() * jacobian).ldlt().solve(jacobian.transpose() * tip_diffs);

        // apply joint deltas and clip
        {
            int icol = 0;
            for(auto ivar : active_variables)
            {
                auto v = solution[ivar] + joint_diffs(icol);
                if(!std::isfinite(v)) continue;
                v = modelInfo.clip(v, ivar);
                solution[ivar] = v;
                icol++;
            }
        }
    }
};



// only pseudoinverse jacobian
struct IKJacobian : IKJacobianBase<IKBase>
{
    using IKBase::initialize;
    std::vector<double> solution;
    IKJacobian(const IKParams& p) : IKJacobianBase<IKBase>(p)
    {
    }
    void initialize(const std::vector<double>& initial_guess, const std::vector<Frame>& tipObjectives)
    {
        IKJacobianBase<IKBase>::initialize(initial_guess, tipObjectives);
        solution = initial_guess;
        if(thread_index > 0)
            for(auto& vi : active_variables)
                solution[vi] = random(modelInfo.getMin(vi), modelInfo.getMax(vi));
    }
    const std::vector<double>& getSolution() const
    {
        return solution;
    }
    void step()
    {
        optimizeJacobian(solution);
    }
    size_t concurrency() const { return 1; }
};
static IKFactory::Class<IKJacobian> cIKJacobian("jac");



// combining evolution and pseudoinverse jacobian
template<class BASE>
struct IKHybrid : IKJacobianBase<BASE>
{
    using BASE::computeFitness;
    using IKJacobianBase<BASE>::optimizeJacobian;
    std::vector<double> best_solution, optimized_solution;
    double best_fitness, optimized_fitness;
    IKHybrid(const IKParams& p) : IKJacobianBase<BASE>(p)
    {
    }
    void initialize(const std::vector<double>& initial_guess, const std::vector<Frame>& tip_objectives)
    {
        BASE::initialize(initial_guess, tip_objectives);
        best_solution = initial_guess;
        best_fitness = computeFitness(best_solution);
        optimized_solution = best_solution;
        optimized_fitness = best_fitness;
    }
    const std::vector<double>& getSolution()
    {
        return best_solution;
    }
    void step()
    {
        FNPROFILER();
        BASE::step();
        {
            auto& current_solution = BASE::getSolution();
            double current_fitness = computeFitness(current_solution);
            if(current_fitness < best_fitness) best_fitness = current_fitness, best_solution = current_solution;   
            if(current_fitness < optimized_fitness) optimized_fitness = current_fitness, optimized_solution = current_solution;   
        }
        {
            optimizeJacobian(optimized_solution);
            double optimized_fitness = computeFitness(optimized_solution);
            if(optimized_fitness < best_fitness) best_fitness = optimized_fitness, best_solution = optimized_solution;
        }
    }
    size_t concurrency() const { return 4; }
};
static IKFactory::Class<IKHybrid<IKEvolution>> bio1jac("bio1jac");
static IKFactory::Class<IKHybrid<IKEvolution2>> bio2jac("bio2jac");




}









