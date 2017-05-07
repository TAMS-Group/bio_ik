// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#include "ik_base.h"


#include <boost/thread/barrier.hpp>

namespace bio_ik
{



// executes a function in parallel on pre-allocated threads
class ParallelExecutor
{
    std::function<void(size_t)> fun;
    std::vector<std::thread> threads;
    boost::barrier barrier;
    volatile bool exit;
public:
    template<class FUN>
    ParallelExecutor(size_t thread_count, const FUN& f) : exit(false), threads(thread_count), fun(f), barrier(thread_count)
    {
        for(size_t i = 1; i < thread_count; i++)
        {
            std::thread t([this, i] ()
            {
                while(true)
                {
                    barrier.wait(); if(exit) break;
                    fun(i);
                    barrier.wait(); if(exit) break;
                }
            });
            std::swap(t, threads[i]);
        }
    }
    ~ParallelExecutor()
    {
        exit = true;
        barrier.wait();
        for(auto& t : threads) if(t.joinable()) t.join();
    }
    void run()
    {
        barrier.wait();
        fun(0);
        barrier.wait();
    }
};







// runs ik on multiple threads until a stop criterion is met
struct IKParallel
{
    IKParams params;
    std::vector<std::unique_ptr<IKSolver>> solvers;
    std::vector<std::vector<double>> solver_solutions;
    std::vector<std::vector<double>> solver_temps;
    std::vector<int> solver_success;
    std::vector<double> solver_fitness;
    int thread_count;
    //std::vector<RobotFK_Fast> fk; // TODO: remove
    double timeout;
    bool success;
    std::atomic<int> finished;
    std::vector<double> result;
    std::unique_ptr<ParallelExecutor> par;
    Problem problem;

    IKParallel(const IKParams& params) : params(params)
    {
        // solver class name
        std::string name;
        //params.node_handle.param("mode", name, std::string("bio1"));
        //params.node_handle.param("mode", name, std::string("bio2_memetic_l"));
        params.node_handle.param("mode", name, std::string("bio2_memetic"));
        //params.node_handle.param("mode", name, std::string("bio3_memetic"));
 
        // create solvers
        solvers.emplace_back(IKFactory::create(name, params));
        thread_count = solvers.front()->concurrency();
        params.node_handle.param("threads", thread_count, thread_count);
        while(solvers.size() < thread_count) solvers.emplace_back(IKFactory::clone(solvers.front().get()));
        for(size_t i = 0; i < thread_count; i++) solvers[i]->thread_index = i;

        //while(fk.size() < thread_count) fk.emplace_back(params.robot_model);

        // init buffers
        solver_solutions.resize(thread_count);
        solver_temps.resize(thread_count);
        solver_success.resize(thread_count);
        solver_fitness.resize(thread_count);

        // create parallel executor
        par.reset(new ParallelExecutor(thread_count, [this] (size_t i) { solverthread(i); }));
    }

    void initialize(const Problem& problem)
    {
        this->problem = problem;
        //for(auto& f : fk) f.initialize(problem.tip_link_indices);
    }

private:

    void solverthread(size_t i)
    {
        THREADPROFILER("thread", i);
        COUNTERPROFILER("solver threads");

        // initialize ik solvers
        {
            BLOCKPROFILER("ik solver init");
            solvers[i]->initialize(problem);
        }

        // run solver iterations until solution found or timeout
        for(size_t iteration = 0; (ros::WallTime::now().toSec() < timeout && finished == 0) || (iteration == 0 && i == 0); iteration++)
        {
            if(finished) break;

            // run solver for a few steps
            solvers[i]->step();
            for(int it2 = 1; it2 < 4; it2++)
                if(ros::WallTime::now().toSec() < timeout && finished == 0)
                    solvers[i]->step();

            if(finished) break;

            // get solution and check stop criterion
            auto& result = solver_temps[i];
            result = solvers[i]->getSolution();
            auto& fk = solvers[i]->model;
            fk.applyConfiguration(result);
            bool success = solvers[i]->checkSolution(result, fk.getTipFrames());
            if(success) finished = 1;
            solver_success[i] = success;
            solver_solutions[i] = result;
            solver_fitness[i] = solvers[i]->computeFitness(result, fk.getTipFrames());

            if(success) break;
        }

        finished = 1;

        for(auto& s : solvers) s->canceled = true;
    }

public:

    void solve()
    {
        BLOCKPROFILER("solve mt");

        // prepare
        result = problem.initial_guess;
        timeout = problem.timeout;
        success = false;
        finished = 0;
        for(auto& s : solver_solutions) s = problem.initial_guess;
        for(auto& s : solver_temps) s = problem.initial_guess;
        for(auto& s : solver_success) s = 0;
        for(auto& f : solver_fitness) f = DBL_MAX;
        for(auto& s : solvers) s->canceled = false;

        // run solvers
        {
            BLOCKPROFILER("solve mt 2");
            par->run();
        }

        size_t best_index = 0;
        double best_fitness = DBL_MAX;

        // if exact primary goal matches have been found ...
        for(size_t i = 0; i < thread_count; i++)
        {
            if(solver_success[i])
            {
                double fitness;
                if(solvers[0]->problem.secondary_goals.empty())
                {
                    // ... and if no secondary goals have been specified,
                    // select the best result according to primary goals
                    fitness = solver_fitness[i];
                }
                else
                {
                    // ... and if secondary goals have been specified,
                    // select the result that best satisfies primary and secondary goals
                    fitness = solver_fitness[i] + solvers[0]->computeSecondaryFitnessAllVariables(solver_solutions[i]);
                }
                if(fitness < best_fitness)
                {
                    best_fitness = fitness;
                    best_index = i;
                }
            }
        }

        // if no exact primary goal matches have been found,
        // select best primary goal approximation
        if(best_fitness == DBL_MAX)
        {
            for(size_t i = 0; i < thread_count; i++)
            {
                if(solver_fitness[i] < best_fitness)
                {
                    best_fitness = solver_fitness[i];
                    best_index = i;
                }
            }
        }

        result = solver_solutions[best_index];
        success = solver_success[best_index];
    }

    bool getSuccess() const { return success; }

    const std::vector<double>& getSolution() const { return result; }

};




}
