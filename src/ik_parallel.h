// Bio IK for ROS
// Philipp Ruppel

#include "ik_base.h"


#include <boost/thread/barrier.hpp>

namespace bio_ik
{



// executes a function in parallel on pre-allocated threads
class ParallelExecutor
{
    std::function<void(size_t)> fun;
    size_t thread_count;
    std::vector<std::thread> threads;
    boost::barrier barrier;
    volatile bool exit;
public:
    template<class FUN>
    ParallelExecutor(size_t thread_count, const FUN& f) : exit(false), thread_count(thread_count), threads(thread_count), fun(f), barrier(thread_count)
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
    //std::vector<Frame> tip_goals;
    //std::vector<RobotFK_MoveIt> fk;
    std::vector<RobotFK_Fast> fk; // TODO: remove
    double timeout;
    bool success;
    std::atomic<int> finished;
    std::vector<double> result;
    std::vector<std::thread> threads;
    std::unique_ptr<ParallelExecutor> par;
    double dpos, drot, dtwist;
    IKRequest request;
    
    IKParallel(const IKParams& params) : params(params)
    {
        std::string name;
        params.node_handle.param("mode", name, std::string("bio2_memetic"));
        
        solvers.emplace_back(IKFactory::create(name, params));
        
        thread_count = solvers.front()->concurrency();
        
        //thread_count = 1;
        
        params.node_handle.param("threads", thread_count, thread_count);
        
        /*params.node_handle.param("dpos", dpos, 0.0001);
        params.node_handle.param("drot", drot, 0.5);
        params.node_handle.param("dtwist", dtwist, DBL_MAX);*/
        
        params.node_handle.param("dpos", dpos, DBL_MAX);
        params.node_handle.param("drot", drot, DBL_MAX);
        params.node_handle.param("dtwist", dtwist, 1e-5);

        while(solvers.size() < thread_count) solvers.emplace_back(IKFactory::clone(solvers.front().get()));
        
        for(size_t i = 0; i < thread_count; i++)
            solvers[i]->thread_index = i;
        
        while(fk.size() < thread_count) fk.emplace_back(params.robot_model);
        
        solver_solutions.resize(thread_count);
        solver_temps.resize(thread_count);
        solver_success.resize(thread_count);
        solver_fitness.resize(thread_count);
        
        threads.resize(thread_count);
        
        par.reset(new ParallelExecutor(thread_count, [this] (size_t i) { solverthread(i); }));
    }
    
    void initialize(const IKRequest& request)
    {
        this->request = request;
        for(auto& f : fk) f.initialize(request.tip_link_indices);
    }
    
private:

    /*bool checkSolution(const std::vector<Frame>& tips) const
    {
        // TODO: use new goals
    
        //if(request.secondary_goals.size()) return false;
    
        for(auto& goal : request.goals)
            if(goal.goal_type != GoalType::Pose)
                return false;
    
        if(dpos != DBL_MAX || drot != DBL_MAX)
        {
            double p_dist = 0;
            double r_dist = 0;
            for(auto& goal : request.goals)
            {
                auto& a = tips[goal.tip_index];
                auto& b = goal.frame;
                p_dist = std::max(p_dist, (b.pos - a.pos).length());
                r_dist = std::max(r_dist, b.rot.angleShortestPath(a.rot));
            }
            r_dist = r_dist * 180 / M_PI;
            if(!(p_dist <= dpos)) return false;
            if(!(r_dist <= drot)) return false;
        }
        
        if(dtwist != DBL_MAX)
        {
            for(auto& goal : request.goals)
            {
                KDL::Frame fk_kdl, ik_kdl;
                frameToKDL(goal.frame, fk_kdl);
                frameToKDL(tips[goal.tip_index], ik_kdl);
                KDL::Twist kdl_diff(fk_kdl.M.Inverse() * KDL::diff(fk_kdl.p, ik_kdl.p), fk_kdl.M.Inverse() * KDL::diff(fk_kdl.M, ik_kdl.M));
                if(!KDL::Equal(kdl_diff, KDL::Twist::Zero(), dtwist)) return false;
            }
        }
        
        return true;
    }*/
    
    void solverthread(size_t i)
    {
        //BLOCKPROFILER("solverthread");
        
        THREADPROFILER("thread", i);
        
        COUNTERPROFILER("solver threads");
    
        //if(ros::Time::now().toSec() - timeout > 0.01) ERROR("start overtime", ros::Time::now().toSec() - timeout);
        //if((ros::Time::now().toSec() < timeout && finished == 0) || i == 0)
        {
            {
                BLOCKPROFILER("ik solver init");
                solvers[i]->initialize(request);
            }
            for(size_t iteration = 0; (ros::Time::now().toSec() < timeout && finished == 0) || (iteration == 0 && i == 0); iteration++)
            {
                //if(finished || ros::Time::now().toSec() > timeout) break;
                if(finished) break;
            
                solvers[i]->step();
                
                for(int it2 = 1; it2 < 4; it2++)
                    if(ros::Time::now().toSec() < timeout && finished == 0) 
                        solvers[i]->step();
                        
                //if(finished || ros::Time::now().toSec() > timeout) break;
                if(finished) break;
                        
                auto& result = solver_temps[i];
                result = solvers[i]->getSolution();
                //params.robot_model->enforcePositionBounds(result.data());
                fk[i].applyConfiguration(result);
                
                //bool success = checkSolution(fk[i].getTipFrames());
                bool success = solvers[i]->checkSolution(result, fk[i].getTipFrames());
                if(success) finished = 1;
                solver_success[i] = success;
                solver_solutions[i] = result;
                solver_fitness[i] = solvers[i]->computeFitness(result, fk[i].getTipFrames());
                if(success) break;
            }
        }
        //if(ros::Time::now().toSec() - timeout > 0.01) ERROR("overtime", ros::Time::now().toSec() - timeout);
        
        finished = 1;
        for(auto& s : solvers) s->canceled = true;
    }
    
public:

    void solve()
    {
        BLOCKPROFILER("solve mt");
        
        auto& initial_guess = request.initial_guess;
        //auto& tip_goals = request.tip_objectives;
        auto timeout = request.timeout;
        
        //for(size_t i = 1; i < thread_count; i++) if(threads[i].joinable()) threads[i].join();
        
        result = initial_guess;
        success = false;
        finished = 0;

        for(auto& s : solver_solutions) s = initial_guess;
        for(auto& s : solver_temps) s = initial_guess;
        for(auto& s : solver_success) s = 0;
        for(auto& f : solver_fitness) f = DBL_MAX;
        
        for(auto& s : solvers) s->canceled = false;
        
        //this->tip_goals = tip_goals;
        this->timeout = timeout;
        
        {
        
            BLOCKPROFILER("solve mt 2");
        
        
            
            /*#pragma omp parallel num_threads(thread_count)
            {
                int i = omp_get_thread_num();
                solverthread(i);
            }*/
            
            
            
            
            /*#pragma omp parallel for num_threads(thread_count)
            for(size_t i = 0; i < thread_count; i++)
            {
                solverthread(i);
            }*/
            
            
            
            /*
            for(size_t i = 1; i < thread_count; i++)
            {
                BLOCKPROFILER("thread start");
                std::thread t([this, i] () { solverthread(i); });
                std::swap(threads[i], t);
            }
            solverthread(0);
            for(size_t i = 1; i < thread_count; i++) threads[i].join();
            */
            
            
            
            /*
            std::vector<pthread_t> threads(thread_count);
            std::vector<std::pair<IKConcurrent*, size_t>> args(thread_count);
            for(size_t i = 1; i < thread_count; i++)
            {
                args[i].first = this;
                args[i].second = i;
                int r = pthread_create(&(threads[i]), 0, [] (void* arg)
                {
                    auto p = (std::pair<IKConcurrent*, size_t>*)arg;
                    auto _this = p->first;
                    auto _i = p->second;
                    _this->solverthread(_i);
                    return (void*)0;
                }, &(args[i]));
                if(r) ERROR("failed to start thread");
            }
            solverthread(0);
            for(size_t i = 1; i < thread_count; i++)
                pthread_join(threads[i], 0);
            */
            
            
            
            par->run();
            
            
            
            /*using boost::experimental::parallel::task_region;
            using boost::experimental::parallel::task_region_handle;
            task_region([&](task_region_handle& trh)
            {
                for(size_t i = 1; i < thread_count; i++)
                    trh.run([this, i] { solverthread(i); });
                solverthread(i);
            });*/
            
            
        }
            
        /*result = solver_solutions[0];
        for(size_t i = 0; i < thread_count; i++)
        {
            if(solver_success[i])
            {
                result = solver_solutions[i];
                success = true;
                break;
            }
        }*/
        
        size_t best_index = 0;
        double best_fitness = DBL_MAX;
        
        // if exact primary goal matches have been found ...
        for(size_t i = 0; i < thread_count; i++)
        {
            if(solver_success[i])
            {
                double fitness;
                if(solvers[0]->request.secondary_goals.empty())
                {
                    // ... and no secondary goals have been specified,
                    // select the result best result according to primary goals
                    fitness = solver_fitness[i];
                }
                else
                {
                    // ... and secondary goals have been specified,
                    // select the result that best satisfies the secondary goals
                    fitness = solvers[0]->computeSecondaryFitnessAllVariables(solver_solutions[i]);
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
                    
        /*LOG_VAR(initial_guess.size());
        LOG_VAR(result.size());
        LOG_VAR(best_index);*/
                    
        result = solver_solutions[best_index];
        success = solver_success[best_index];
        
        //LOG_VAR(result.size());
    }
    
    bool getSuccess() const { return success; }
    
    const std::vector<double>& getSolution() const { return result; }
    
};




}




