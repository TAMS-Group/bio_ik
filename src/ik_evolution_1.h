// Bio IK for ROS
// Philipp Ruppel

#pragma once

#include "ik_base.h"

namespace bio_ik
{

struct IKEvolution : IKBase
{
    struct Individual
    {
        std::vector<double> genes; 
        std::vector<double> gradients;
        double extinction;
        double fitness;
    };
    
    std::vector<double> solution;
    std::vector<Individual> population;
    int populationSize, eliteCount;
    std::vector<Individual*> tempPool;
    std::vector<Individual> tempOffspring;
    std::vector<double> initialGuess;
    
    bool opt_no_wipeout;

    void setParams(const IKParams& p)
    {
        auto& n = p.node_handle;
        n.param("no_wipeout", opt_no_wipeout, false);
        n.param("population_size", populationSize, 8);
        n.param("elite_count", eliteCount, 4);
    }
    
    bool in_final_adjustment_loop;
    
    template<class t>
    inline t select(const std::vector<t>& v)
    {
        //FNPROFILER();
        linear_int_distribution<size_t> d(v.size());
        size_t index = d(rng);
        return v[index];
    }
    
    inline double clip(double v, size_t i)
    {
        return modelInfo.clip(v, i);
    }
    
    inline double getMutationStrength(size_t i, const Individual& parentA, const Individual& parentB)
    {
        double extinction = 0.5 * (parentA.extinction + parentB.extinction);
        double span = modelInfo.getSpan(i);
        return span * extinction;
    }
    
    double computeAngularScale(size_t tip_index, const Frame& tip_frame)
    {
        //return 1;
        double angular_scale = sqrt(heuristicErrorTree.getChainLength(tip_index) * tip_frame.pos.length()) / M_PI;
        //double angular_scale = heuristicErrorTree.getChainLength(tip_index) * (1.0 / M_PI);
        if(opt_angular_scale_full_circle) angular_scale *= 0.5;
        return angular_scale;
    }
    
    double computeTipFitness(size_t tip_index, const Frame& tip_frame, bool balanced, bool for_heuristic_error)
    {
        // TODO: other goal types
            
        const auto& ta = tipObjectives[tip_index];
        const auto& tb = tip_frame;
        
        double tdist, rdist;

        //if(for_heuristic_error || !opt_quadratic_error || in_final_adjustment_loop)
        if(for_heuristic_error)
        {
            tdist = ta.pos.distance(tb.pos) / computeAngularScale(tip_index, tip_frame);
            rdist = ta.rot.angle(tb.rot); 
        } 
        else 
        {
            double angularScale = computeAngularScale(tip_index, tip_frame);
            
            
            //double angularScale = computeAngularScale(tip_index);
            
            // TODO: optimize div
            
            tdist = ta.pos.distance2(tb.pos) / (angularScale * angularScale); // square distance
            
            //rdist = (1.0 - ta.rot.dot(tb.rot)) * 2.0; // small-angle approximation for square angle
            
            rdist = (ta.rot - tb.rot).length2();
            
            // TODO: rotation dist correct ????
            
            /*double dot = ta.rot.dot(tb.rot);
            if(dot < 0)
                */
            
        }
        
        if(params.tip_infos[tip_index].position_only_ik) rdist = 0;
        
        //if(balanced) return mix(tdist, rdist, 0.5);

        return mix(tdist, rdist, balanced ? 0.5 : mix(0.5, random(), fitness_randomization));
        //return mix(tdist, rdist, (balanced || in_final_adjustment_loop) ? 0.5 : random(0.2, 0.8));
    }
    
    
    /*double getHeuristicError(size_t variable_index, bool balanced)
    {
        //return modelInfo.getSpan(variable_index) * random(-1, 1) * (1 << std::uniform_int_distribution<int>(0, 4)(rng)) * (1.0 / (1 << 4));
        //return modelInfo.getSpan(variable_index) * (1 << std::uniform_int_distribution<int>(0, 20)(rng)) * (1.0 / (1 << 20));
                
        //return modelInfo.getSpan(variable_index) * random(-1, 1) * random() * random() * random();
        
        //if(in_final_adjustment_loop) if(random() < 0.5) return random();
        
        double heuristic_error = 0;
        for(size_t tip_index = 0; tip_index < tipObjectives.size(); tip_index++)
           heuristic_error += heuristicErrorTree.getInfluence(variable_index, tip_index) * computeTipFitness(tip_index, model.getTipFrame(tip_index), balanced, true); 
        
        //if(in_final_adjustment_loop) heuristic_error *= random() * random() * 5;
        //if(in_final_adjustment_loop) heuristic_error *= 2;
        //heuristic_error *= 2;
        //heuristic_error *= random() * random() * 5;
        //heuristic_error *= random(0, 2);
        
        return heuristic_error;
    }*/
    
    
    double getHeuristicError(size_t variable_index, bool balanced)
    {
        double heuristic_error = 0;
        for(int tip_index = 0; tip_index < tipObjectives.size(); tip_index++)
        {
            double influence = heuristicErrorTree.getInfluence(variable_index, tip_index);
            if(influence == 0) continue;
            
            const auto& ta = tipObjectives[tip_index];
            const auto& tb = model.getTipFrame(tip_index);
            
            double length = heuristicErrorTree.getJointVariableChainLength(tip_index, variable_index);
            
            //LOG_ALWAYS("a", heuristicErrorTree.getJointVariableChainLength(tip_index, variable_index));
            
            //double length = model.getJointVariableFrame(variable_index).pos.distance(model.getTipFrame(tip_index).pos); if(length <= 0.000000001) length = 0;
            
            //LOG_ALWAYS("b", length);
            
            if(modelInfo.isPrismatic(variable_index))
            {
                //heuristic_error += ta.pos.distance(tb.pos) * influence;
                //if(length) heuristic_error += ta.rot.angle(tb.rot) * length * influence;
                
                if(length)
                {
                    heuristic_error += ta.pos.distance(tb.pos) * influence * 0.5;
                    heuristic_error += ta.rot.angle(tb.rot) * length * influence * 0.5;
                }
                else
                {
                    heuristic_error += ta.pos.distance(tb.pos) * influence;
                }
            }
            
            if(modelInfo.isRevolute(variable_index))
            {
                //if(length) heuristic_error += ta.pos.distance(tb.pos) / length * influence;
                //heuristic_error += ta.rot.angle(tb.rot) * influence;
                
                if(length)
                {
                    heuristic_error += ta.pos.distance(tb.pos) / length * influence * 0.5;
                    heuristic_error += ta.rot.angle(tb.rot) * influence * 0.5;
                }
                else
                {
                    heuristic_error += ta.rot.angle(tb.rot) * influence;
                }
                
                //double d = 0.0;
                //if(length) d = std::max(d, ta.pos.distance(tb.pos) / length);
                //d = std::max(d, ta.rot.angle(tb.rot));
                //heuristic_error += d * influence;
            }
        }
        //heuristic_error *= 0.5;
        //LOG_ALWAYS(heuristic_error);
        return heuristic_error;
    }
    
    bool in_adjustment_2, in_get_solution_fitness;
    
    void reroll(Individual& offspring)
    {
        FNPROFILER();
        //for(size_t i = 0; i < offspring.genes.size(); i++)
        for(auto i : getGenes())
        {
            offspring.genes[i] = random(modelInfo.getMin(i), modelInfo.getMax(i));
            
            offspring.genes[i] = mix(offspring.genes[i], (modelInfo.getMin(i) + modelInfo.getMax(i)) * 0.5, random(0.0, 0.1));
            
            offspring.gradients[i] = 0;
        }
        offspring.fitness = computeFitness(offspring.genes, false);
    }
    
    bool checkWipeout()
    {
        FNPROFILER();
        auto& genes = population[0].genes;
        //for(size_t i = 0; i < genes.size(); i++)
        for(auto i : getGenes())
        {
            double v0 = genes[i];
            double fitness = computeFitness(genes, true);
            double heuristicError = getHeuristicError(i, true);
            //double heuristicError = 0.001;
            genes[i] = modelInfo.clip(v0 + random(0, heuristicError), i);
            double incFitness = computeFitness(genes, true);
            genes[i] = modelInfo.clip(v0 - random(0, heuristicError), i);
            double decFitness = computeFitness(genes, true);
            genes[i] = v0;
            if(incFitness < fitness || decFitness < fitness) 
            {
                //LOG("no wipeout");
                return false;
            }
        }
        //LOG("wipeout 1");
        return true;
    }
    
    void computeExtinctions()
    {
        double min = population.front().fitness;
        double max = population.back().fitness;
        for(size_t i = 0; i < populationSize; i++)
        {
            double grading = (double)i / (double)(populationSize - 1);
            population[i].extinction = (population[i].fitness + min * (grading - 1)) / max;
        }
    }
    
    bool tryUpdateSolution()
    {
        FNPROFILER();
        double solutionFitness = computeFitness(solution, true);
        double candidateFitness = computeFitness(population[0].genes, true);
        //LOG_VAR(solutionFitness);
        //LOG_VAR(candidateFitness);
        if(candidateFitness < solutionFitness)
        {
            solution = population[0].genes;
            //solution = initialGuess;
            //for(auto i : getGenes())
            //    solution[i] = population[0].genes[i];
            return true;
        }
        return false;
    }
    
    double getMutationProbability(const Individual& parentA, const Individual& parentB) 
    {
        double extinction = 0.5 * (parentA.extinction + parentB.extinction);
        double inverse = 1.0 / parentA.genes.size();
        return extinction * (1.0 - inverse) + inverse;
    }
    
    void sortByFitness()
    {
        FNPROFILER();
        sort(population.begin(), population.end(), [] (const Individual& a, const Individual& b) 
        {
            return a.fitness < b.fitness;
        });
    }
    
    double bounce(double v, int i)
    {
        double c = clip(c, i);
        v = c - (v - c) * 2;
        //v = c + c - v;
        v = clip(v, i);
        return v;
    }
    
    void reproduce(Individual& offspring, const Individual& parentA, const Individual& parentB, const Individual& prototype)
    {
        FNPROFILER();
        for(size_t i = 0; i < offspring.genes.size(); i++)
        //for(auto i : getGenes())
        {
            offspring.genes[i] = mix(parentA.genes[i], parentB.genes[i], random());
            offspring.genes[i] += parentA.gradients[i] * random();
            offspring.genes[i] += parentB.gradients[i] * random();
            
            double storage = offspring.genes[i];
            
            if(random() < getMutationProbability(parentA, parentB))
                offspring.genes[i] += random(-1, 1) * getMutationStrength(i, parentA, parentB);
                //offspring.genes[i] += normal_random() * getMutationStrength(i, parentA, parentB);
                
            offspring.genes[i] += mix(
                random() * (0.5 * (parentA.genes[i] + parentB.genes[i]) - offspring.genes[i]),
                random() * (prototype.genes[i] - offspring.genes[i]),
                random());
                
            //offspring.genes[i] = clip(offspring.genes[i], i);
            
            //offspring.genes[i] += fabs(offspring.genes[i] - storage) * offspring.genes[i] - (modelInfo.getMin(i) + modelInfo.getMax(i)) * 0.5;

            //offspring.genes[i] = mix(offspring.genes[i], (modelInfo.getMin(i) + modelInfo.getMax(i)) * 0.5, random() * 0.1 * fabs(offspring.genes[i] - storage) / modelInfo.getSpan(i));
            
            //offspring.genes[i] = clip(offspring.genes[i], i);
            
            offspring.genes[i] = bounce(offspring.genes[i], i);
            
            offspring.gradients[i] = random() * offspring.gradients[i] + offspring.genes[i] - storage;
        }
        
        offspring.fitness = computeFitness(offspring.genes, false);
    }
    
    void exploit(Individual& individual)
    {
        FNPROFILER();
        
        double fitness_sum = 0;
        model.incrementalBegin(individual.genes);

        for(auto i : getGenes())
        {
            double fitness = computeFitness(individual.genes, true);

            double heuristicError = getHeuristicError(i, true);
            double v_0 = individual.genes[i];

            double v_inc = bounce(v_0 + random(0, heuristicError), i);
            double v_dec = bounce(v_0 - random(0, heuristicError), i);

            individual.genes[i] = v_inc;
            double inc_fitness = computeFitness(individual.genes, true);
            individual.genes[i] = v_dec;
            double dec_fitness = computeFitness(individual.genes, true);
            if(inc_fitness < fitness && inc_fitness <= dec_fitness)
            {
                individual.genes[i] = v_inc;
                individual.gradients[i] = v_0 * random() + v_inc - v_0;
                fitness_sum += inc_fitness;
            } else 
            if(dec_fitness < fitness && dec_fitness <= inc_fitness) 
            {
                individual.genes[i] = v_dec;
                individual.gradients[i] = v_0 * random() + v_dec - v_0;
                fitness_sum += dec_fitness;
            } else
            {
                individual.genes[i] = v_0;
                fitness_sum += fitness;
            }
        }
        
        model.incrementalEnd();
        
        individual.fitness = fitness_sum / individual.genes.size();
    }

    IKEvolution(const IKParams& p) : IKBase(p),
        populationSize(12), 
        eliteCount(4), 
        in_final_adjustment_loop(false),
        in_get_solution_fitness(false),
        in_adjustment_2(false)
    {
        setParams(p);
    }
    
    void initialize(const std::vector<double>& initialGuess, const std::vector<Frame>& tipObjectives)
    {
        IKBase::initialize(initialGuess, tipObjectives);
        
        {
            this->initialGuess = initialGuess;
            solution = initialGuess;

            population.resize(populationSize);
            
            {
                auto& p = population[0];
                p.genes = solution;
                p.gradients.clear();
                p.gradients.resize(p.genes.size(), 0);
                p.fitness = computeFitness(p.genes, false);
            }
            
            for(int i = 1; i < populationSize; i++)
            {
                auto& p = population[i];
                p.genes = solution;
                p.gradients.clear();
                p.gradients.resize(p.genes.size(), 0);
                reroll(p);
            }
                
            sortByFitness();
            computeExtinctions();
        }
    }
    
    const std::vector<double>& getSolution() const
    {
        return solution;
    }

    double getSolutionFitness()
    {
        in_get_solution_fitness = true;
        double f = computeFitness(solution, true);
        in_get_solution_fitness = false;
        return f;
    }
    
    const std::vector<Frame>& getSolutionTipFrames()
    {
        model.applyConfiguration(solution);
        return model.getTipFrames();
    }
    
    bool evolve()
    {
        FNPROFILER();
        
        auto& offspring = tempOffspring;
        offspring = population;
    
        for(size_t i = 0; i < eliteCount; i++)
        {
            offspring[i] = population[i];
            exploit(offspring[i]);
        }
        
        auto& pool = tempPool;
        pool.resize(populationSize);
        iota(pool.begin(), pool.end(), &population[0]);
        
        for(size_t i = eliteCount; i < populationSize; i++)
        {
            if(pool.size() > 0)
            {
                auto& parentA = *select(pool);
                auto& parentB = *select(pool);
                auto& prototype = *select(pool);
                reproduce(offspring[i], parentA, parentB, prototype);
                if(offspring[i].fitness < parentA.fitness) pool.erase(remove(pool.begin(), pool.end(), &parentA), pool.end());
                if(offspring[i].fitness < parentB.fitness) pool.erase(remove(pool.begin(), pool.end(), &parentB), pool.end());
            } 
            else 
            {
                reroll(offspring[i]);
            }
        }
        
        population = offspring;
        
        sortByFitness();
        
        computeExtinctions();

        if(tryUpdateSolution()) return true;
        if(opt_no_wipeout) return false;
        if(!checkWipeout()) return false;
        
        initialize(initialGuess, tipObjectives);

        return tryUpdateSolution();
    }

    void step()
    {
        in_adjustment_2 = false;
        evolve();
    }
    
    virtual size_t concurrency() const { return 4; }
};

static IKFactory::Class<IKEvolution> cIKEvolution("bio1");

}
