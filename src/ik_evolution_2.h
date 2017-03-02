// Bio IK for ROS
// Philipp Ruppel

#pragma once

#include "ik_base.h"

namespace bio_ik
{

// fast evolutionary inverse kinematics
struct IKEvolution2 : IKBase
{
    struct Individual
    {
        std::vector<double> genes;
        std::vector<double> gradients;
        double fitness;
    };

    struct Species
    {
        std::vector<Individual> individuals;
        double fitness;
        bool improved;
    };
    
    std::vector<double> initial_guess;
    std::vector<double> solution;
    std::vector<double> temp_joint_variables;
    double solution_fitness;
    std::vector<Species> species;
    std::vector<Individual> children;
    std::vector<std::vector<Frame>> phenotypes;
    std::vector<size_t> child_indices;
    std::vector<double*> genotypes;
    std::vector<Frame> phenotype;
    std::vector<size_t> gene_resets;
    
    struct GeneInfo
    {
        double clip_min, clip_max, span;
    };
    std::vector<GeneInfo> gene_infos;
    
    IKEvolution2(const IKParams& p) : IKBase(p)
    {
        if(active_variables.size())
        {
            // init gene reset buffer
            // for preferring solutions close to initial guess
            // weighted by inverse max joint velocity
            std::vector<double> mutation_cost_sat;
            for(size_t gi = 0; gi < active_variables.size(); gi++)
            {
                auto& var_bounds = params.robot_model->getVariableBounds(params.robot_model->getVariableNames()[active_variables[gi]]);
                double c = var_bounds.max_velocity_ ? 1.0 / var_bounds.max_velocity_ : 0.0;
                if(mutation_cost_sat.empty())
                    mutation_cost_sat.push_back(c);
                else
                    mutation_cost_sat.push_back(mutation_cost_sat.back() + c);
            }
            if(mutation_cost_sat.back() == 0)
                for(size_t gi = 0; gi < active_variables.size(); gi++)
                    mutation_cost_sat[gi] = gi;
            for(size_t iter = 0; iter < 1024; iter++)
            {
                double f = random() * mutation_cost_sat.back();
                size_t i = 0;
                while(mutation_cost_sat[i] < f) i++;
                if(i >= mutation_cost_sat.size()) i = mutation_cost_sat.size() - 1;
                gene_resets.push_back(i);
            }
        }
    }
    
    void genesToJointVariables(const std::vector<double>& genes, std::vector<double>& variables)
    {
        for(size_t i = 0; i < active_variables.size(); i++)
            variables[active_variables[i]] = genes[i];
    }
    
    const std::vector<double>& getSolution() const
    {
        return solution;
    }

    void initialize(const std::vector<double>& initial_guess, const std::vector<Frame>& tipObjectives)
    {
        BLOCKPROFILER("initialization");
    
        IKBase::initialize(initial_guess, tipObjectives);
        
        // set solution to initial guess
        this->initial_guess = initial_guess;
        solution = initial_guess;
        solution_fitness = computeFitness(solution);
        
        // init temporary buffer with positions of inactive joints
        temp_joint_variables = initial_guess;
        
        // params
        size_t population_size = 2;
        size_t child_count = 16;
        
        // initialize population on current island
        species.resize(2);
        for(auto& s : species)
        {
            // create individuals
            s.individuals.resize(population_size);
            
            // initialize first individual
            {
                auto& v = s.individuals[0];
                
                // init genes
                v.genes.resize(active_variables.size());
                //if(thread_index == 0) // on first island?
                //if(thread_index % 2 == 0) // on every second island...
                //if(1)
                {
                    // set population to initial_guess
                    for(size_t i = 0; i < v.genes.size(); i++)
                        v.genes[i] = initial_guess[active_variables[i]];
                } 
                /*else 
                {
                    // initialize populations on other islands randomly
                    for(size_t i = 0; i < v.genes.size(); i++)
                        v.genes[i] = random(modelInfo.getMin(active_variables[i]), modelInfo.getMax(active_variables[i]));
                }*/
                
                // set gradients to zero
                v.gradients.clear();
                v.gradients.resize(active_variables.size(), 0);
            }
            
            // clone first individual
            for(size_t i = 1; i < s.individuals.size(); i++)
            {
                s.individuals[i].genes = s.individuals[0].genes;
                s.individuals[i].gradients = s.individuals[0].gradients;
            }
        }
        
        // space for child population
        children.resize(population_size + child_count);
        for(auto& child : children) 
        {
            child.genes.resize(active_variables.size());
            child.gradients.resize(active_variables.size());
        }
        
        // init gene infos
        if(gene_infos.empty())
        {
            gene_infos.resize(active_variables.size());
            for(size_t i = 0; i < active_variables.size(); i++)
            {
                gene_infos[i].clip_min = modelInfo.getClipMin(active_variables[i]);
                gene_infos[i].clip_max = modelInfo.getClipMax(active_variables[i]);
                gene_infos[i].span = modelInfo.getSpan(active_variables[i]);
            }
        }
    }
    
    // create offspring and mutate
    void reproduce(const std::vector<Individual>& population)
    {
        auto gene_count = children[0].genes.size();
        auto* rr = fast_random_gauss_n((children.size() - population.size()) * gene_count);
        for(size_t child_index = population.size(); child_index < children.size(); child_index++)
        {
            double mutation_rate = (1 << fast_random_index(16)) * (1.0 / (1 << 20));
            auto& parent = population[0];
            auto& parent2 = population[1];
            double fmix = (child_index % 2 == 0) * 0.2;
            for(size_t gene_index = 0; gene_index < gene_count; gene_index++)
            {
                auto& gene_info = gene_infos[gene_index];
                double r = (*(rr++));
                double f = mutation_rate * gene_info.span;
                double gene = parent.genes[gene_index];
                double parent_gene = gene;
                gene += r * f;
                double parent_gradient = mix(parent.gradients[gene_index], parent2.gradients[gene_index], fmix);
                double gradient = parent_gradient * (child_index % 3);
                gene += gradient;
                gene = clamp2(gene, gene_info.clip_min, gene_info.clip_max);
                children[child_index].genes[gene_index] = gene;
                children[child_index].gradients[gene_index] = mix(parent_gradient, gene - parent_gene, 0.1);
            }
            
            if(thread_index == 0 && fast_random_index(4) == 0)
            {
                auto gene_index = fast_random_element(gene_resets);
                auto& p1 = children[child_index].genes[gene_index];
                auto p2 = mix(p1, initial_guess[active_variables[0]], fast_random());
                auto& dp = children[child_index].gradients[gene_index];
                dp = mix(dp, p2 - p1, 0.1);
                p1 = p2;
            }
        }
    }

    void step()
    {
        FNPROFILER();

        for(size_t ispecies = 0; ispecies < species.size(); ispecies++)
        {
            auto& species = this->species[ispecies];
        
            BLOCKPROFILER("evolution");
            
            auto& population = species.individuals;
            
            // initialize forward kinematics approximator
            genesToJointVariables(species.individuals[0].genes, temp_joint_variables);
            model.applyConfiguration(temp_joint_variables);
            model.initializeMutationApproximator(active_variables);
            
            // run evolution for a few generations
            for(size_t generation = 0; generation < 16; generation++)
            //for(size_t generation = 0; generation < (ispecies == 0 ? 16 : 8); generation++)
            {
                
                // reproduction
                {
                    BLOCKPROFILER("reproduction");
                    reproduce(population);
                }
                
                // keep parents
                {
                    BLOCKPROFILER("keep alive");
                    for(size_t i = 0; i < population.size(); i++)
                    {
                        children[i].genes = population[i].genes;
                        children[i].gradients = population[i].gradients;
                    }
                }
                
                // genotype-phenotype mapping
                {
                    BLOCKPROFILER("phenotype");
                    size_t child_count = children.size();
                    size_t gene_count = children[0].genes.size();
                    genotypes.resize(child_count);
                    for(size_t i = 0; i < child_count; i++)
                        genotypes[i] = children[i].genes.data();
                    model.computeApproximateMutations(active_variables.size(), active_variables.data(), child_count, genotypes.data(), phenotypes);
                }
                
                // fitness
                {
                    BLOCKPROFILER("fitness");
                    for(size_t child_index = 0; child_index < children.size(); child_index++)
                    {
                        children[child_index].fitness = computeFitness(phenotypes[child_index]);
                    }
                }

                // selection
                {
                    BLOCKPROFILER("selection");
                    child_indices.resize(children.size());
                    for(size_t i = 0; i < children.size(); i++) 
                        child_indices[i] = i;
                    for(size_t i = 0; i < population.size(); i++)
                    {
                        size_t jmin = i;
                        double fmin = children[child_indices[i]].fitness;
                        for(size_t j = i + 1; j < children.size(); j++)
                        {
                            double f = children[child_indices[j]].fitness;
                            if(f < fmin) 
                                jmin = j, fmin = f;
                        }
                        std::swap(child_indices[i], child_indices[jmin]);
                    }
                    for(size_t i = 0; i < population.size(); i++)
                    {
                        std::swap(population[i].genes, children[child_indices[i]].genes);
                        std::swap(population[i].gradients, children[child_indices[i]].gradients);
                    }
                }
                
                // exploit
                /*{
                    BLOCKPROFILER("exploit");
                    
                    auto& individual = population[0];
                    
                    auto gene_count = individual.genes.size();
                    
                    //double mutation_scale = (1 << fast_random_index(16)) * (1.0 / (1 << 20));
                    
                    for(size_t i = 0; i < active_variables.size(); i++)
                        solution[active_variables[i]] = individual.genes[i];
                    
                    genotypes[0] = individual.genes.data();
                    model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes);
                    phenotype = phenotypes[0];
                    individual.fitness = computeFitness(phenotype);
                    
                    for(size_t iteration = 0; iteration < 1; iteration++)
                    {
                        for(size_t gene_index = 0; gene_index < gene_count; gene_index++)
                        {
                            //double mutation_scale = (1 << fast_random_index(16)) * (1.0 / (1 << 20));
                            
                            const size_t mutation_count = 8;
                            double mutation_values[mutation_count];
                            double* ptr_mutation_values[mutation_count];
                            for(size_t i = 0; i < mutation_count / 2; i++)
                            {
                                double mutation_scale = (1 << fast_random_index(16)) * (1.0 / (1 << 20));
                                //double mutation_scale = (1 << fast_random_index(16)) * (1.0 / (1 << 16));
                                
                                double g = individual.gradients[gene_index];
                                g *= i % 2;
                                
                                mutation_values[i * 2 + 0] = individual.genes[gene_index] + mutation_scale + g;
                                mutation_values[i * 2 + 1] = individual.genes[gene_index] - mutation_scale + g;
                            }

                            auto& gene_info = gene_infos[gene_index];
                            for(size_t mutation_index = 0; mutation_index < mutation_count; mutation_index++)
                            {
                                mutation_values[mutation_index] = clamp2(mutation_values[mutation_index], gene_info.clip_min, gene_info.clip_max);
                                ptr_mutation_values[mutation_index] = &(mutation_values[mutation_index]);
                            }
                            
                            //model.computeApproximateMutations(1, &gene_index, mutation_count, &(ptr_mutation_values[0]), phenotypes);
                            
                            model.computeApproximateMutations(1, &gene_index, mutation_count, &(ptr_mutation_values[0]), solution, phenotype, phenotypes);
                            
                            double best_mutation_fitness = individual.fitness;
                            ssize_t best_mutation_index = -1;
                            
                            for(size_t mutation_index = 0; mutation_index < mutation_count; mutation_index++)
                            {
                                double mutation_fitness = computeFitness(phenotypes[mutation_index]);
                                if(mutation_fitness < best_mutation_fitness)
                                {
                                    best_mutation_fitness = mutation_fitness;
                                    best_mutation_index = mutation_index;
                                }
                            }
                            
                            if(best_mutation_index >= 0)
                            {
                                individual.fitness = best_mutation_fitness;
                                individual.gradients[gene_index] = mix(individual.gradients[gene_index], mutation_values[best_mutation_index] - individual.genes[gene_index], 0.1);
                                individual.genes[gene_index] = mutation_values[best_mutation_index];
                                solution[active_variables[gene_index]] = individual.genes[gene_index];
                                phenotype = phenotypes[best_mutation_index];
                            }
                        }
                    }
                }*/
                
                
                //if(canceled) return;

            }
        }
        
        {
            BLOCKPROFILER("species");
        
            // compute species fitness
            for(auto& species : this->species)
            {
                genesToJointVariables(species.individuals[0].genes, temp_joint_variables);
                double fitness = computeFitness(temp_joint_variables);
                species.improved = (fitness != species.fitness);
                species.fitness = fitness;
            }
                
            // sort species by fitness
            std::sort(species.begin(), species.end(), [] (const Species& a, const Species& b) { return a.fitness < b.fitness; });
            
            // wipeouts
            for(size_t species_index = 1; species_index < species.size(); species_index++)
            {
                if(fast_random() < 0.1 || !species[species_index].improved)
                    for(auto& individual : species[species_index].individuals)
                        for(size_t i = 0; i < individual.genes.size(); i++)
                            individual.genes[i] = random(modelInfo.getMin(active_variables[i]), modelInfo.getMax(active_variables[i]));
            }

            // update solution
            if(species[0].fitness < solution_fitness)
            {
                genesToJointVariables(species[0].individuals[0].genes, solution);
                solution_fitness = species[0].fitness;
            }
        }
    }
    
    // number of islands
    virtual size_t concurrency() const { return 4; }
};

static IKFactory::Class<IKEvolution2> cIKEvolution2("bio2");

}








