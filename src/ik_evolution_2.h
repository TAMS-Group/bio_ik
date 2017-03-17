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
        aligned_vector<double> genes;
        aligned_vector<double> gradients;
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
    std::vector<size_t> quaternion_genes;
    
    /*struct GeneInfo
    {
        double clip_min, clip_max, span;
    };
    std::vector<GeneInfo> gene_infos;*/
    
    aligned_vector<double> genes_min, genes_max, genes_span;
    
    IKEvolution2(const IKParams& p) : IKBase(p)
    {
    }
    
    
    void genesToJointVariables(const Individual& individual, std::vector<double>& variables)
    {
        auto& genes = individual.genes;
        variables.resize(params.robot_model->getVariableCount());
        for(size_t i = 0; i < active_variables.size(); i++)
            variables[active_variables[i]] = genes[i];
    }
    
    const std::vector<double>& getSolution() const
    {
        return solution;
    }

    void initialize(const IKRequest& request)
    {
        BLOCKPROFILER("initialization");
    
        IKBase::initialize(request);
        
        quaternion_genes.clear();
        for(size_t igene = 0; igene < active_variables.size(); igene++)
        {
            size_t ivar = active_variables[igene];
            auto* joint_model = params.robot_model->getJointOfVariable(ivar);
            if(joint_model->getFirstVariableIndex() + 3 != ivar) continue;
            if(joint_model->getType() != moveit::core::JointModel::FLOATING) continue;
            quaternion_genes.push_back(igene);
        }
        
        // set solution to initial guess
        initial_guess = request.initial_guess;
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
                if(1)
                {
                    // set to initial_guess
                    for(size_t i = 0; i < v.genes.size(); i++)
                        v.genes[i] = initial_guess[active_variables[i]];
                } 
                else 
                {
                    // initialize populations on other islands randomly
                    for(size_t i = 0; i < v.genes.size(); i++)
                        v.genes[i] = random(modelInfo.getMin(active_variables[i]), modelInfo.getMax(active_variables[i]));
                }
                
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
        //if(genes_min.empty())
        {
            genes_min.resize(active_variables.size());
            genes_max.resize(active_variables.size());
            genes_span.resize(active_variables.size());
            for(size_t i = 0; i < active_variables.size(); i++)
            {
                genes_min[i] = modelInfo.getClipMin(active_variables[i]);
                genes_max[i] = modelInfo.getClipMax(active_variables[i]);
                genes_span[i] = modelInfo.getSpan(active_variables[i]);
                //LOG("genes_span", i, genes_span[i]);
            }
        }
    }
    
    // create offspring and mutate
    

    
    __attribute__((hot))
    __attribute__((noinline))
    //__attribute__((target_clones("avx2", "avx", "sse2", "default")))
    //__attribute__((target("avx")))
    void reproduce(const std::vector<Individual>& population)
    {
        const auto __attribute__((aligned(32))) * __restrict__ genes_span = this->genes_span.data();
        const auto __attribute__((aligned(32))) * __restrict__ genes_min = this->genes_min.data();
        const auto __attribute__((aligned(32))) * __restrict__ genes_max = this->genes_max.data();
    
        auto gene_count = children[0].genes.size();
        
        auto* __restrict__ rr = fast_random_gauss_n((children.size() - population.size()) * gene_count + children.size() * 4 + 4);
        rr = (const double*)(((size_t)rr + 3) / 4 * 4);
        
        for(size_t child_index = population.size(); child_index < children.size(); child_index++)
        {
            double mutation_rate = (1 << fast_random_index(16)) * (1.0 / (1 << 23));
            auto& parent = population[0];
            auto& parent2 = population[1];
            double fmix = (child_index % 2 == 0) * 0.2;
            double gradient_factor = child_index % 3;
            
            auto __attribute__((aligned(32))) * __restrict__ parent_genes = parent.genes.data();
            auto __attribute__((aligned(32))) * __restrict__ parent_gradients = parent.gradients.data();
            
            auto __attribute__((aligned(32))) * __restrict__ parent2_genes = parent2.genes.data();
            auto __attribute__((aligned(32))) * __restrict__ parent2_gradients = parent2.gradients.data();
            
            auto& child = children[child_index];
            
            auto __attribute__((aligned(32))) * __restrict__ child_genes = child.genes.data();
            auto __attribute__((aligned(32))) * __restrict__ child_gradients = child.gradients.data();
            
            /*genes_span = (const double*)__builtin_assume_aligned(genes_span, 32);
            genes_min = (const double*)__builtin_assume_aligned(genes_min, 32);
            genes_max = (const double*)__builtin_assume_aligned(genes_max, 32);
            parent_genes = (double*)__builtin_assume_aligned(parent_genes, 32);
            parent_gradients = (double*)__builtin_assume_aligned(parent_gradients, 32);
            parent2_genes = (double*)__builtin_assume_aligned(parent2_genes, 32);
            parent2_gradients = (double*)__builtin_assume_aligned(parent2_gradients, 32);
            child_genes = (double*)__builtin_assume_aligned(child_genes, 32);
            child_gradients = (double*)__builtin_assume_aligned(child_gradients, 32);
            rr = (double*)__builtin_assume_aligned(rr, 32);*/
            
            #pragma unroll
            #pragma omp simd \
                aligned(genes_span:32), \
                aligned(genes_min:32), \
                aligned(genes_max:32), \
                aligned(parent_genes:32), \
                aligned(parent_gradients:32), \
                aligned(parent2_genes:32), \
                aligned(parent2_gradients:32), \
                aligned(child_genes:32), \
                aligned(child_gradients:32) \
                aligned(rr:32)
            for(size_t gene_index = 0; gene_index < gene_count; gene_index++)
            {
                double r = rr[gene_index];
                double f = mutation_rate * genes_span[gene_index];
                double gene = parent_genes[gene_index];
                double parent_gene = gene;
                gene += r * f;
                double parent_gradient = mix(parent_gradients[gene_index], parent2_gradients[gene_index], fmix);
                double gradient = parent_gradient * gradient_factor;
                gene += gradient;
                gene = clamp(gene, genes_min[gene_index], genes_max[gene_index]);
                child_genes[gene_index] = gene;
                child_gradients[gene_index] = mix(parent_gradient, gene - parent_gene, 0.3);
            }
            
            rr += (gene_count + 3) / 4 * 4;
            
            for(auto quaternion_gene_index : quaternion_genes)
            {
                auto& qpos = (*(Quaternion*)&(children[child_index].genes[quaternion_gene_index]));
                normalizeFast(qpos);
            }
        }
    }
    
    /*__attribute__((hot))
    __attribute__((noinline))
    void reproduce(const std::vector<Individual>& population)
    {
        auto gene_count = children[0].genes.size();
        auto* rr = fast_random_gauss_n((children.size() - population.size()) * gene_count);
        for(size_t child_index = population.size(); child_index < children.size(); child_index++)
        {
            double mutation_rate = (1 << fast_random_index(16)) * (1.0 / (1 << 23));
            auto& parent = population[0];
            auto& parent2 = population[1];
            double fmix = (child_index % 2 == 0) * 0.2;
            double gradient_factor = child_index % 3;
            
            #pragma omp simd
            for(size_t gene_index = 0; gene_index < gene_count; gene_index++)
            {
                double r = rr[gene_index];
                double f = mutation_rate * genes_span[gene_index];
                double gene = parent.genes[gene_index];
                double parent_gene = gene;
                gene += r * f;
                double parent_gradient = mix(parent.gradients[gene_index], parent2.gradients[gene_index], fmix);
                double gradient = parent_gradient * gradient_factor;
                gene += gradient;
                gene = clamp(gene, genes_min[gene_index], genes_max[gene_index]);
                children[child_index].genes[gene_index] = gene;
                children[child_index].gradients[gene_index] = mix(parent_gradient, gene - parent_gene, 0.3);
            }
            
            rr += gene_count;
            
            for(auto quaternion_gene_index : quaternion_genes)
            {
                auto& qpos = (*(Quaternion*)&(children[child_index].genes[quaternion_gene_index]));
                //qpos.normalize();
                normalizeFast(qpos);
                //auto& qvel = (*(Quaternion*)&(children[child_index].gradients[quaternion_gene_index]));
                //qvel -= qvel * qvel.dot(qpos);
            }
        }
    }*/

    void step()
    {
        FNPROFILER();

        for(size_t ispecies = 0; ispecies < species.size(); ispecies++)
        {
            auto& species = this->species[ispecies];
        
            BLOCKPROFILER("evolution");
            
            auto& population = species.individuals;
            
            // initialize forward kinematics approximator
            genesToJointVariables(species.individuals[0], temp_joint_variables);
            model.applyConfiguration(temp_joint_variables);
            model.initializeMutationApproximator(active_variables);
            
            //LOG_VAR(model.getTipFrames()[0]);
            
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
                                //individual.gradients[gene_index] = mix(individual.gradients[gene_index], mutation_values[best_mutation_index] - individual.genes[gene_index], 0.1);
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
                genesToJointVariables(species.individuals[0], temp_joint_variables);
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
                {
                    {
                        auto& individual = species[species_index].individuals[0];
                        for(size_t i = 0; i < individual.genes.size(); i++)
                            individual.genes[i] = random(modelInfo.getMin(active_variables[i]), modelInfo.getMax(active_variables[i]));
                    }
                    for(size_t i = 0; i < species[species_index].individuals.size(); i++)
                        species[species_index].individuals[i] = species[species_index].individuals[0];
                }
            }

            // update solution
            if(species[0].fitness < solution_fitness)
            {
                genesToJointVariables(species[0].individuals[0], solution);
                solution_fitness = species[0].fitness;
            }
        }
    }
    
    // number of islands
    virtual size_t concurrency() const { return 4; }
};

static IKFactory::Class<IKEvolution2> cIKEvolution2("bio2");

}








