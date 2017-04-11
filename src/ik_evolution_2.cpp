// Bio IK for ROS
// Philipp Ruppel

#include "ik_base.h"

#include "../../CppNumericalSolvers/include/cppoptlib/solver/lbfgssolver.h"

namespace bio_ik
{

// fast evolutionary inverse kinematics
template<int memetic>
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
    std::vector<std::vector<Frame>> phenotypes, phenotypes2, phenotypes3;
    std::vector<size_t> child_indices;
    std::vector<double*> genotypes;
    std::vector<Frame> phenotype;
    std::vector<size_t> quaternion_genes;
    //std::vector<size_t> gene_resets;
    
    /*struct GeneInfo
    {
        double clip_min, clip_max, span;
    };
    std::vector<GeneInfo> gene_infos;*/
    
    aligned_vector<double> genes_min, genes_max, genes_span;
    
    IKEvolution2(const IKParams& p) : IKBase(p)
    {
    }
    
    
    
    
    struct OptlibProblem : cppoptlib::Problem<double> 
    {
        IKEvolution2* ik;
        //std::vector<double> fk_values;
        OptlibProblem(IKEvolution2* ik) : ik(ik)
        {
        }
        void initialize()
        {
            //fk_values = ik->request.initial_guess;
        }
        double value(const TVector& x) 
        {
            //for(size_t i = 0; i < ik->active_variables.size(); i++) fk_values[ik->active_variables[i]] = x[i];
            //return ik->computeFitness(fk_values);
            const double* genes = x.data();
            ik->model.computeApproximateMutations(ik->active_variables.size(), ik->active_variables.data(), 1, &genes, ik->phenotypes);
            return ik->computeCombinedFitnessActiveVariables(ik->phenotypes[0], genes);
        }
    };
    typedef cppoptlib::LbfgsSolver<OptlibProblem> OptlibSolver;
    std::shared_ptr<OptlibSolver> optlib_solver;
    std::shared_ptr<OptlibProblem> optlib_problem;
    typename OptlibSolver::TVector optlib_vector;
    
    
    
    
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
        
        // init list of quaternion joint genes to be normalized during each mutation
        quaternion_genes.clear();
        for(size_t igene = 0; igene < active_variables.size(); igene++)
        {
            size_t ivar = active_variables[igene];
            auto* joint_model = params.robot_model->getJointOfVariable(ivar);
            if(joint_model->getFirstVariableIndex() + 3 != ivar) continue;
            if(joint_model->getType() != moveit::core::JointModel::FLOATING) continue;
            quaternion_genes.push_back(igene);
        }
        
        /*// init gene reset buffer
        // for preferentially mutating towards initial guess
        // weighted by inverse max joint velocity
        gene_resets.clear();
        if(active_variables.size())
        {
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
                double f = fast_random() * mutation_cost_sat.back();
                size_t i = 0;
                while(mutation_cost_sat[i] < f) i++;
                if(i >= mutation_cost_sat.size()) i = mutation_cost_sat.size() - 1;
                gene_resets.push_back(i);
            }
        }*/
        
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
            }
            
            
            /*double max_velocity_sum = 0.0;
            double max_velocity_div = 0.0;
            for(size_t i = 0; i < active_variables.size(); i++)
            {
                if(modelInfo.getMaxVelocity(active_variables[i]) > 0.0)
                {
                    max_velocity_sum += modelInfo.getMaxVelocity(active_variables[i]);
                    max_velocity_div += 1;
                }
            }
            if(max_velocity_div > 0.0)
            {
                double max_velocity_avg = max_velocity_sum / max_velocity_div;
                double max_velocity_scale = 1.0 / max_velocity_avg;
                for(size_t i = 0; i < active_variables.size(); i++)
                {
                    double f = 1.0;
                    if(modelInfo.getMaxVelocity(active_variables[i]) > 0.0)
                        f = modelInfo.getMaxVelocity(active_variables[i]) * max_velocity_scale;
                    //genes_span[i] *= f;
                    //LOG_VAR(f);
                    genes_span[i] = f;
                }
            }*/
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
            
            //gradient_factor = 0;
            
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
            
            /*//if(!gene_resets.empty() && thread_index == 0 && fast_random_index(4) == 0)
            if(!gene_resets.empty() && thread_index == 0 && fast_random_index(4) != 0)
            {
                auto gene_index = fast_random_element(gene_resets);
                auto& p1 = children[child_index].genes[gene_index];
                auto p2 = mix(p1, initial_guess[active_variables[gene_index]], fast_random());
                auto& dp = children[child_index].gradients[gene_index];
                dp = mix(dp, p2 - p1, 0.1);
                p1 = p2;
            }*/
            
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
            
            size_t generation_count = 16;
            if(memetic) generation_count = 8;
            
            // run evolution for a few generations
            for(size_t generation = 0; generation < generation_count; generation++)
            //for(size_t generation = 0; generation < (ispecies == 0 ? 16 : 8); generation++)
            {
                
                // reproduction
                {
                    BLOCKPROFILER("reproduction");
                    reproduce(population);
                }
                
                size_t child_count = children.size();
                
                // pre-selection
                //if(false)
                if(request.secondary_goals.size())
                {
                    BLOCKPROFILER("pre-selection");

                    //child_count = (children.size() + population.size()) / 2;
                    
                    //child_count = (children.size() + population.size()) * 3 / 4;
                    
                    //child_count = (children.size() - population.size()) * (random_index(4) + 1) / 4 + population.size();

                    //child_count = random_index(children.size() - population.size() - 1) + 1 + population.size();
                    
                    //child_count = fast_random_index(children.size() - population.size() - 2) + population.size();
                    
                    child_count = random_index(children.size() - population.size() - 1) + 1 + population.size();
                    
                    //LOG_VAR(child_count);
                    
                    //child_count = 15;
                    for(size_t child_index = population.size(); child_index < children.size(); child_index++)
                    {
                        children[child_index].fitness = computeSecondaryFitnessActiveVariables(children[child_index].genes.data());
                        //LOG_VAR(children[child_index].fitness);
                    }
                        
                    {
                        BLOCKPROFILER("pre-selection sort");
                        std::sort(
                                children.begin() + population.size(),
                                children.end(), 
                                [] (const Individual& a, const Individual& b)
                                {
                                    return a.fitness < b.fitness;
                                }
                            );
                    }
                        
                    //LOG_VAR("c");
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
                    size_t gene_count = children[0].genes.size();
                    genotypes.resize(child_count);
                    for(size_t i = 0; i < child_count; i++)
                        genotypes[i] = children[i].genes.data();
                    model.computeApproximateMutations(active_variables.size(), active_variables.data(), child_count, genotypes.data(), phenotypes);
                }
                
                // fitness
                {
                    BLOCKPROFILER("fitness");
                    for(size_t child_index = 0; child_index < child_count; child_index++)
                    {
                        children[child_index].fitness = computeFitnessActiveVariables(phenotypes[child_index], genotypes[child_index]);
                    }
                }

                // selection
                {
                    BLOCKPROFILER("selection");
                    child_indices.resize(child_count);
                    for(size_t i = 0; i < child_count; i++) 
                        child_indices[i] = i;
                    for(size_t i = 0; i < population.size(); i++)
                    {
                        size_t jmin = i;
                        double fmin = children[child_indices[i]].fitness;
                        for(size_t j = i + 1; j < child_count; j++)
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
            
            
            
            
            /*{
                
                auto& individual = population[0];
                
                gradient.resize(active_variables.size());
                
                for(size_t generation = 0; generation < 16; generation++)
                {
                    genotypes[0] = individual.genes.data();
                    model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes2);
                    
                    double dp = 0.00001;
                    
                    for(size_t i = 0; i < active_variables.size(); i++)
                    {
                        double* pp = &(genotypes[0][i]);
                        
                        genotypes[0][i] = individual.genes[i] - dp;
                        model.computeApproximateMutations(1, &(active_variables[i]), 1, &pp, phenotypes3, phenotypes2[0]);
                        double fa = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                        
                        genotypes[0][i] = individual.genes[i] + dp;
                        model.computeApproximateMutations(1, &(active_variables[i]), 1, &pp, phenotypes3, phenotypes2[0]);
                        double fb = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                        
                        genotypes[0][i] = individual.genes[i];
                        
                        gradient[i] = fb - fa;
                    }
                    
                    
                    double sum = dp * dp;
                    for(size_t i = 0; i < active_variables.size(); i++)
                        sum += fabs(gradient[i]);
                    double f = 1.0 / sum * dp;
                    for(size_t i = 0; i < active_variables.size(); i++)
                        gradient[i] *= f;
                        
                        
                    for(size_t i = 0; i < active_variables.size(); i++) genotypes[0][i] = individual.genes[i] - gradient[i];
                    model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3, phenotypes2[0]);
                    double f1 = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                    
                    for(size_t i = 0; i < active_variables.size(); i++) genotypes[0][i] = individual.genes[i];
                    model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3, phenotypes2[0]);
                    double f2 = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                    
                    for(size_t i = 0; i < active_variables.size(); i++) genotypes[0][i] = individual.genes[i] + gradient[i];
                    model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3, phenotypes2[0]);
                    double f3 = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                    
                    
                    
                    double cost_diff = (f3 - f1) * 0.5;
                    double joint_diff = f2 / cost_diff;
                    
                    
                    
                    for(size_t i = 0; i < active_variables.size(); i++)
                        individual.genes[i] = modelInfo.clip(individual.genes[i] - gradient[i] * joint_diff, active_variables[i]);
                    
                    
                }
            
            }*/
            
            
            
            
            
            if(memetic == 'q' || memetic == 'l')
            {
                //if(ispecies == 0)
                {
                
                    auto& individual = population[0];
                    
                    //genesToJointVariables(individual, temp_joint_variables);
                    //model.applyConfiguration(temp_joint_variables);
                    //model.initializeMutationApproximator(active_variables);
                    
                    gradient.resize(active_variables.size());
                    
                    if(genotypes.empty()) genotypes.emplace_back();
                    
                    phenotypes2.resize(1);
                    phenotypes3.resize(1);
                    
                    double dp = 0.0000001;
                    if(fast_random() < 0.5) dp = -dp;
                    
                    for(size_t generation = 0; generation < 8; generation++)
                    {
                        temp = individual.genes;
                        genotypes[0] = temp.data();
                        
                        model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes2);
                        
                        //double fa = computeCombinedFitnessActiveVariables(phenotypes2[0], genotypes[0]);
                        
                        double f2p = computeFitnessActiveVariables(phenotypes2[0], genotypes[0]);
                        
                        double fa = f2p + computeSecondaryFitnessActiveVariables(genotypes[0]);
                        
                        //double dp = 0.000001;
                        
                        //if(generation % 2) dp = -dp;
                        
                        for(size_t i = 0; i < active_variables.size(); i++)
                        {
                            double* pp = &(genotypes[0][i]);
                            
                            // genotypes[0][i] = individual.genes[i] - dp;
                            // //model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                            // model.computeApproximateMutation1(active_variables[i], -dp, phenotypes2[0], phenotypes3[0]);
                            // double fa = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            genotypes[0][i] = individual.genes[i] + dp;
                            //model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                            model.computeApproximateMutation1(active_variables[i], +dp, phenotypes2[0], phenotypes3[0]);
                            double fb = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            genotypes[0][i] = individual.genes[i];
                            
                            double d = fb - fa;
                            
                            //if(d < 0 && individual.genes[i] == modelInfo.getClipMin(active_variables[i])) d = 0;
                            //if(d > 0 && individual.genes[i] == modelInfo.getClipMax(active_variables[i])) d = 0;
                            
                            gradient[i] = d;
                            
                            //gradient[i] *= modelInfo.getMaxVelocity(active_variables[i]);
                            
                            //if(memetic == 'q') gradient[i] *= fabs(gradient[i]);
                            //if(memetic == 'q') gradient[i] = 1.0f / (dp * dp + fabs(gradient[i])) * sign(gradient[i]);
                        }
                         
                        
                        double sum = dp * dp;
                        for(size_t i = 0; i < active_variables.size(); i++)
                            sum += fabs(gradient[i]);
                        double f = 1.0 / sum * dp;
                        for(size_t i = 0; i < active_variables.size(); i++)
                            gradient[i] *= f;
                            
                            
                        for(size_t i = 0; i < active_variables.size(); i++) genotypes[0][i] = individual.genes[i] - gradient[i];
                        model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                        double f1 = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                        
                        //for(size_t i = 0; i < active_variables.size(); i++) genotypes[0][i] = individual.genes[i];
                        //model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                        //double f2 = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                        
                        double f2 = fa;
                        
                        for(size_t i = 0; i < active_variables.size(); i++) genotypes[0][i] = individual.genes[i] + gradient[i];
                        model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                        double f3 = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                        
                        
                        if(memetic == 'q')
                        {
                        
                            double v1 = (f2 - f1); // f / j
                            double v2 = (f3 - f2); // f / j
                            double v = (v2 + v1) * 0.5; // f / j
                            double a = (v2 - v1); // f / j^2
                            double joint_diff = v / a; // (f / j) / (f / j^2) = f / j / f * j * j = j

                            for(size_t i = 0; i < active_variables.size(); i++) temp[i] = modelInfo.clip(individual.genes[i] - gradient[i] * joint_diff, active_variables[i]);
                            
                            genotypes[0] = temp.data();
                            model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes2);
                            
                            double f4p = computeFitnessActiveVariables(phenotypes2[0], genotypes[0]);
                            
                            if(f4p < f2p)
                            {
                                //for(size_t i = 0; i < active_variables.size(); i++) individual.gradients[i] = mix(individual.gradients[i], temp[i] - individual.genes[i], 0.1);
                                individual.genes = temp;
                                continue;
                            }
                            else
                            {
                                break;
                            }
                        
                        }
                        
                        if(memetic == 'l')
                        {
                        
                            double cost_diff = (f3 - f1) * 0.5;
                            double joint_diff = f2 / cost_diff;
                        
                            for(size_t i = 0; i < active_variables.size(); i++) temp[i] = modelInfo.clip(individual.genes[i] - gradient[i] * joint_diff, active_variables[i]);
                            
                            genotypes[0] = temp.data();
                            model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes2);
                            
                            double f4p = computeFitnessActiveVariables(phenotypes2[0], genotypes[0]);
                            
                            if(f4p < f2p)
                            {
                                individual.genes = temp;
                                continue;
                            }
                            else
                            {
                                break;
                            }
                        
                        }
                        
                        
                        
                    }
                
                }
                
            }
            
            
            
            
            if(memetic == 'o' /*&& fast_random() < 0.1*/)
            {
                auto& individual = population[0];
            
                if(!optlib_solver)
                {
                    optlib_solver = std::make_shared<OptlibSolver>();
                    cppoptlib::Criteria<double> crit;
                    crit.iterations = 4;
                    //crit.iterations = 1;
                    optlib_solver->setStopCriteria(crit);
                    optlib_problem = std::make_shared<OptlibProblem>(this);
                }
                
                optlib_vector.resize(active_variables.size());
                for(size_t i = 0; i < active_variables.size(); i++) optlib_vector[i] = individual.genes[i];
                
                optlib_problem->initialize();
                
                optlib_solver->minimize(*optlib_problem, optlib_vector);
                
                for(size_t i = 0; i < active_variables.size(); i++) individual.genes[i] = modelInfo.clip(optlib_vector[i], active_variables[i]);
            }
            
            
            
            
            /*
            if(memetic)
            {
                //if(ispecies == 0)
                {
                
                    auto& individual = population[0];
                    
                    //genesToJointVariables(individual, temp_joint_variables);
                    //model.applyConfiguration(temp_joint_variables);
                    //model.initializeMutationApproximator(active_variables);
                    
                    gradient.resize(active_variables.size());
                    
                    if(genotypes.empty()) genotypes.emplace_back();
                    
                    phenotypes2.resize(1);
                    phenotypes3.resize(1);
                    
                    double dp = 0.0000001;
                    if(fast_random() < 0.5) dp = -dp;
                    
                    for(size_t generation = 0; generation < 8; generation++)
                    {
                        temp = individual.genes;
                        genotypes[0] = temp.data();
                        
                        for(size_t i = 0; i < active_variables.size(); i++)
                        {
                            double* pp = &(genotypes[0][i]);
                            
                            genotypes[0][i] = individual.genes[i] - dp;
                            model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                            double f1 = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            genotypes[0][i] = individual.genes[i];
                            model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                            double f2 = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            genotypes[0][i] = individual.genes[i] + dp;
                            model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                            double f3 = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            double v1 = (f2 - f1); // f / j
                            double v2 = (f3 - f2); // f / j
                            double v = (v2 + v1) * 0.5; // f / j
                            double a = (v2 - v1); // f / j^2
                            double joint_diff = -v / a;

                            genotypes[0][i] = modelInfo.clip(individual.genes[i] + joint_diff, active_variables[i]);
                            model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes3);
                            double fx = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            if(fx < f2)
                            {
                                individual.genes[i] = genotypes[0][i];
                            }
                            else
                            {
                                genotypes[0][i] = individual.genes[i];
                            }
                            
                            //break;
                        }
                        
                        //break;
                    }
                
                }
                
            }
            */
            
            /*
            if(memetic)
            {
                //if(ispecies == 0)
                {
                
                    auto& individual = population[0];
                    
                    //genesToJointVariables(individual, temp_joint_variables);
                    //model.applyConfiguration(temp_joint_variables);
                    //model.initializeMutationApproximator(active_variables);
                    
                    gradient.resize(active_variables.size());
                    
                    if(genotypes.empty()) genotypes.emplace_back();
                    
                    phenotypes2.resize(1);
                    phenotypes3.resize(1);
                    
                    double dp = 0.0000001;
                    if(fast_random() < 0.5) dp = -dp;
                    
                    for(size_t generation = 0; generation < 8; generation++)
                    {
                        temp = individual.genes;
                        genotypes[0] = temp.data();
                        
                        model.computeApproximateMutations(active_variables.size(), active_variables.data(), 1, genotypes.data(), phenotypes2);
                        double f2 = computeFitnessActiveVariables(phenotypes2[0], genotypes[0]);
                        
                        for(size_t i = 0; i < active_variables.size(); i++)
                        {
                            double* pp = &(genotypes[0][i]);
                            
                            genotypes[0][i] = individual.genes[i] - dp;
                            model.computeApproximateMutation1(active_variables[i], -dp, phenotypes2[0], phenotypes3[0]);
                            double f1 = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            genotypes[0][i] = individual.genes[i] + dp;
                            model.computeApproximateMutation1(active_variables[i], +dp, phenotypes2[0], phenotypes3[0]);
                            double f3 = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            genotypes[0][i] = individual.genes[i];
                            
                            double v1 = (f2 - f1); // f / j
                            double v2 = (f3 - f2); // f / j
                            double v = (v2 + v1) * 0.5; // f / j
                            double a = (v2 - v1); // f / j^2
                            double joint_diff = v / a;

                            genotypes[0][i] = modelInfo.clip(individual.genes[i] + joint_diff, active_variables[i]);
                            model.computeApproximateMutation1(active_variables[i], +joint_diff, phenotypes2[0], phenotypes3[0]);
                            double fx = computeFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            
                            if(fx < f2)
                            {
                                f2 = fx;
                                phenotypes2[0] = phenotypes3[0];
                                individual.genes[i] = genotypes[0][i];
                            }
                            else
                            {
                                genotypes[0][i] = individual.genes[i];
                            }
                        }
                        
                    }
                
                }
                
            }
            */
            
            
            
            
            
            
            
            
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
    
    aligned_vector<double> gradient, temp;
    
    // number of islands
    virtual size_t concurrency() const { return 4; }
};

static IKFactory::Class<IKEvolution2<0>> bio2("bio2");
static IKFactory::Class<IKEvolution2<'q'>> bio2_memetic("bio2_memetic");
static IKFactory::Class<IKEvolution2<'l'>> bio2_memetic_l("bio2_memetic_l");
static IKFactory::Class<IKEvolution2<'o'>> bio2_memetic_0("bio2_memetic_lbfgs");

}








