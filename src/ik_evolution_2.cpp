/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-2017, Philipp Sebastian Ruppel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "ik_base.h"

#ifdef ENABLE_CPP_OPTLIB
#include "cppoptlib/solver/lbfgssolver.h"
#endif

namespace bio_ik
{

// fast evolutionary inverse kinematics
template <int memetic> struct IKEvolution2 : IKBase
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
    std::vector<aligned_vector<Frame>> phenotypes, phenotypes2, phenotypes3;
    std::vector<size_t> child_indices;
    std::vector<double*> genotypes;
    std::vector<Frame> phenotype;
    std::vector<size_t> quaternion_genes;
    aligned_vector<double> genes_min, genes_max, genes_span;
    aligned_vector<double> gradient, temp;

    IKEvolution2(const IKParams& p)
        : IKBase(p)
    {
    }

#ifdef ENABLE_CPP_OPTLIB
    struct OptlibProblem : cppoptlib::Problem<double>
    {
        IKEvolution2* ik;
        OptlibProblem(IKEvolution2* ik)
            : ik(ik)
        {
        }
        double value(const TVector& x)
        {
            const double* genes = x.data();
            ik->model.computeApproximateMutations(1, &genes, ik->phenotypes);
            return ik->computeCombinedFitnessActiveVariables(ik->phenotypes[0], genes);
        }
    };
    typedef cppoptlib::LbfgsSolver<OptlibProblem> OptlibSolver;
    std::shared_ptr<OptlibSolver> optlib_solver;
    std::shared_ptr<OptlibProblem> optlib_problem;
    typename OptlibSolver::TVector optlib_vector;
#endif

    void genesToJointVariables(const Individual& individual, std::vector<double>& variables)
    {
        auto& genes = individual.genes;
        variables.resize(params.robot_model->getVariableCount());
        for(size_t i = 0; i < problem.active_variables.size(); i++)
            variables[problem.active_variables[i]] = genes[i];
    }

    const std::vector<double>& getSolution() const { return solution; }

    void initialize(const Problem& problem)
    {
        BLOCKPROFILER("initialization");

        IKBase::initialize(problem);

        // init list of quaternion joint genes to be normalized during each mutation
        quaternion_genes.clear();
        for(size_t igene = 0; igene < problem.active_variables.size(); igene++)
        {
            size_t ivar = problem.active_variables[igene];
            auto* joint_model = params.robot_model->getJointOfVariable(ivar);
            if(joint_model->getFirstVariableIndex() + 3 != ivar) continue;
            if(joint_model->getType() != moveit::core::JointModel::FLOATING) continue;
            quaternion_genes.push_back(igene);
        }

        // set solution to initial guess
        initial_guess = problem.initial_guess;
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
                v.genes.resize(problem.active_variables.size());
                // if(thread_index == 0) // on first island?
                // if(thread_index % 2 == 0) // on every second island...
                if(1)
                {
                    // set to initial_guess
                    for(size_t i = 0; i < v.genes.size(); i++)
                        v.genes[i] = initial_guess[problem.active_variables[i]];
                }
                else
                {
                    // initialize populations on other islands randomly
                    for(size_t i = 0; i < v.genes.size(); i++)
                        v.genes[i] = random(modelInfo.getMin(problem.active_variables[i]), modelInfo.getMax(problem.active_variables[i]));
                }

                // set gradients to zero
                v.gradients.clear();
                v.gradients.resize(problem.active_variables.size(), 0);
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
            child.genes.resize(problem.active_variables.size());
            child.gradients.resize(problem.active_variables.size());
        }

        // init gene infos
        // if(genes_min.empty())
        {
            genes_min.resize(problem.active_variables.size());
            genes_max.resize(problem.active_variables.size());
            genes_span.resize(problem.active_variables.size());
            for(size_t i = 0; i < problem.active_variables.size(); i++)
            {
                genes_min[i] = modelInfo.getClipMin(problem.active_variables[i]);
                genes_max[i] = modelInfo.getClipMax(problem.active_variables[i]);
                genes_span[i] = modelInfo.getSpan(problem.active_variables[i]);
            }
        }

        /*
        // init chain mutation masks
        chain_mutation_masks.resize(chain_mutation_mask_count);
        for(auto& chain_mutation_mask : chain_mutation_masks)
        {
            temp_mutation_chain.clear();
            if(problem.tip_link_indices.size() > 1)
            {
                for(auto* chain_link = params.robot_model->getLinkModel(random_element(problem.tip_link_indices)); chain_link; chain_link = chain_link->getParentLinkModel())
                    temp_mutation_chain.push_back(chain_link);
                temp_mutation_chain.resize(random_index(temp_mutation_chain.size()) + 1);
            }

            temp_chain_mutation_var_mask.resize(params.robot_model->getVariableCount());
            for(auto& m : temp_chain_mutation_var_mask) m = 0;
            for(auto* chain_link : temp_mutation_chain)
            {
                auto* chain_joint = chain_link->getParentJointModel();
                for(size_t ivar = chain_joint->getFirstVariableIndex(); ivar < chain_joint->getFirstVariableIndex() + chain_joint->getVariableCount(); ivar++)
                    temp_chain_mutation_var_mask[ivar] = 1;
            }

            chain_mutation_mask.resize(problem.active_variables.size());
            for(size_t igene = 0; igene < problem.active_variables.size(); igene++)
                chain_mutation_mask[igene] = temp_chain_mutation_var_mask[problem.active_variables[igene]];
        }
        */
    }

    /*
    const size_t chain_mutation_mask_count = 256;
    std::vector<std::vector<int>> chain_mutation_masks;
    std::vector<const moveit::core::LinkModel*> temp_mutation_chain;
    std::vector<int> temp_chain_mutation_var_mask;
    */

    // aligned_vector<double> rmask;

    // create offspring and mutate
    __attribute__((hot)) __attribute__((noinline))
    //__attribute__((target_clones("avx2", "avx", "sse2", "default")))
    //__attribute__((target("avx")))
    void
    reproduce(const std::vector<Individual>& population)
    {
        const auto __attribute__((aligned(32)))* __restrict__ genes_span = this->genes_span.data();
        const auto __attribute__((aligned(32)))* __restrict__ genes_min = this->genes_min.data();
        const auto __attribute__((aligned(32)))* __restrict__ genes_max = this->genes_max.data();

        auto gene_count = children[0].genes.size();

        size_t s = (children.size() - population.size()) * gene_count + children.size() * 4 + 4;

        auto* __restrict__ rr = fast_random_gauss_n(s);
        rr = (const double*)(((size_t)rr + 3) / 4 * 4);

        /*rmask.resize(s);
        for(auto& m : rmask) m = fast_random() < 0.1 ? 1.0 : 0.0;
        double* dm = rmask.data();*/

        for(size_t child_index = population.size(); child_index < children.size(); child_index++)
        {
            double mutation_rate = (1 << fast_random_index(16)) * (1.0 / (1 << 23));
            auto& parent = population[0];
            auto& parent2 = population[1];
            double fmix = (child_index % 2 == 0) * 0.2;
            double gradient_factor = child_index % 3;

            auto __attribute__((aligned(32)))* __restrict__ parent_genes = parent.genes.data();
            auto __attribute__((aligned(32)))* __restrict__ parent_gradients = parent.gradients.data();

            auto __attribute__((aligned(32)))* __restrict__ parent2_genes = parent2.genes.data();
            auto __attribute__((aligned(32)))* __restrict__ parent2_gradients = parent2.gradients.data();

            auto& child = children[child_index];

            auto __attribute__((aligned(32)))* __restrict__ child_genes = child.genes.data();
            auto __attribute__((aligned(32)))* __restrict__ child_gradients = child.gradients.data();

#pragma omp simd aligned(genes_span : 32), aligned(genes_min : 32), aligned(genes_max : 32), aligned(parent_genes : 32), aligned(parent_gradients : 32), aligned(parent2_genes : 32), aligned(parent2_gradients : 32), aligned(child_genes : 32), aligned(child_gradients : 32) aligned(rr : 32)
#pragma unroll
            for(size_t gene_index = 0; gene_index < gene_count; gene_index++)
            {
                // double mutation_rate = (1 << fast_random_index(16)) * (1.0 / (1 << 23));

                double r = rr[gene_index];
                // r *= dm[gene_index];
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
            // dm += (gene_count + 3) / 4 * 4;

            /*if(problem.tip_link_indices.size() > 1)
            {
                if(fast_random() < 0.5)
                {
                    auto& mask = chain_mutation_masks[fast_random_index(chain_mutation_mask_count)];
                    for(size_t gene_index = 0; gene_index < gene_count; gene_index++)
                    {
                        if(!mask[gene_index])
                        {
                            child_genes[gene_index] = parent_genes[gene_index];
                            child_gradients[gene_index] = parent_gradients[gene_index];
                        }
                    }
                }
            }*/

            for(auto quaternion_gene_index : quaternion_genes)
            {
                auto& qpos = (*(Quaternion*)&(children[child_index].genes[quaternion_gene_index]));
                normalizeFast(qpos);
            }
        }
    }

    void step()
    {
        FNPROFILER();

        for(size_t ispecies = 0; ispecies < species.size(); ispecies++)
        {
            auto& species = this->species[ispecies];
            auto& population = species.individuals;

            {
                BLOCKPROFILER("evolution");

                // initialize forward kinematics approximator
                genesToJointVariables(species.individuals[0], temp_joint_variables);
                {
                    BLOCKPROFILER("fk");
                    model.applyConfiguration(temp_joint_variables);
                    model.initializeMutationApproximator(problem.active_variables);
                }

                // run evolution for a few generations
                size_t generation_count = 16;
                if(memetic) generation_count = 8;
                for(size_t generation = 0; generation < generation_count; generation++)
                {
                    // BLOCKPROFILER("evolution");

                    if(canceled) break;

                    // reproduction
                    {
                        BLOCKPROFILER("reproduction");
                        reproduce(population);
                    }

                    size_t child_count = children.size();

                    // pre-selection by secondary objectives
                    if(problem.secondary_goals.size())
                    {
                        BLOCKPROFILER("pre-selection");
                        child_count = random_index(children.size() - population.size() - 1) + 1 + population.size();
                        for(size_t child_index = population.size(); child_index < children.size(); child_index++)
                        {
                            children[child_index].fitness = computeSecondaryFitnessActiveVariables(children[child_index].genes.data());
                        }
                        {
                            BLOCKPROFILER("pre-selection sort");
                            std::sort(children.begin() + population.size(), children.end(), [](const Individual& a, const Individual& b) { return a.fitness < b.fitness; });
                        }
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
                        model.computeApproximateMutations(child_count, genotypes.data(), phenotypes);
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
                                if(f < fmin) jmin = j, fmin = f;
                            }
                            std::swap(child_indices[i], child_indices[jmin]);
                        }
                        for(size_t i = 0; i < population.size(); i++)
                        {
                            std::swap(population[i].genes, children[child_indices[i]].genes);
                            std::swap(population[i].gradients, children[child_indices[i]].gradients);
                        }
                    }
                }
            }

            // memetic optimization
            {
                BLOCKPROFILER("memetics");

                if(memetic == 'q' || memetic == 'l')
                {

                    // init
                    auto& individual = population[0];
                    gradient.resize(problem.active_variables.size());
                    if(genotypes.empty()) genotypes.emplace_back();
                    phenotypes2.resize(1);
                    phenotypes3.resize(1);

                    // differentiation step size
                    double dp = 0.0000001;
                    if(fast_random() < 0.5) dp = -dp;

                    for(size_t generation = 0; generation < 8; generation++)
                    // for(size_t generation = 0; generation < 32; generation++)
                    {

                        if(canceled) break;

                        // compute gradient
                        temp = individual.genes;
                        genotypes[0] = temp.data();
                        model.computeApproximateMutations(1, genotypes.data(), phenotypes2);
                        double f2p = computeFitnessActiveVariables(phenotypes2[0], genotypes[0]);
                        double fa = f2p + computeSecondaryFitnessActiveVariables(genotypes[0]);
                        for(size_t i = 0; i < problem.active_variables.size(); i++)
                        {
                            double* pp = &(genotypes[0][i]);
                            genotypes[0][i] = individual.genes[i] + dp;
                            model.computeApproximateMutation1(problem.active_variables[i], +dp, phenotypes2[0], phenotypes3[0]);
                            double fb = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[0]);
                            genotypes[0][i] = individual.genes[i];
                            double d = fb - fa;
                            gradient[i] = d;
                        }

                        // normalize gradient
                        double sum = dp * dp;
                        for(size_t i = 0; i < problem.active_variables.size(); i++)
                            sum += fabs(gradient[i]);
                        double f = 1.0 / sum * dp;
                        for(size_t i = 0; i < problem.active_variables.size(); i++)
                            gradient[i] *= f;

                        // sample support points for line search
                        for(size_t i = 0; i < problem.active_variables.size(); i++)
                            genotypes[0][i] = individual.genes[i] - gradient[i];
                        model.computeApproximateMutations(1, genotypes.data(), phenotypes3);
                        double f1 = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[0]);

                        double f2 = fa;

                        for(size_t i = 0; i < problem.active_variables.size(); i++)
                            genotypes[0][i] = individual.genes[i] + gradient[i];
                        model.computeApproximateMutations(1, genotypes.data(), phenotypes3);
                        double f3 = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[0]);

                        // quadratic step size
                        if(memetic == 'q')
                        {

                            // compute step size
                            double v1 = (f2 - f1); // f / j
                            double v2 = (f3 - f2); // f / j
                            double v = (v1 + v2) * 0.5; // f / j
                            double a = (v1 - v2); // f / j^2
                            double step_size = v / a; // (f / j) / (f / j^2) = f / j / f * j * j = j

                            // double v1 = (f2 - f1) / dp;
                            // double v2 = (f3 - f2) / dp;
                            // double v = (v1 + v2) * 0.5;
                            // double a = (v2 - v1) / dp;
                            // // v * x + a * x * x = 0;
                            // // v = - a * x
                            // // - v / a = x
                            // // x = -v / a;
                            // double step_size = -v / a / dp;

                            // for(double f : { 1.0, 0.5, 0.25 })
                            {

                                double f = 1.0;

                                // move by step size along gradient and compute fitness
                                for(size_t i = 0; i < problem.active_variables.size(); i++)
                                    genotypes[0][i] = modelInfo.clip(individual.genes[i] + gradient[i] * step_size * f, problem.active_variables[i]);
                                model.computeApproximateMutations(1, genotypes.data(), phenotypes2);
                                double f4p = computeFitnessActiveVariables(phenotypes2[0], genotypes[0]);

                                // accept new position if better
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

                            // break;
                        }

                        // linear step size
                        if(memetic == 'l')
                        {

                            // compute step size
                            double cost_diff = (f3 - f1) * 0.5; // f / j
                            double step_size = f2 / cost_diff; // f / (f / j) = j

                            // move by step size along gradient and compute fitness
                            for(size_t i = 0; i < problem.active_variables.size(); i++)
                                temp[i] = modelInfo.clip(individual.genes[i] - gradient[i] * step_size, problem.active_variables[i]);
                            model.computeApproximateMutations(1, genotypes.data(), phenotypes2);
                            double f4p = computeFitnessActiveVariables(phenotypes2[0], genotypes[0]);

                            // accept new position if better
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

#ifdef ENABLE_CPP_OPTLIB
                // cppoptlib::LbfgsSolver memetic test
                if(memetic == 'o')
                {
                    auto& individual = population[0];

                    // create cppoptlib solver and cppoptlib problem, if not yet existing
                    if(!optlib_solver)
                    {
                        optlib_solver = std::make_shared<OptlibSolver>();
                        cppoptlib::Criteria<double> crit;
                        crit.iterations = 4;
                        optlib_solver->setStopCriteria(crit);
                        optlib_problem = std::make_shared<OptlibProblem>(this);
                    }

                    // init starting point
                    optlib_vector.resize(problem.active_variables.size());
                    for(size_t i = 0; i < problem.active_variables.size(); i++)
                        optlib_vector[i] = individual.genes[i];

                    // solve
                    optlib_solver->minimize(*optlib_problem, optlib_vector);

                    // get result
                    for(size_t i = 0; i < problem.active_variables.size(); i++)
                        individual.genes[i] = modelInfo.clip(optlib_vector[i], problem.active_variables[i]);
                }
#endif
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
            std::sort(species.begin(), species.end(), [](const Species& a, const Species& b) { return a.fitness < b.fitness; });

            // wipeouts
            for(size_t species_index = 1; species_index < species.size(); species_index++)
            {
                if(fast_random() < 0.1 || !species[species_index].improved)
                // if(fast_random() < 0.05 || !species[species_index].improved)
                {
                    {
                        auto& individual = species[species_index].individuals[0];

                        for(size_t i = 0; i < individual.genes.size(); i++)
                            individual.genes[i] = random(modelInfo.getMin(problem.active_variables[i]), modelInfo.getMax(problem.active_variables[i]));

                        for(auto& v : individual.gradients)
                            v = 0;
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

static IKFactory::Class<IKEvolution2<0>> bio2("bio2");
static IKFactory::Class<IKEvolution2<'q'>> bio2_memetic("bio2_memetic");
static IKFactory::Class<IKEvolution2<'l'>> bio2_memetic_l("bio2_memetic_l");

#ifdef ENABLE_CPP_OPTLIB
static IKFactory::Class<IKEvolution2<'o'>> bio2_memetic_0("bio2_memetic_lbfgs");
#endif
}
