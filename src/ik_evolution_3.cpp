// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#include "ik_base.h"

namespace bio_ik
{

// fast evolutionary inverse kinematics
template<int memetic, bool approx_fk>
struct IKEvolution3 : IKBase
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
    std::vector<aligned_vector<double>> genotype_buffer;
    std::vector<Frame> phenotype;
    std::vector<size_t> quaternion_genes;
    aligned_vector<double> genes_min, genes_max, genes_span;
    aligned_vector<double> gradient;
    std::vector<size_t> gene_subset, gene_subset_vars;

    IKEvolution3(const IKParams& p) : IKBase(p)
    {
    }

    void genesToJointVariables(const Individual& individual, std::vector<double>& variables)
    {
        auto& genes = individual.genes;
        variables.resize(params.robot_model->getVariableCount());
        for(size_t i = 0; i < problem.active_variables.size(); i++)
            variables[problem.active_variables[i]] = genes[i];
    }

    const std::vector<double>& getSolution() const
    {
        return solution;
    }

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
                //if(thread_index == 0) // on first island?
                //if(thread_index % 2 == 0) // on every second island...
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
        //if(genes_min.empty())
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
    }

    void reproduce(const std::vector<Individual>& population)
    {
        const auto __attribute__((aligned(32))) * __restrict__ genes_span = this->genes_span.data();
        const auto __attribute__((aligned(32))) * __restrict__ genes_min = this->genes_min.data();
        const auto __attribute__((aligned(32))) * __restrict__ genes_max = this->genes_max.data();

        auto gene_count = children[0].genes.size();

        for(size_t child_index = population.size(); child_index < children.size(); child_index++)
        {
            double mutation_rate = (1 << fast_random_index(16)) * (1.0 / (1 << 23));

            //double mutation_rate = 0.1;

            auto& parent = population[0];
            auto& parent2 = population[1];
            double fmix = (child_index % 2 == 0) * 0.2;
            double gradient_factor = child_index % 3;
            auto& child = children[child_index];

            child.genes = parent.genes;
            child.gradients = parent.gradients;

            if(fast_random() < 0.25)
            {

                double f = fast_random();
                f = clamp(f * 2.0, 0.0, 1.0);
                for(size_t gene_index : gene_subset)
                {
                    double diff = problem.initial_guess[problem.active_variables[gene_index]] - child.genes[gene_index];
                    diff *= f;
                    child.genes[gene_index] += diff;
                    child.gradients[gene_index] += mix(child.gradients[gene_index], diff, 0.3);
                }

            }
            else
            {

                for(size_t gene_index : gene_subset)
                {
                    double r = fast_random_gauss();
                    double f = mutation_rate * genes_span[gene_index];
                    double gene = parent.genes[gene_index];
                    double parent_gene = gene;
                    gene += r * f;
                    double parent_gradient = mix(parent.gradients[gene_index], parent2.gradients[gene_index], fmix);
                    double gradient = parent_gradient * gradient_factor;
                    gene += gradient;
                    gene = clamp(gene, genes_min[gene_index], genes_max[gene_index]);
                    child.genes[gene_index] = gene;
                    child.gradients[gene_index] = mix(parent_gradient, gene - parent_gene, 0.3);
                }

            }

            /*if(fast_random() < 0.25)
            {
                double f = fast_random();
                for(size_t gene_index : gene_subset_inv)
                {
                    double diff = problem.initial_guess[problem.active_variables[gene_index]] - child.genes[gene_index];
                    diff *= f;
                    child.genes[gene_index] += diff;
                    child.gradients[gene_index] += mix(child.gradients[gene_index], diff, 0.3);
                }
            }*/

            /*if(fast_random() < 0.25)
            {
                double f = fast_random();
                for(size_t gene_index : gene_subset)
                {
                    //if(fast_random() < 0.25)
                    {
                        double diff = problem.initial_guess[problem.active_variables[gene_index]] - child.genes[gene_index];
                        diff *= f;
                        child.genes[gene_index] += diff;
                        child.gradients[gene_index] += mix(child.gradients[gene_index], diff, 0.3);
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


    std::vector<const moveit::core::LinkModel*> temp_mutation_chain;
    std::vector<int> temp_chain_mutation_var_mask;

    void step()
    {
        FNPROFILER();




        /*gene_subset.resize(problem.active_variables.size());
        std::iota(gene_subset.begin(), gene_subset.end(), 0);
        std::random_shuffle(gene_subset.begin(), gene_subset.end());
        //std::reverse(gene_subset.begin(), gene_subset.end());
        gene_subset.resize(random_index(gene_subset.size() / 2) + 1);
        //gene_subset.resize(gene_subset.size() - 1);
        std::sort(gene_subset.begin(), gene_subset.end());*/


        if(random() < 0.5)
        {

            gene_subset.resize(problem.active_variables.size());
            std::iota(gene_subset.begin(), gene_subset.end(), 0);

        }
        else
        {

            temp_mutation_chain.clear();
            for(auto* chain_link = params.robot_model->getLinkModel(random_element(problem.tip_link_indices)); chain_link; chain_link = chain_link->getParentLinkModel())
                temp_mutation_chain.push_back(chain_link);

            if(random() < 0.25) std::reverse(temp_mutation_chain.begin(), temp_mutation_chain.end());

            temp_mutation_chain.resize(random_index(temp_mutation_chain.size()) + 1);
            //temp_mutation_chain.resize(temp_mutation_chain.size() - 1);

            temp_chain_mutation_var_mask.resize(params.robot_model->getVariableCount());
            for(auto& m : temp_chain_mutation_var_mask) m = 0;
            for(auto* chain_link : temp_mutation_chain)
            {
                auto* chain_joint = chain_link->getParentJointModel();
                for(size_t ivar = chain_joint->getFirstVariableIndex(); ivar < chain_joint->getFirstVariableIndex() + chain_joint->getVariableCount(); ivar++)
                    temp_chain_mutation_var_mask[ivar] = 1;
            }

            gene_subset.clear();
            //gene_subset_inv.clear();
            for(size_t igene = 0; igene < problem.active_variables.size(); igene++)
            {
                if(temp_chain_mutation_var_mask[problem.active_variables[igene]])
                    gene_subset.push_back(igene);
                //else
                //    gene_subset_inv.push_back(igene);
            }

            std::sort(gene_subset.begin(), gene_subset.end());
            //std::sort(gene_subset_inv.begin(), gene_subset_inv.end());

        }



        gene_subset_vars.clear();
        for(size_t i2 : gene_subset) gene_subset_vars.push_back(problem.active_variables[i2]);



        for(size_t ispecies = 0; ispecies < species.size(); ispecies++)
        {
            auto& species = this->species[ispecies];

            BLOCKPROFILER("evolution");

            auto& population = species.individuals;

            //for(auto& g : population[0].gradients) g = 0;

            for(size_t i = 1; i < population.size(); i++) population[i] = population[0];

            // initialize forward kinematics approximator
            {
                genesToJointVariables(population[0], temp_joint_variables);
                model.applyConfiguration(temp_joint_variables);
                model.initializeMutationApproximator(gene_subset_vars);
            }

            // run evolution for a few generations
            size_t generation_count = 16;
            if(memetic) generation_count = 8;
            for(size_t generation = 0; generation < generation_count; generation++)
            {

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
                        std::sort(
                                children.begin() + population.size(),
                                children.end(),
                                [] (const Individual& a, const Individual& b)
                                {
                                    return a.fitness < b.fitness;
                                }
                            );
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


                if(approx_fk)
                {

                    BLOCKPROFILER("phenotype & fitness");

                    // genotype-phenotype mapping
                    {
                        BLOCKPROFILER("phenotype");
                        genotypes.resize(child_count);
                        genotype_buffer.resize(child_count);
                        for(size_t child_index = 0; child_index < child_count; child_index++)
                        {
                            genotype_buffer[child_index].clear();
                            for(auto gene_index : gene_subset) genotype_buffer[child_index].push_back(children[child_index].genes[gene_index]);
                            genotypes[child_index] = genotype_buffer[child_index].data();
                        }
                        model.computeApproximateMutations(child_count, genotypes.data(), phenotypes);
                    }

                    // fitness
                    {
                        BLOCKPROFILER("fitness");
                        for(size_t child_index = 0; child_index < child_count; child_index++)
                        {
                            children[child_index].fitness = computeFitnessActiveVariables(phenotypes[child_index], children[child_index].genes.data());
                        }
                    }

                }
                else
                {

                    BLOCKPROFILER("phenotype & fitness");
                    for(size_t child_index = 0; child_index < child_count; child_index++)
                    {
                        genesToJointVariables(children[child_index], temp_joint_variables);
                        model.applyConfiguration(temp_joint_variables);
                        children[child_index].fitness = computeFitnessActiveVariables(model.getTipFrames(), children[child_index].genes.data());
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
            }


            if(memetic && !approx_fk)
            {
                genesToJointVariables(population[0], temp_joint_variables);
                model.applyConfiguration(temp_joint_variables);
                model.initializeMutationApproximator(gene_subset_vars);
            }


            // memetic optimization
            {
                if(memetic == 'q' || memetic == 'l')
                {

                    // init
                    auto& individual = population[0];
                    gradient.resize(gene_subset.size());
                    if(genotypes.empty()) genotypes.resize(2);
                    if(genotype_buffer.empty()) genotype_buffer.resize(2);
                    for(size_t i = 0; i < 2; i++)
                    {
                        genotype_buffer[i].resize(problem.active_variables.size());
                        genotypes[i] = genotype_buffer[i].data();
                    }
                    phenotypes2.resize(1);
                    phenotypes3.resize(1);

                    // differentiation step size
                    double dp = 0.0000001;
                    if(fast_random() < 0.5) dp = -dp;

                    for(size_t generation = 0; generation < 32; generation++)
                    {

                        // compute gradient
                        genotype_buffer[0].resize(gene_subset.size());
                        genotype_buffer[1].resize(problem.active_variables.size());
                        for(size_t i2 = 0; i2 < gene_subset.size(); i2++) genotype_buffer[0][i2] = individual.genes[gene_subset[i2]];
                        genotype_buffer[1] = individual.genes;
                        genotypes[0] = genotype_buffer[0].data();
                        genotypes[1] = genotype_buffer[1].data();
                        model.computeApproximateMutations(1, genotypes.data(), phenotypes2);
                        double f2p = computeFitnessActiveVariables(phenotypes2[0], genotypes[1]);
                        double fa = f2p + computeSecondaryFitnessActiveVariables(genotypes[1]);
                        for(size_t j = 0; j < gene_subset.size(); j++)
                        {
                            auto i = gene_subset[j];
                            genotypes[0][j] = individual.genes[i] + dp;
                            genotypes[1][i] = individual.genes[i] + dp;
                            model.computeApproximateMutation1(problem.active_variables[i], +dp, phenotypes2[0], phenotypes3[0]);
                            double fb = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[1]);
                            genotypes[0][j] = individual.genes[i];
                            genotypes[1][i] = individual.genes[i];
                            double d = fb - fa;
                            gradient[j] = d;
                        }

                        // normalize gradient
                        double sum = (dp * dp) * (dp * dp);
                        for(size_t i = 0; i < gradient.size(); i++)
                            sum += gradient[i] * gradient[i];
                        double f = 1.0 / std::sqrt(sum) * dp;
                        for(size_t i = 0; i < gradient.size(); i++)
                            gradient[i] *= f;

                        // sample support points for line search
                        for(size_t i = 0; i < gene_subset.size(); i++)
                        {
                            genotypes[0][i] = individual.genes[gene_subset[i]] - gradient[i];
                            genotypes[1][gene_subset[i]] = genotypes[0][i];
                        }
                        model.computeApproximateMutations(1, genotypes.data(), phenotypes3);
                        double f1 = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[1]);

                        double f2 = fa;

                        for(size_t i = 0; i < gene_subset.size(); i++)
                        {
                            genotypes[0][i] = individual.genes[gene_subset[i]] + gradient[i];
                            genotypes[1][gene_subset[i]] = genotypes[0][i];
                        }
                        model.computeApproximateMutations(1, genotypes.data(), phenotypes3);
                        double f3 = computeCombinedFitnessActiveVariables(phenotypes3[0], genotypes[1]);

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

                            //for(double f : { 1.0, 0.5, 0.25 })
                            {

                                double f = 1.0;

                                // move by step size along gradient and compute fitness
                                for(size_t i = 0; i < gene_subset.size(); i++)
                                {
                                    genotypes[0][i] = modelInfo.clip(individual.genes[gene_subset[i]] + gradient[i] * step_size * f, problem.active_variables[gene_subset[i]]);
                                    genotypes[1][gene_subset[i]] = genotypes[0][i];
                                }
                                model.computeApproximateMutations(1, genotypes.data(), phenotypes2);
                                double f4p = computeFitnessActiveVariables(phenotypes2[0], genotypes[1]);


                                // accept new position if better
                                if(f4p < f2p)
                                {
                                    individual.genes = genotype_buffer[1];
                                    continue;
                                }
                                else
                                {
                                    break;
                                }

                            }

                            //break;

                        }

                    }
                }
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
                //if(fast_random() < 0.05 || !species[species_index].improved)
                {
                    {
                        auto& individual = species[species_index].individuals[0];

                        switch(random_index(2))
                        {
                            case 0:
                            {
                                for(size_t i = 0; i < individual.genes.size(); i++)
                                    individual.genes[i] = random(modelInfo.getMin(problem.active_variables[i]), modelInfo.getMax(problem.active_variables[i]));
                                break;
                            }
                            case 1:
                            {
                                for(size_t i = 0; i < individual.genes.size(); i++)
                                    individual.genes[i] = problem.initial_guess[problem.active_variables[i]];
                                break;
                            }
                        }

                        for(auto& v : individual.gradients) v = 0;
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

static IKFactory::Class<IKEvolution3<0, 1>> bio3("bio3");
static IKFactory::Class<IKEvolution3<'q', 1>> bio3_memetic("bio3_memetic");
static IKFactory::Class<IKEvolution3<'l', 1>> bio3_memetic_l("bio3_memetic_l");

static IKFactory::Class<IKEvolution3<0, 0>> bio3b("bio3b");
static IKFactory::Class<IKEvolution3<'q', 0>> bio3b_memetic("bio3b_memetic");
static IKFactory::Class<IKEvolution3<'l', 0>> bio3b_memetic_l("bio3b_memetic_l");

}
