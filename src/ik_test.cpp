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

namespace bio_ik
{

struct IKTest : IKBase
{

    RobotFK_MoveIt fkref;

    std::vector<double> temp;

    double d_rot_sum, d_pos_sum, d_div;

    IKTest(const IKParams& params)
        : IKBase(params)
        , fkref(params.robot_model)
    {
        d_rot_sum = d_pos_sum = d_div = 0;
    }

    /*double tipdiff(const std::vector<Frame>& fa, const std::vector<Frame>& fb)
    {
        double diff = 0.0;
        for(size_t i = 0; i < problem.tip_link_indices.size(); i++)
        {
            //LOG_VAR(fa[i]);
            //LOG_VAR(fb[i]);
            diff += fa[i].rot.angleShortestPath(fb[i].rot);
            diff += fa[i].pos.distance(fb[i].pos);
        }
        return diff;
    }*/

    void initialize(const Problem& problem)
    {
        IKBase::initialize(problem);

        fkref.initialize(problem.tip_link_indices);
        model.initialize(problem.tip_link_indices);

        fkref.applyConfiguration(problem.initial_guess);
        model.applyConfiguration(problem.initial_guess);

        // double diff = tipdiff(fkref.getTipFrames(), model.getTipFrames());
        // LOG_VAR(diff);

        /*{
            auto& fa = fkref.getTipFrames();
            auto& fb = model.getTipFrames();
            for(size_t i = 0; i < problem.tip_link_indices.size(); i++)
            {
                LOG("d rot", i, fa[i].rot.angleShortestPath(fb[i].rot));
                LOG("d pos", i, fa[i].pos.distance(fb[i].pos));
            }
        }*/

        {
            temp = problem.initial_guess;
            for(size_t ivar : problem.active_variables)
                if(modelInfo.isRevolute(ivar) || modelInfo.isPrismatic(ivar)) temp[ivar] = modelInfo.clip(temp[ivar] + random(-0.1, 0.1), ivar);

            fkref.applyConfiguration(temp);
            auto& fa = fkref.getTipFrames();

            model.applyConfiguration(problem.initial_guess);
            model.initializeMutationApproximator(problem.active_variables);

            std::vector<aligned_vector<Frame>> fbm;

            std::vector<double> mutation_values;
            for(size_t ivar : problem.active_variables)
                mutation_values.push_back(temp[ivar]);
            const double* mutation_ptr = mutation_values.data();

            model.computeApproximateMutations(1, &mutation_ptr, fbm);

            auto& fb = fbm[0];

            // auto& fb = model.getTipFrames();

            for(size_t i = 0; i < problem.tip_link_indices.size(); i++)
            {
                // LOG("d rot", i, fa[i].rot.angleShortestPath(fb[i].rot));
                // LOG("d pos", i, fa[i].pos.distance(fb[i].pos));

                d_rot_sum += fa[i].rot.angleShortestPath(fb[i].rot);
                d_pos_sum += fa[i].pos.distance(fb[i].pos);
                d_div += 1;
            }
        }

        LOG("d rot avg", d_rot_sum / d_div);
        LOG("d pos avg", d_pos_sum / d_div);
    }

    void step() {}

    const std::vector<double>& getSolution() const { return problem.initial_guess; }
};

static IKFactory::Class<IKTest> test("test");
}
