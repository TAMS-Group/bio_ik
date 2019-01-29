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

#include <fann.h>
#include <fann_cpp.h>

#include <mutex>

#define LOG_LIST(l)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                \
    {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              \
        LOG(#l "[]");                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              \
        for(std::size_t i = 0; i < l.size(); i++)                                                                                                                                                                                                                                                                                                                                                                                                                                                                  \
        {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          \
            LOG(#l "[" + std::to_string(i) + "]", l[i]);                                                                                                                                                                                                                                                                                                                                                                                                                                                           \
        }                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          \
    }

namespace bio_ik
{

/*
static inline KDL::Twist toTwist(const Frame& frame)
{
    KDL::Twist t;
    t.vel.x(frame.pos.x());
    t.vel.y(frame.pos.y());
    t.vel.z(frame.pos.z());
    auto r = frame.rot.getAxis() * frame.rot.getAngle();
    t.rot.x(r.x());
    t.rot.y(r.y());
    t.rot.z(r.z());
    return t;
}

static inline KDL::Twist frameTwist(const Frame& a, const Frame& b)
{
    auto frame = inverse(a) * b;
    KDL::Twist t;
    t.vel.x(frame.pos.x());
    t.vel.y(frame.pos.y());
    t.vel.z(frame.pos.z());
    auto r = frame.rot.getAxis() * frame.rot.getAngle();
    t.rot.x(r.x());
    t.rot.y(r.y());
    t.rot.z(r.z());
    return t;
}
*/

struct IKNeural : IKBase
{
    std::vector<double> solution;
    FANN::neural_net net;

    std::vector<std::pair<fann_type, fann_type>> input_minmax, output_minmax;

    static void find_minmax(const std::vector<fann_type>& values, std::vector<std::pair<fann_type, fann_type>>& minmax)
    {
        for(size_t i = 0; i < minmax.size(); i++)
        {
            minmax[i] = std::make_pair(values[i], values[i]);
        }
        for(size_t i = 0; i < values.size(); i++)
        {
            auto& v = values[i];
            auto& p = minmax[i % minmax.size()];
            p.first = std::min(p.first, v);
            p.second = std::max(p.second, v);
        }
    }

    static void normalize(std::vector<fann_type>& values, const std::vector<std::pair<fann_type, fann_type>>& minmax)
    {
        for(size_t i = 0; i < values.size(); i++)
        {
            auto& v = values[i];
            auto& p = minmax[i % minmax.size()];
            v = (v - p.first) / (p.second - p.first);
        }
    }

    static void denormalize(std::vector<fann_type>& values, const std::vector<std::pair<fann_type, fann_type>>& minmax)
    {
        for(size_t i = 0; i < values.size(); i++)
        {
            auto& v = values[i];
            auto& p = minmax[i % minmax.size()];
            v = v * (p.second - p.first) + p.first;
        }
    }

    unsigned int input_count, output_count;

    IKNeural(const IKParams& p)
        : IKBase(p)
    {
        trained = false;
    }

    bool trained;

    void train()
    {
        trained = true;

        input_count = problem.active_variables.size() + problem.tip_link_indices.size() * 6;
        output_count = problem.active_variables.size();

        LOG_VAR(input_count);
        LOG_VAR(output_count);

        // std::vector<unsigned int> levels = {input_count, input_count, input_count, output_count};

        // std::vector<unsigned int> levels = {input_count, input_count + output_count, output_count};

        // std::vector<unsigned int> levels = {input_count, input_count + output_count, input_count + output_count, output_count};

        // std::vector<unsigned int> levels = {input_count, 100, output_count};

        std::vector<unsigned int> levels = {input_count, 50, output_count};

        net.create_standard_array(levels.size(), levels.data());

        size_t var_count = params.robot_model->getVariableCount();
        std::vector<double> state1(var_count), state2(var_count);
        std::vector<Frame> frames1, frames2;

        std::vector<fann_type> inputs, outputs;
        std::vector<fann_type*> input_pp, output_pp;

        LOG("neuro ik generating training data");

        unsigned int samples = 10000;

        for(size_t iter = 0; iter < samples; iter++)
        {
            for(size_t ivar = 0; ivar < var_count; ivar++)
            {
                state1[ivar] = random(modelInfo.getMin(ivar), modelInfo.getMax(ivar));
                state1[ivar] = modelInfo.clip(state1[ivar], ivar);
                // state2[ivar] = modelInfo.clip(state1[ivar] + random_gauss() * modelInfo.getSpan(ivar), ivar);
                state2[ivar] = modelInfo.clip(state1[ivar] + random_gauss() * 0.1, ivar);
            }

            model.applyConfiguration(state1);
            frames1 = model.getTipFrames();
            model.applyConfiguration(state2);
            frames2 = model.getTipFrames();

            for(auto ivar : problem.active_variables)
            {
                inputs.push_back(state1[ivar]);
                outputs.push_back(state2[ivar] - state1[ivar]);
            }

            for(size_t itip = 0; itip < problem.tip_link_indices.size(); itip++)
            {
                double translational_scale = 1.0;
                double rotational_scale = 1.0;

                // Frame diff = inverse(frames1[itip]) * frames2[itip];
                // auto twist = toTwist(diff);
                auto twist = frameTwist(frames1[itip], frames2[itip]);

                inputs.push_back(frames2[itip].pos.x() - frames1[itip].pos.x());
                inputs.push_back(frames2[itip].pos.y() - frames1[itip].pos.y());
                inputs.push_back(frames2[itip].pos.z() - frames1[itip].pos.z());

                inputs.push_back(twist.rot.x() * rotational_scale);
                inputs.push_back(twist.rot.y() * rotational_scale);
                inputs.push_back(twist.rot.z() * rotational_scale);
            }
        }

        for(auto& v : inputs)
            if(!std::isfinite(v)) throw std::runtime_error("NAN");
        for(auto& v : outputs)
            if(!std::isfinite(v)) throw std::runtime_error("NAN");

        input_minmax.resize(input_count);
        output_minmax.resize(output_count);

        find_minmax(inputs, input_minmax);
        find_minmax(outputs, output_minmax);

        normalize(inputs, input_minmax);
        normalize(outputs, output_minmax);

        for(size_t iter = 0; iter < samples; iter++)
        {
            input_pp.push_back(inputs.data() + iter * input_count);
            output_pp.push_back(outputs.data() + iter * output_count);
        }

        LOG("neuro ik training");

        FANN::training_data train;
        train.set_train_data(samples, input_count, input_pp.data(), output_count, output_pp.data());
        net.set_callback(
            [](FANN::neural_net& net, FANN::training_data& train, unsigned int max_epochs, unsigned int epochs_between_reports, float desired_error, unsigned int epochs, void* user_data) {
                if(epochs % epochs_between_reports != 0) return 0;
                // LOG("training", epochs, "/", max_epochs, epochs * 100 / max_epochs, "%");
                LOG("training", epochs, net.get_MSE(), desired_error);
                return 0;
            },
            0);

        net.set_activation_function_hidden(FANN::SIGMOID);
        net.set_activation_function_output(FANN::SIGMOID);

        net.init_weights(train);

        net.train_on_data(train, 1000, 1, 0.0001);

        fann_type err = net.test_data(train);
        LOG("neuro ik training error:", err);

        /*std::vector<fann_type> iiv, oov, ttv;
        for(size_t iter = 0; iter < 10; iter++)
        {
            auto* ii = input_pp[iter];
            auto* oo = net.run(ii);
            auto* tt = output_pp[iter];
            iiv.assign(ii, ii + input_count);
            ttv.assign(tt, tt + output_count);
            oov.assign(oo, oo + output_count);
            LOG_LIST(iiv);
            LOG_LIST(ttv);
            LOG_LIST(oov);
        }*/

        LOG("training done");
    }

    size_t iterations = 0;

    void initialize(const Problem& problem)
    {
        static std::mutex mutex;
        std::lock_guard<std::mutex> lock(mutex);
        IKBase::initialize(problem);
        solution = problem.initial_guess;
        if(!trained) train();
        iterations = 0;
        if(thread_index > 0)
            for(auto& vi : problem.active_variables)
                solution[vi] = random(modelInfo.getMin(vi), modelInfo.getMax(vi));
    }

    const std::vector<double>& getSolution() const { return solution; }

    std::vector<fann_type> inputs, outputs;

    std::vector<Frame> tip_objectives;

    /*void step()
    {
        //if(iterations > 1) return;
        iterations++;

        inputs.clear();
        for(auto ivar : problem.active_variables)
        {
            inputs.push_back(solution[ivar]);
        }

        tip_objectives.resize(model.getTipFrames().size());
        for(auto& g : problem.goals)
        {
            tip_objectives[g.tip_index] = g.frame;
        }

        model.applyConfiguration(solution);
        auto& frames1 = model.getTipFrames();
        auto& frames2 = tip_objectives;

        double scale = 1.0;

        for(size_t itip = 0; itip < tip_objectives.size(); itip++)
        {
            double translational_scale = 1.0;
            double rotational_scale = 1.0;

            //Frame diff = inverse(frames1[itip]) * frames2[itip];
            //auto twist = toTwist(diff);
            auto twist = frameTwist(frames1[itip], frames2[itip]);

            auto dpos = frames2[itip].pos - frames1[itip].pos;
            auto drot = Vector3(
                twist.rot.x() * rotational_scale,
                twist.rot.y() * rotational_scale,
                twist.rot.z() * rotational_scale
            );

            scale = 1.0 / (0.0000001 + dpos.length() + drot.length());

            dpos = dpos * scale;
            drot = drot * scale;

            inputs.push_back(dpos.x());
            inputs.push_back(dpos.y());
            inputs.push_back(dpos.z());

            inputs.push_back(drot.x());
            inputs.push_back(drot.y());
            inputs.push_back(drot.z());
        }

        normalize(inputs, input_minmax);

        auto* oo = net.run(inputs.data());

        outputs.assign(oo, oo + output_count);

        denormalize(outputs, output_minmax);

        auto& vv = problem.active_variables;
        for(size_t iout = 0; iout < vv.size(); iout++)
        {
            size_t ivar = vv[iout];
            solution[ivar] = modelInfo.clip(solution[ivar] + outputs[iout] * 0.1 / scale, ivar);
        }
    }*/

    void step()
    {
        // if(iterations > 1) return;
        iterations++;

        inputs.clear();
        for(auto ivar : problem.active_variables)
        {
            inputs.push_back(solution[ivar]);
        }

        tip_objectives.resize(model.getTipFrames().size());
        for(auto& g : problem.goals)
        {
            tip_objectives[g.tip_index] = g.frame;
        }

        model.applyConfiguration(solution);
        auto& frames1 = model.getTipFrames();
        auto& frames2 = tip_objectives;

        double scale = 1.0;
        for(size_t itip = 0; itip < tip_objectives.size(); itip++)
        {
            double translational_scale = 1.0;
            double rotational_scale = 1.0;
            auto twist = frameTwist(frames1[itip], frames2[itip]);
            auto dpos = frames2[itip].pos - frames1[itip].pos;
            auto drot = Vector3(twist.rot.x() * rotational_scale, twist.rot.y() * rotational_scale, twist.rot.z() * rotational_scale);

            /*if(iterations % 2)
            {
                scale = 1.0 / (0.0000001 + dpos.length());
                inputs.push_back(dpos.x() * scale);
                inputs.push_back(dpos.y() * scale);
                inputs.push_back(dpos.z() * scale);
                inputs.push_back(0);
                inputs.push_back(0);
                inputs.push_back(0);
            } else {
                scale = 1.0 / (0.0000001 + drot.length());
                inputs.push_back(0);
                inputs.push_back(0);
                inputs.push_back(0);
                inputs.push_back(drot.x() * scale);
                inputs.push_back(drot.y() * scale);
                inputs.push_back(drot.z() * scale);
            }*/

            {
                scale = 1.0 / (0.0000001 + dpos.length() + drot.length());
                inputs.push_back(dpos.x() * scale);
                inputs.push_back(dpos.y() * scale);
                inputs.push_back(dpos.z() * scale);
                inputs.push_back(drot.x() * scale);
                inputs.push_back(drot.y() * scale);
                inputs.push_back(drot.z() * scale);
            }
        }
        normalize(inputs, input_minmax);
        auto* oo = net.run(inputs.data());
        outputs.assign(oo, oo + output_count);
        denormalize(outputs, output_minmax);
        auto& vv = problem.active_variables;
        for(size_t iout = 0; iout < vv.size(); iout++)
        {
            size_t ivar = vv[iout];
            solution[ivar] = modelInfo.clip(solution[ivar] + outputs[iout] * 1 / scale, ivar);
        }
    }
};

static IKFactory::Class<IKNeural> neural("neural");

struct IKNeural2 : IKBase
{
    std::vector<double> solution;
    FANN::neural_net net;

    std::vector<std::pair<fann_type, fann_type>> input_minmax, output_minmax;

    /*static void find_minmax(const std::vector<fann_type>& values, std::vector<std::pair<fann_type, fann_type>>& minmax)
    {
        for(size_t i = 0; i < minmax.size(); i++)
        {
            minmax[i] = std::make_pair(values[i], values[i]);
        }
        for(size_t i = 0; i < values.size(); i++)
        {
            auto& v = values[i];
            auto& p = minmax[i % minmax.size()];
            p.first = std::min(p.first, v);
            p.second = std::max(p.second, v);
        }
    }*/

    static void find_minmax(const std::vector<fann_type>& values, std::vector<std::pair<fann_type, fann_type>>& minmax)
    {
        std::vector<double> centers(minmax.size(), 0.0);
        for(size_t i = 0; i < values.size(); i++)
            centers[i % minmax.size()] += values[i] * (1.0 * minmax.size() / values.size());

        std::vector<double> ranges2(minmax.size(), 0.0001);
        for(size_t i = 0; i < values.size(); i++)
        {
            double d = values[i] - centers[i % minmax.size()];
            d = d * d;
            ranges2[i % minmax.size()] += d * (1.0 * minmax.size() / values.size());
        }

        for(size_t i = 0; i < minmax.size(); i++)
        {
            auto& p = minmax[i];
            p.first = centers[i] - sqrt(ranges2[i]);
            p.second = centers[i] + sqrt(ranges2[i]);
        }
    }

    static void normalize(std::vector<fann_type>& values, const std::vector<std::pair<fann_type, fann_type>>& minmax)
    {
        for(size_t i = 0; i < values.size(); i++)
        {
            auto& v = values[i];
            auto& p = minmax[i % minmax.size()];
            v = (v - p.first) / (p.second - p.first);
        }
    }

    static void denormalize(std::vector<fann_type>& values, const std::vector<std::pair<fann_type, fann_type>>& minmax)
    {
        for(size_t i = 0; i < values.size(); i++)
        {
            auto& v = values[i];
            auto& p = minmax[i % minmax.size()];
            v = v * (p.second - p.first) + p.first;
        }
    }

    unsigned int input_count, output_count;

    IKNeural2(const IKParams& p)
        : IKBase(p)
    {
        trained = false;
    }

    bool trained;

    void train()
    {
        trained = true;

        input_count = problem.tip_link_indices.size() * 7;
        output_count = problem.active_variables.size();

        LOG_VAR(input_count);
        LOG_VAR(output_count);

        // std::vector<unsigned int> levels = {input_count, 100, 100, output_count};

        // std::vector<unsigned int> levels = {input_count, input_count, input_count, output_count};

        std::vector<unsigned int> levels = {input_count, input_count + output_count, output_count};

        // std::vector<unsigned int> levels = {input_count, input_count + output_count, input_count + output_count, output_count};

        // std::vector<unsigned int> levels = {input_count, output_count};

        net.create_standard_array(levels.size(), levels.data());

        size_t var_count = params.robot_model->getVariableCount();
        std::vector<double> state = problem.initial_guess;
        std::vector<Frame> frames;

        std::vector<fann_type> inputs, outputs;
        std::vector<fann_type*> input_pp, output_pp;

        LOG("neuro ik generating training data");

        unsigned int samples = 10000;

        for(size_t iter = 0; iter < samples; iter++)
        {
            for(size_t ivar : problem.active_variables)
                state[ivar] = random(modelInfo.getMin(ivar), modelInfo.getMax(ivar));

            model.applyConfiguration(state);
            frames = model.getTipFrames();

            for(auto ivar : problem.active_variables)
                outputs.push_back(state[ivar]);

            for(size_t itip = 0; itip < problem.tip_link_indices.size(); itip++)
            {
                inputs.push_back(frames[itip].pos.x());
                inputs.push_back(frames[itip].pos.y());
                inputs.push_back(frames[itip].pos.z());

                auto rot = frames[itip].rot;
                rot = rot * rot;
                // rot = tf2::Quaternion(0, 0, 0, 1);
                inputs.push_back(rot.x());
                inputs.push_back(rot.y());
                inputs.push_back(rot.z());
                inputs.push_back(rot.w());
            }
        }

        for(auto& v : inputs)
            if(!std::isfinite(v)) throw std::runtime_error("NAN");
        for(auto& v : outputs)
            if(!std::isfinite(v)) throw std::runtime_error("NAN");

        input_minmax.resize(input_count);
        output_minmax.resize(output_count);

        find_minmax(inputs, input_minmax);
        find_minmax(outputs, output_minmax);

        normalize(inputs, input_minmax);
        normalize(outputs, output_minmax);

        for(size_t iter = 0; iter < samples; iter++)
        {
            input_pp.push_back(inputs.data() + iter * input_count);
            output_pp.push_back(outputs.data() + iter * output_count);
        }

        LOG("neuro ik training");

        FANN::training_data train;
        train.set_train_data(samples, input_count, input_pp.data(), output_count, output_pp.data());
        net.set_callback(
            [](FANN::neural_net& net, FANN::training_data& train, unsigned int max_epochs, unsigned int epochs_between_reports, float desired_error, unsigned int epochs, void* user_data) {
                if(epochs % epochs_between_reports != 0) return 0;
                // LOG("training", epochs, "/", max_epochs, epochs * 100 / max_epochs, "%");
                LOG("training", epochs, net.get_MSE(), desired_error);
                return 0;
            },
            0);

        net.set_activation_function_hidden(FANN::SIGMOID);
        net.set_activation_function_output(FANN::SIGMOID);

        net.init_weights(train);

        net.train_on_data(train, 100, 1, 0.0001);

        fann_type err = net.test_data(train);
        LOG("neuro ik training error:", err);

        /*std::vector<fann_type> iiv, oov, ttv;
        for(size_t iter = 0; iter < 10; iter++)
        {
            auto* ii = input_pp[iter];
            auto* oo = net.run(ii);
            auto* tt = output_pp[iter];
            iiv.assign(ii, ii + input_count);
            ttv.assign(tt, tt + output_count);
            oov.assign(oo, oo + output_count);
            LOG_LIST(iiv);
            LOG_LIST(ttv);
            LOG_LIST(oov);
        }*/

        LOG("training done");
    }

    size_t iterations = 0;

    void initialize(const Problem& problem)
    {
        IKBase::initialize(problem);
        solution = problem.initial_guess;
        if(!trained) train();
        iterations = 0;
    }

    const std::vector<double>& getSolution() const { return solution; }

    std::vector<fann_type> inputs, outputs;

    std::vector<Frame> tip_objectives;

    void step()
    {
        if(iterations > 1) return;
        iterations++;

        inputs.clear();

        tip_objectives.resize(model.getTipFrames().size());
        for(auto& g : problem.goals)
        {
            tip_objectives[g.tip_index] = g.frame;
        }

        auto& frames = tip_objectives;

        for(size_t itip = 0; itip < tip_objectives.size(); itip++)
        {
            inputs.push_back(frames[itip].pos.x());
            inputs.push_back(frames[itip].pos.y());
            inputs.push_back(frames[itip].pos.z());

            auto rot = frames[itip].rot;
            rot = rot * rot;
            // rot = tf2::Quaternion(0, 0, 0, 1);
            inputs.push_back(rot.x());
            inputs.push_back(rot.y());
            inputs.push_back(rot.z());
            inputs.push_back(rot.w());
        }

        normalize(inputs, input_minmax);

        auto* oo = net.run(inputs.data());

        outputs.assign(oo, oo + output_count);

        denormalize(outputs, output_minmax);

        auto& vv = problem.active_variables;
        for(size_t iout = 0; iout < vv.size(); iout++)
        {
            size_t ivar = vv[iout];
            solution[ivar] = modelInfo.clip(outputs[iout], ivar);
        }
    }
};

static IKFactory::Class<IKNeural2> neural2("neural2");
}
