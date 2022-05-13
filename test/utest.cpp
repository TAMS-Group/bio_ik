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
 *   * Neither the name of the copyright holder nor the names of its
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

// BioIK unit tests

#include <gtest/gtest.h>
#include <random>

#include "../src/utils.h"
#include <bio_ik/frame.h>

using namespace bio_ik;

#define FRAME_EXPECT_NEAR(r, s) \
    EXPECT_NEAR(r.rot.getX(), s.rot.getX(), 0.001); \
    EXPECT_NEAR(r.rot.getY(), s.rot.getY(), 0.001); \
    EXPECT_NEAR(r.rot.getZ(), s.rot.getZ(), 0.001); \
    EXPECT_NEAR(r.rot.getW(), s.rot.getW(), 0.001); \
    EXPECT_NEAR(r.pos.getX(), s.pos.getX(), 0.001); \
    EXPECT_NEAR(r.pos.getY(), s.pos.getY(), 0.001); \
    EXPECT_NEAR(r.pos.getZ(), s.pos.getZ(), 0.001);
    
template<class RNG>
Frame random_frame(RNG& rng)
{
    std::uniform_real_distribution<double> dist(-1, 1);
    tf2::Quaternion q(dist(rng), dist(rng), dist(rng), dist(rng));
    q.normalize();
    tf2::Vector3 v(dist(rng), dist(rng), dist(rng));
    return Frame(v, q);
}

TEST(BioIK, change)
{
    std::mt19937 rng(0);

    for(int i = 0; i < 10000; i++)
    {
        Frame fa = random_frame(rng);
        Frame fb = random_frame(rng);
        Frame fc = random_frame(rng);
        Frame fx, fy, fz;
        
        concat(fb, fa, fx);
        concat(fc, fa, fy);
        
        change(fc, fb, fx, fz);
        
        FRAME_EXPECT_NEAR(fy, fz);
    }
}

TEST(BioIK, linear_int_distribution)
{
    std::mt19937 rng(0);
    
    int n = 8;
    std::vector<double> v(n);
    
    linear_int_distribution<size_t> d(n);
    int iter = 1000000;
    for(int i = 0; i < iter; i++)
        v[d(rng)]++;
    
    std::vector<double> r(n);
    for(int i = 0; i < n; i++) r[i] = n - i;
    
    for(auto* vp : { &v, &r })
    {
        auto& v = *vp;
        double s = 0;
        for(int i = 0; i < n; i++) s += v[i];
        for(int i = 0; i < n; i++) v[i] /= s;
    }
    
    for(int i = 0; i < n; i++)
    {
        //LOG_ALWAYS(i, v[i], r[i]);
        EXPECT_NEAR(v[i], r[i], 0.001);
    }
}

int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}



