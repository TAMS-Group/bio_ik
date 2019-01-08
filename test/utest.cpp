// BioIK unit tests

#include <gtest/gtest.h>

#include "../src/utils.h"
#include "../src/frame.h"

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



