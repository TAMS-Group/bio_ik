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

#pragma once

#include <vector>

#include <Eigen/Dense>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_kdl/tf2_kdl.h>

namespace bio_ik
{

typedef tf2::Quaternion Quaternion;
typedef tf2::Vector3 Vector3;

struct Frame
{
    Vector3 pos;
    double __padding[4 - (sizeof(Vector3) / sizeof(double))];
    Quaternion rot;
    inline Frame() {}
    inline Frame(const tf2::Vector3& pos, const tf2::Quaternion& rot)
        : pos(pos)
        , rot(rot)
    {
    }
    explicit inline Frame(const KDL::Frame& kdl)
    {
        pos = tf2::Vector3(kdl.p.x(), kdl.p.y(), kdl.p.z());
        double qx, qy, qz, qw;
        kdl.M.GetQuaternion(qx, qy, qz, qw);
        rot = tf2::Quaternion(qx, qy, qz, qw);
    }
    explicit inline Frame(const geometry_msgs::Pose& msg)
    {
        tf2::fromMsg(msg.orientation, rot);
        pos = tf2::Vector3(msg.position.x, msg.position.y, msg.position.z);
    }
    explicit inline Frame(const Eigen::Isometry3d& f)
    {
        pos = tf2::Vector3(f.translation().x(), f.translation().y(), f.translation().z());
        Eigen::Quaterniond q(f.rotation());
        rot = tf2::Quaternion(q.x(), q.y(), q.z(), q.w());
    }

    inline const Vector3& getPosition() const { return pos; }
    inline const Quaternion& getOrientation() const { return rot; }
    inline void setPosition(const Vector3& p) { pos = p; }
    inline void setOrientation(const Quaternion& q) { rot = q; }

private:
    template <size_t i> struct IdentityFrameTemplate
    {
        static const Frame identity_frame;
    };

public:
    static inline const Frame& identity() { return IdentityFrameTemplate<0>::identity_frame; }
};

inline void frameToKDL(const Frame& frame, KDL::Frame& kdl_frame)
{
    kdl_frame.p.x(frame.pos.x());
    kdl_frame.p.y(frame.pos.y());
    kdl_frame.p.z(frame.pos.z());
    kdl_frame.M = KDL::Rotation::Quaternion(frame.rot.x(), frame.rot.y(), frame.rot.z(), frame.rot.w());
}

template <size_t i> const Frame Frame::IdentityFrameTemplate<i>::identity_frame(Vector3(0, 0, 0), Quaternion(0, 0, 0, 1));

static std::ostream& operator<<(std::ostream& os, const Frame& f) { return os << "(" << f.pos.x() << "," << f.pos.y() << "," << f.pos.z() << ";" << f.rot.x() << "," << f.rot.y() << "," << f.rot.z() << "," << f.rot.w() << ")"; }

__attribute__((always_inline)) inline void quat_mul_vec(const tf2::Quaternion& q, const tf2::Vector3& v, tf2::Vector3& r)
{
    double v_x = v.x();
    double v_y = v.y();
    double v_z = v.z();

    // if(__builtin_expect(v_x == 0 && v_y == 0 && v_z == 0, 0)) { r = tf2::Vector3(0, 0, 0); return; }
    // if(v_x == 0 && v_y == 0 && v_z == 0) { r = tf2::Vector3(0, 0, 0); return; }

    double q_x = q.x();
    double q_y = q.y();
    double q_z = q.z();
    double q_w = q.w();

    if((v_x == 0 && v_y == 0 && v_z == 0) || (q_x == 0 && q_y == 0 && q_z == 0 && q_w == 1))
    {
        r = v;
        return;
    }
    // if((v_x + v_y + v_z == 0 && v_x == 0 && v_y == 0) || (q_x + q_y + q_z == 0 && q_x == 0 && q_y == 0 && q_w == 1)) { r = v; return; }
    // if(q_x == 0 && q_y == 0 && q_z == 0 && q_w == 1) { r = v; return; }

    double t_x = q_y * v_z - q_z * v_y;
    double t_y = q_z * v_x - q_x * v_z;
    double t_z = q_x * v_y - q_y * v_x;

    double r_x = q_w * t_x + q_y * t_z - q_z * t_y;
    double r_y = q_w * t_y + q_z * t_x - q_x * t_z;
    double r_z = q_w * t_z + q_x * t_y - q_y * t_x;

    r_x += r_x;
    r_y += r_y;
    r_z += r_z;

    r_x += v_x;
    r_y += v_y;
    r_z += v_z;

    r.setX(r_x);
    r.setY(r_y);
    r.setZ(r_z);
}

__attribute__((always_inline)) inline void quat_mul_quat(const tf2::Quaternion& p, const tf2::Quaternion& q, tf2::Quaternion& r)
{
    double p_x = p.x();
    double p_y = p.y();
    double p_z = p.z();
    double p_w = p.w();

    double q_x = q.x();
    double q_y = q.y();
    double q_z = q.z();
    double q_w = q.w();

    double r_x = (p_w * q_x + p_x * q_w) + (p_y * q_z - p_z * q_y);
    double r_y = (p_w * q_y - p_x * q_z) + (p_y * q_w + p_z * q_x);
    double r_z = (p_w * q_z + p_x * q_y) - (p_y * q_x - p_z * q_w);
    double r_w = (p_w * q_w - p_x * q_x) - (p_y * q_y + p_z * q_z);

    r.setX(r_x);
    r.setY(r_y);
    r.setZ(r_z);
    r.setW(r_w);
}

__attribute__((always_inline)) inline void concat(const Frame& a, const Frame& b, Frame& r)
{
    tf2::Vector3 d;
    quat_mul_vec(a.rot, b.pos, d);
    r.pos = a.pos + d;
    quat_mul_quat(a.rot, b.rot, r.rot);
}

__attribute__((always_inline)) inline void concat(const Frame& a, const Frame& b, const Frame& c, Frame& r)
{
    Frame tmp;
    concat(a, b, tmp);
    concat(tmp, c, r);
}

__attribute__((always_inline)) inline void quat_inv(const tf2::Quaternion& q, tf2::Quaternion& r)
{
    r.setX(-q.x());
    r.setY(-q.y());
    r.setZ(-q.z());
    r.setW(q.w());
}

__attribute__((always_inline)) inline void invert(const Frame& a, Frame& r)
{
    Frame tmp;
    quat_inv(a.rot, r.rot);
    quat_mul_vec(r.rot, -a.pos, r.pos);
}

__attribute__((always_inline)) inline void change(const Frame& a, const Frame& b, const Frame& c, Frame& r)
{
    Frame tmp;
    invert(b, tmp);
    concat(a, tmp, c, r);
}

__attribute__((always_inline)) inline Frame inverse(const Frame& f)
{
    Frame r;
    invert(f, r);
    return r;
}

__attribute__((always_inline)) inline Frame operator*(const Frame& a, const Frame& b)
{
    Frame r;
    concat(a, b, r);
    return r;
}

__attribute__((always_inline)) inline Frame& operator*=(Frame& a, const Frame& b)
{
    a = a * b;
    return a;
}

__attribute__((always_inline)) inline void normalizeFast(Quaternion& q)
{
    double f = (3.0 - q.length2()) * 0.5;
    q.setX(q.x() * f);
    q.setY(q.y() * f);
    q.setZ(q.z() * f);
    q.setW(q.w() * f);
}

__attribute__((always_inline)) inline KDL::Twist frameTwist(const Frame& a, const Frame& b)
{
    auto frame = inverse(a) * b;
    KDL::Twist t;
    t.vel.x(frame.pos.x());
    t.vel.y(frame.pos.y());
    t.vel.z(frame.pos.z());

    double ra = frame.rot.getAngle();
    // double ra = frame.rot.getAngleShortestPath();
    if(ra > +M_PI) ra -= 2 * M_PI;
    // if(ra < -M_PI) ra += 2 * M_PI;

    auto r = frame.rot.getAxis() * ra;
    t.rot.x(r.x());
    t.rot.y(r.y());
    t.rot.z(r.z());

    return t;
}
}
