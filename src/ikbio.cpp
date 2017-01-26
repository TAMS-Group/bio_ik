// Bio IK for ROS
// Philipp Ruppel

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <urdf/model.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf_model/model.h>
#include <srdfdom/model.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <eigen_conversions/eigen_msg.h>

#include <random>
#include <type_traits>
#include <atomic>




using namespace std;
using namespace ros;
using namespace moveit;




namespace bio_ik_kinematics_plugin
{


//#define ENABLE_LOG

template<class T>
inline void vprint(const T& a) {
    cerr << a << endl;
}
template<class T, class... AA>
inline void vprint(const T& a, AA... aa) {
    cerr << a << " ";
    vprint(aa...);
};

#define LOG_ALWAYS(...) vprint("ikbio ", __VA_ARGS__)

#ifdef ENABLE_LOG
#define LOG(...) LOG_ALWAYS(__VA_ARGS__)
#else
#define LOG(...) 
#endif

#define LOG_VAR(v) LOG(#v, (v));
#define LOG_FNC() LOG("fun", __func__, __LINE__)

// #define ERROR(...) { LOG("ERROR", __VA_ARGS__); exit(-1); }

// #define ERROR(a) { throw runtime_error(a); }

#include <signal.h>

#define ERROR(...) { LOG_ALWAYS(__VA_ARGS__); raise(SIGINT); }



// Compatability 
// MoveIt API changed from boost::shared_ptr to std::shared_ptr
// Built-in RobotModelConstPtr is only available in recent versions
// Define compatible RobotModel pointer
typedef decay<decltype(((moveit::core::RobotState*)0)->getRobotModel())>::type MoveItRobotModelConstPtr;







double profilertime()
{
    return ros::Time::now().toSec();
}
struct profilerbin;
vector<profilerbin*> profilerbins;
struct profilerbin
{
    const char* name;
    double time;
    double iterations;
    profilerbin(const char* name) : name(name), time(0), iterations(0) { profilerbins.push_back(this); }
};
struct profilerscope
{
    profilerbin* bin;
    double starttime;
    profilerscope(profilerbin* bin) : bin(bin), starttime(profilertime()) { }
    ~profilerscope() { bin->time += profilertime() - starttime; }
};
#define FNPROFILE() static profilerbin _profilerbin(__func__); profilerscope _profilerscope(&_profilerbin);
void profilerlog()
{
    for(auto* bin : profilerbins)
        LOG(bin->name, bin->time * 1000 / ++bin->iterations, "ms");
}
#define PROFILER_COUNTER(name) { static profilerbin bin(name); bin.time++; }












inline double mix(double a, double b, double f)
{
    return a * (1.0 - f) + b * f;
}

double clamp(double v, double lo, double hi)
{
  if(v < lo) v = lo;
  if(v > hi) v = hi;
  return v;
}






















struct Frame
{
    tf::Vector3 pos;
    tf::Quaternion rot;
    inline Frame() //: pos(0, 0, 0), rot(0, 0, 0, 1)
    {
    }
    inline Frame(const tf::Vector3& pos, const tf::Quaternion& rot) : pos(pos), rot(rot)
    {
    }
    explicit inline Frame(const KDL::Frame& kdl)
    {
        pos = tf::Vector3(kdl.p.x(), kdl.p.y(), kdl.p.z());
        double qx, qy, qz, qw;
        kdl.M.GetQuaternion(qx, qy, qz, qw);
        rot = tf::Quaternion(qx, qy, qz, qw);
    }
    explicit inline Frame(const geometry_msgs::Pose& msg)
    {
        tf::quaternionMsgToTF(msg.orientation, rot);
        pos = tf::Vector3(msg.position.x, msg.position.y, msg.position.z);
    }
    explicit inline Frame(const Eigen::Affine3d& f)
    {
        pos = tf::Vector3(f.translation().x(), f.translation().y(), f.translation().z());
        Eigen::Quaterniond q(f.rotation());
        rot = tf::Quaternion(q.x(), q.y(), q.z(), q.w());
    }
private:
    static const Frame identity_frame;
public:
    static inline const Frame& identity()
    {
        return identity_frame;
    }
};

const Frame Frame::identity_frame(tf::Vector3(0, 0, 0), tf::Quaternion(0, 0, 0, 1));



ostream& operator << (ostream& os, const Frame& f)
{
    return os << "(" << f.pos.x() << "," << f.pos.y() << "," << f.pos.z() << ";" << f.rot.x() << "," << f.rot.y() << "," << f.rot.z() << "," << f.rot.w() << ")";
}












/*
inline tf::Vector3 quatRotateFast(const tf::Quaternion& q, const tf::Vector3& v)
{
    double v_x = v.x();
    double v_y = v.y();
    double v_z = v.z();
    if(v_x == 0 && v_y == 0 && v_z == 0) return tf::Vector3(0, 0, 0);

    double q_w = q.w();
    if(q_w != 1)
    {
        double q_x = q.x();
        double q_y = q.y();
        double q_z = q.z();

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

        return tf::Vector3(r_x, r_y, r_z);
    } 
    else 
    {
        return v;
    }
}
*/


inline void quatRotateFast(const tf::Quaternion& q, const tf::Vector3& v, tf::Vector3& rs)
{
#ifdef CHECK_NAN
    if(!isfinite(q.x()) || !isfinite(v.x())) ERROR("NAN");
#endif

    double v_x = v.x();
    double v_y = v.y();
    double v_z = v.z();
    if(v_x == 0 && v_y == 0 && v_z == 0)
    {
        rs.setX(0);
        rs.setY(0);
        rs.setZ(0);
        return;
    }

    double q_w = q.w();
    if(q_w != 1)
    {
        double q_x = q.x();
        double q_y = q.y();
        double q_z = q.z();

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

        rs.setX(r_x);
        rs.setY(r_y);
        rs.setZ(r_z);
    } 
    else 
    {
        rs = v;
    }
}







/*
inline Frame operator * (const Frame& a, const Frame& b)
{
    //return Frame(a.pos + quatRotateFast(a.rot, b.pos), a.rot * b.rot);
    tf::Vector3 dp;
    quatRotateFast(a.rot, b.pos, dp);
    return Frame(a.pos + dp, a.rot * b.rot);
}
*/

inline void concatenate(const Frame& a, const Frame& b, Frame& rs)
{
#ifdef CHECK_NAN
    if(!isfinite(a.rot.x()) || !isfinite(a.pos.x()) || !isfinite(b.rot.x()) || !isfinite(b.pos.x())) ERROR("NAN Frame", a, b);
#endif
    
    tf::Vector3 dp;
    quatRotateFast(a.rot, b.pos, dp);
    rs.pos = a.pos + dp;
    rs.rot = a.rot * b.rot;
}

inline void invert(const Frame& f, Frame& ret)
{
    ret.rot = f.rot.inverse();
    quatRotateFast(ret.rot, -f.pos, ret.pos);
}

inline bool operator == (const Frame& a, const Frame& b)
{
    return a.pos == b.pos && a.rot == b.rot;
}

inline bool operator != (const Frame& a, const Frame& b)
{
    //return a.pos != b.pos || a.rot != b.rot;
    return !(a == b);
}




























class ModelInfo
{
    struct VariableInfo
    {
        bool bounded;
        double min;
        double max;
    };
    vector<VariableInfo> variables;
    vector<size_t> activeVariables;
    vector<moveit::core::JointModel::JointType> variable_joint_types;
public:
    ModelInfo(MoveItRobotModelConstPtr model, const vector<string>& tipNames)
    {
        for(auto& name : model->getVariableNames())
        {
            auto& bounds = model->getVariableBounds(name);
            VariableInfo info;
            info.bounded = bounds.position_bounded_;
            info.min = bounds.min_position_;
            info.max = bounds.max_position_;
            LOG("joint variable", variables.size(), name, info.bounded, info.min, info.max);
            variables.push_back(info);
        }
        for(size_t variable_index = 0; variable_index < model->getVariableCount(); variable_index++)
        {
            variable_joint_types.push_back(model->getJointOfVariable(variable_index)->getType());
        }
        for(auto tipName : tipNames)
        {
            auto* tipLink = model->getLinkModel(tipName);
            vector<const moveit::core::JointModel*> chain;
            for(auto* link = tipLink; link; link = link->getParentLinkModel())
            {
                auto* joint = link->getParentJointModel();
                if(joint->getMimic()) continue;
                if(joint->isPassive()) continue;
                chain.push_back(joint);
            }
            reverse(chain.begin(), chain.end());
            for(auto* joint : chain)
            {
                auto first = joint->getFirstVariableIndex();
                auto count = joint->getVariableCount();
                for(size_t vi = first; vi < first + count; vi++)
                {
                    if(find(activeVariables.begin(), activeVariables.end(), vi) != activeVariables.end()) continue;
                    activeVariables.push_back(vi);
                }
            }
        }
        for(auto& i : activeVariables) LOG("active variable", i);
    }
    double clip(double p, size_t i) const
    {
        auto& info = variables[i];
        if(info.bounded)
        {
            if(p > info.max) p = info.max;
            if(p < info.min) p = info.min;
        }
        return p;
    }
    double getSpan(size_t i) const { return variables[i].max - variables[i].min; }
    double getMin(size_t i) const { return variables[i].min; }
    double getMax(size_t i) const { return variables[i].max; }
    const vector<size_t>& getActiveVariables() const { return activeVariables; }
    bool isRevolute(size_t variable_index) const { return variable_joint_types[variable_index] == moveit::core::JointModel::REVOLUTE; }
    bool isPrismatic(size_t variable_index) const { return variable_joint_types[variable_index] == moveit::core::JointModel::PRISMATIC; }
};










            

class ModelFK_KDL
{
    MoveItRobotModelConstPtr robot_model;
    KDL::Tree kdlTree;
    shared_ptr<KDL::TreeFkSolverPos_recursive> kdlSolver;
    vector<string> tipNames;
    vector<Frame> tipFrames;
    KDL::JntArray jntArray;
    KDL::Frame kdlFrame;
    vector<double> jj;
public:
    ModelFK_KDL(MoveItRobotModelConstPtr model, const vector<string>& tipNames) 
        : tipNames(tipNames), tipFrames(tipNames.size()), robot_model(model)
    {
        LOG_VAR(kdl_parser::treeFromUrdfModel(*model->getURDF(), kdlTree));
        kdlSolver = make_shared<KDL::TreeFkSolverPos_recursive>(kdlTree);
    }
    void applyConfiguration(const vector<double>& jj0)
    {
        FNPROFILE();
        jj = jj0;
        robot_model->interpolate(jj0.data(), jj0.data(), 0.5, jj.data()); // force mimic update
        jntArray.resize(jj.size());
        for(size_t ji = 0; ji < jj.size(); ji++)
            jntArray(ji) = jj[ji];
        for(size_t ti = 0; ti < tipNames.size(); ti++)
        {
            kdlSolver->JntToCart(jntArray, kdlFrame, tipNames[ti]);
            tipFrames[ti] = Frame(kdlFrame);
        }
    }
    inline void incrementalBegin(const vector<double>& jj)
    {
    }
    inline void incrementalEnd()
    {
    }
    const Frame& getTipFrame(size_t fi) const
    {
        return tipFrames[fi];
    }
    const vector<Frame>& getTipFrames() const
    {
        return tipFrames;
    }
};

class ModelFK_MoveIt
{
    MoveItRobotModelConstPtr robot_model;
    moveit::core::RobotState robot_state;
    vector<string> tipNames;
    vector<Frame> tipFrames;
    vector<const moveit::core::LinkModel*> tipLinks;
    vector<double> jj;
public:
    ModelFK_MoveIt(MoveItRobotModelConstPtr model, const vector<string>& tipNames) 
        : tipNames(tipNames), tipFrames(tipNames.size()), robot_model(model), robot_state(model)
    {
        for(const auto& n : tipNames)
            tipLinks.push_back(robot_model->getLinkModel(n));
    }
    void applyConfiguration(const vector<double>& jj0)
    {
        FNPROFILE();
        jj = jj0;
        robot_model->interpolate(jj0.data(), jj0.data(), 0.5, jj.data()); // force mimic update
        robot_state.setVariablePositions(jj);
        for(size_t i = 0; i < tipFrames.size(); i++)
            tipFrames[i] = Frame(robot_state.getGlobalLinkTransform(tipLinks[i]));
    }
    inline void incrementalBegin(const vector<double>& jj)
    {
    }
    inline void incrementalEnd()
    {
    }
    const Frame& getTipFrame(size_t fi) const
    {
        return tipFrames[fi];
    }
    const vector<Frame>& getTipFrames() const
    {
        return tipFrames;
    }
};

class ModelFK_Fast
{
    MoveItRobotModelConstPtr robot_model;
    vector<string> tipNames;
    vector<Frame> tipFrames;
    vector<const moveit::core::LinkModel*> tipLinks;
    vector<double> variables;
    vector<Frame> linkFrames;
    vector<double> jointCacheVariables;
    vector<Frame> jointCacheFrames;
    vector<const moveit::core::LinkModel*> linkSchedule;
    vector<Frame> globalFrames;
    vector<vector<const moveit::core::LinkModel*>> linkChains;
    vector<tf::Vector3> jointAxisList;
    bool useIncremental;
private:
    inline const Frame& getLinkFrame(const moveit::core::LinkModel* linkModel)
    {
        return linkFrames[linkModel->getLinkIndex()];
    }
    inline bool checkJointMoved(const moveit::core::JointModel* jointModel)
    {
        size_t i0 = jointModel->getFirstVariableIndex();
        size_t cnt = jointModel->getVariableCount();
        if(cnt == 0) return false;
        if(cnt == 1) return variables[i0] != jointCacheVariables[i0];
        for(size_t i = i0; i < i0 + cnt; i++)
            if(variables[i] != jointCacheVariables[i]) 
                return true;
        return false;
    }
    inline void putJointCache(const moveit::core::JointModel* jointModel, const Frame& frame)
    {
        jointCacheFrames[jointModel->getJointIndex()] = frame;
        size_t i0 = jointModel->getFirstVariableIndex();
        size_t cnt = jointModel->getVariableCount();
        for(size_t i = i0; i < i0 + cnt; i++)
            jointCacheVariables[i] = variables[i];
    }
    inline const Frame& getJointFrame(const moveit::core::JointModel* jointModel)
    {
        auto jointType = jointModel->getType();
        if(jointType == moveit::core::JointModel::FIXED)
        {
            return Frame::identity();
        }
        size_t jointIndex = jointModel->getJointIndex();
        if(!checkJointMoved(jointModel)) return jointCacheFrames[jointIndex];
        switch(jointType)
        {
        case moveit::core::JointModel::REVOLUTE:
        {
            auto axis = jointAxisList[jointIndex];
            double v = variables[jointModel->getFirstVariableIndex()];
            double half_angle = v * 0.5f;
            double fsin = sin(half_angle); double fcos = cos(half_angle);
            Frame jointFrame(tf::Vector3(0,0,0), tf::Quaternion(axis.x() * fsin, axis.y() * fsin, axis.z() * fsin, fcos));
            putJointCache(jointModel, jointFrame);
            return jointCacheFrames[jointIndex];
        }
        case moveit::core::JointModel::PRISMATIC:
        {
            auto axis = jointAxisList[jointIndex];
            double v = variables[jointModel->getFirstVariableIndex()];
            Frame jointFrame(axis * v, tf::Quaternion(0, 0, 0, 1));
            putJointCache(jointModel, jointFrame);
            return jointCacheFrames[jointIndex];
        }
        case moveit::core::JointModel::FIXED:
        {
            Frame jointFrame(tf::Vector3(0, 0, 0), tf::Quaternion(0, 0, 0, 1));
            putJointCache(jointModel, jointFrame);
            return jointCacheFrames[jointIndex];
        }
        default:
        {
            //log("inknown joint type", jointType);
            double* jointVariables = variables.data() + jointModel->getFirstVariableIndex();
            Eigen::Affine3d jointTransform;
            jointModel->computeTransform(jointVariables, jointTransform);
            Frame jointFrame = Frame(jointTransform);
            putJointCache(jointModel, jointFrame);
            return jointCacheFrames[jointIndex];
        }
        }
    }
    inline void updateMimic(vector<double>& values)
    {
        for(auto* joint : robot_model->getMimicJointModels())
        {
            auto src = joint->getMimic()->getFirstVariableIndex();
            auto dest = joint->getFirstVariableIndex();
            values[dest] = values[src] * joint->getMimicFactor() + joint->getMimicOffset();
        }
    }
private: 
    vector<size_t> activeVariables;
    vector<pair<size_t, size_t>> variableToLinkMap;
public:
    ModelFK_Fast(MoveItRobotModelConstPtr model, const vector<string>& tipNames) 
        : tipNames(tipNames), tipFrames(tipNames.size()), robot_model(model)
    {
        useIncremental = false;
        for(const auto& n : tipNames)
            tipLinks.push_back(robot_model->getLinkModel(n));
        for(auto* linkModel : model->getLinkModels())
            linkFrames.push_back(Frame(linkModel->getJointOriginTransform()));
        jointCacheVariables.resize(model->getVariableCount(), DBL_MAX);
        jointCacheFrames.resize(model->getJointModelCount());
        globalFrames.resize(model->getLinkModelCount());
        
        for(auto* tipLink : tipLinks)
        {
            vector<const moveit::core::LinkModel*> chain;
            for(auto* link = tipLink; link; link = link->getParentLinkModel())
                chain.push_back(link);
            reverse(chain.begin(), chain.end());
            linkChains.push_back(chain);
            for(auto* link : chain)
            {
                if(find(linkSchedule.begin(), linkSchedule.end(), link) != linkSchedule.end()) continue;
                linkSchedule.push_back(link);
            }
        }
        
        jointAxisList.resize(robot_model->getJointModelCount());
        for(size_t i = 0; i < jointAxisList.size(); i++)
        {
            auto* jointModel = robot_model->getJointModel(i);
            if(auto* jointRevolute = dynamic_cast<const moveit::core::RevoluteJointModel*>(jointModel))
                jointAxisList[i] = tf::Vector3(jointRevolute->getAxis().x(), jointRevolute->getAxis().y(), jointRevolute->getAxis().z());
            if(auto* jointPrismatic = dynamic_cast<const moveit::core::PrismaticJointModel*>(jointModel))
                jointAxisList[i] = tf::Vector3(jointPrismatic->getAxis().x(), jointPrismatic->getAxis().y(), jointPrismatic->getAxis().z());
        }
        
        for(auto* linkModel : linkSchedule)
        {
            auto linkIndex = linkModel->getLinkIndex();
            auto* jointModel = linkModel->getParentJointModel();
            size_t vstart = jointModel->getFirstVariableIndex();
            size_t vcount = jointModel->getVariableCount();
            for(size_t vi = vstart; vi < vstart + vcount; vi++)
            {
                variableToLinkMap.push_back(make_pair(vi, linkIndex));
                
                if(find(activeVariables.begin(), activeVariables.end(), vi) != activeVariables.end()) continue;
                    activeVariables.push_back(vi);
            }
        }

        last_use_incremental = false;
    }
private:
    void updateFull(const vector<double>& jj0)
    {
        FNPROFILE();
        variables = jj0;
        updateMimic(variables);
        for(auto* linkModel : linkSchedule)
        {
            auto* jointModel = linkModel->getParentJointModel();
            
            /*if(linkModel->getParentLinkModel())
                globalFrames[linkModel->getLinkIndex()] = globalFrames[linkModel->getParentLinkModel()->getLinkIndex()] * getLinkFrame(linkModel) * getJointFrame(jointModel);
            else
                globalFrames[linkModel->getLinkIndex()] = getLinkFrame(linkModel) * getJointFrame(jointModel);
            */
            
            if(linkModel->getParentLinkModel())
            {
                Frame tmp;
                concatenate(globalFrames[linkModel->getParentLinkModel()->getLinkIndex()], getLinkFrame(linkModel), tmp);
                concatenate(tmp, getJointFrame(jointModel), globalFrames[linkModel->getLinkIndex()]);
            }
            else
            {
                concatenate(getLinkFrame(linkModel), getJointFrame(jointModel), globalFrames[linkModel->getLinkIndex()]);
            }
            
            //if(linkModel->getParentLinkModel() && linkModel->getParentLinkModel()->getLinkIndex() > linkModel->getLinkIndex()) { LOG("wrong link order"); throw runtime_error("wrong link order"); }
        }
        for(size_t itip = 0; itip < tipLinks.size(); itip++)
        {
            tipFrames[itip] = globalFrames[tipLinks[itip]->getLinkIndex()];
        }
    }
public:
    bool last_use_incremental;
    void applyConfiguration(const vector<double>& jj0)
    {
        bool changed = false;
        if(useIncremental != last_use_incremental)
            changed = true;
        last_use_incremental = useIncremental;
        if(variables.size() != jj0.size())
            changed = true;
        if(!changed)
            for(auto& vi : activeVariables) 
                if(variables[vi] != jj0[vi])
                    changed = true;
        if(!changed) return;
        
        if(useIncremental)
            updateIncremental(jj0);
        else
            updateFull(jj0);
    }
    inline const Frame& getTipFrame(size_t fi) const
    {
        return tipFrames[fi];
    }
    inline const vector<Frame>& getTipFrames() const
    {
        return tipFrames;
    }
private:
    vector<vector<Frame>> chainCache;
    vector<double> vars, variables0;
    vector<Frame> testTips;
    vector<uint8_t> linksChanged;
public:
    inline void incrementalBegin(const vector<double>& jj)
    {
        applyConfiguration(jj);
        chainCache.clear();
        chainCache.resize(tipFrames.size());
        linksChanged.resize(robot_model->getLinkModelCount());
        vars.resize(jj.size());
        variables0.resize(jj.size());
        useIncremental = true;
    }
    inline void incrementalEnd()
    {
        useIncremental = false;
    }
private:
    void updateIncremental(const vector<double>& vars0)
    {
        FNPROFILE();
        
        //PROFILER_COUNTER("incremental update count");
        
        for(auto& vi : activeVariables) 
        {
            vars[vi] = vars0[vi];
            variables0[vi] = variables[vi];
        }
        
        updateMimic(vars);
        
        for(auto* linkModel : linkSchedule)
            linksChanged[linkModel->getLinkIndex()] = false;
        for(auto& x : variableToLinkMap)
        {
            auto variableIndex = x.first;
            auto linkIndex = x.second;
            if(vars[variableIndex] != variables[variableIndex]) linksChanged[linkIndex] = true;
        }

        // update
        for(size_t ichain = 0; ichain < linkChains.size(); ichain++)
        {
            auto& linkChain = linkChains[ichain];
            auto& cacheChain = chainCache[ichain];

            size_t iposmax = 0;
            for(size_t ipos = 0; ipos < linkChain.size(); ipos++)
            {
                auto* linkModel = linkChain[ipos];
                bool changed = linksChanged[linkModel->getLinkIndex()];
                if(changed) iposmax = ipos;
            }
            
            for(size_t ipos = 0; ipos <= iposmax; ipos++)
            {
                auto* linkModel = linkChain[ipos];
                auto* jointModel = linkModel->getParentJointModel();
                
                bool changed = linksChanged[linkModel->getLinkIndex()];
                
                if(cacheChain.size() <= ipos || changed)
                {
                    Frame before, after;
                    
                    if(ipos < cacheChain.size())
                    {
                        before = cacheChain[ipos];
                    }
                    else
                    {
                        /*if(ipos > 0)
                            before = cacheChain[ipos - 1] * getLinkFrame(linkModel) * getJointFrame(jointModel);
                        else
                            before = getLinkFrame(linkModel) * getJointFrame(jointModel);*/
                        if(ipos > 0)
                        {
                            Frame tmp;
                            concatenate(cacheChain[ipos - 1], getLinkFrame(linkModel), tmp);
                            concatenate(tmp, getJointFrame(jointModel), before);
                        }
                        else
                        {
                            concatenate(getLinkFrame(linkModel), getJointFrame(jointModel), before);
                        }
                    }
                    
                    chainCache[ichain].resize(ipos + 1, Frame::identity());
                        
                    if(changed)
                    {
                        if(ichain > 0 && ipos < linkChains[ichain - 1].size() && linkChains[ichain][ipos] == linkChains[ichain - 1][ipos])
                        {
                            after = chainCache[ichain - 1][ipos];
                        }
                        else
                        {
                            size_t vstart = jointModel->getFirstVariableIndex();
                            size_t vcount = jointModel->getVariableCount();

                            if(vcount == 1)
                                variables[vstart] = vars[vstart];
                            else
                                for(size_t vi = vstart; vi < vstart + vcount; vi++)
                                    variables[vi] = vars[vi];
                                
                            /*if(ipos > 0)
                                after = cacheChain[ipos - 1] * getLinkFrame(linkModel) * getJointFrame(jointModel);
                            else
                                after = getLinkFrame(linkModel) * getJointFrame(jointModel);*/
                                
                            if(ipos > 0)
                            {
                                Frame temp;
                                concatenate(cacheChain[ipos - 1], getLinkFrame(linkModel), temp);
                                concatenate(temp, getJointFrame(jointModel), after);
                            }
                            else
                            {
                                concatenate(getLinkFrame(linkModel), getJointFrame(jointModel), after);
                            }
                                
                            if(vcount == 1)
                                variables[vstart] = variables0[vstart];
                            else
                                for(size_t vi = vstart; vi < vstart + vcount; vi++)
                                    variables[vi] = variables0[vi];
                                    
                            //PROFILER_COUNTER("incremental update transforms");
                        }
                        chainCache[ichain][ipos] = after;
                        
                        //tipFrames[ichain] = inverse(before) * tipFrames[ichain];
                        //tipFrames[ichain] = after * tipFrames[ichain];
                        
                        Frame before_inverse;
                        invert(before, before_inverse);
                        
                        //tipFrames[ichain] = after * before_inverse * tipFrames[ichain];
                        {
                            Frame temp;
                            concatenate(after, before_inverse, temp);
                            concatenate(temp, tipFrames[ichain], tipFrames[ichain]);
                        }
                    }
                    else
                    {
                        after = before;
                        chainCache[ichain][ipos] = after;
                    }
                }
            }
        }
        
        // set new vars
        //variables = vars;
        for(auto& vi : activeVariables) variables[vi] = vars[vi];
        
        // test
        /*if(0)
        {
            testTips = tipFrames;
            updateFull(vars);
            for(size_t i = 0; i < tipFrames.size(); i++)
            {
                auto dist = tipFrames[i].pos.distance(testTips[i].pos);
                LOG_VAR(dist);
                if(dist > 0.001)
                {
                    LOG(tipFrames[i].pos.x(), tipFrames[i].pos.y(), tipFrames[i].pos.z());
                    LOG(testTips[i].pos.x(), testTips[i].pos.y(), testTips[i].pos.z());
                    ERROR("incremental update error");
                }
            }
        }*/
    }
};





//typedef ModelFK_KDL ModelFK;
//typedef ModelFK_MoveIt ModelFK;
typedef ModelFK_Fast ModelFK;










class HeuristicErrorTree
{
    size_t variable_count, tip_count;
    vector<double> table;
    vector<double> chain_lengths;
public:
    HeuristicErrorTree(MoveItRobotModelConstPtr robot_model, const vector<string>& tip_names)
    {
        tip_count = tip_names.size();
        variable_count = robot_model->getVariableCount();
        table.resize(tip_count * variable_count);
        for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
        {
            auto& tip_name = tip_names[tip_index];
            for(auto* link_model = robot_model->getLinkModel(tip_name); link_model; link_model = link_model->getParentLinkModel())
            {
                auto* joint_model = link_model->getParentJointModel();
                size_t v1 = joint_model->getFirstVariableIndex();
                size_t vn = joint_model->getVariableCount();
                for(size_t variable_index = v1; variable_index < v1 + vn; variable_index++)
                    table[variable_index * tip_count + tip_index] = 1;
            }
        }
        for(size_t variable_index = 0; variable_index < variable_count; variable_index++)
        {
            double sum = 0;
            for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
                sum += table[variable_index * tip_count + tip_index];
            if(sum > 0)
                for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
                    table[variable_index * tip_count + tip_index] /= sum;
        }
        chain_lengths.resize(tip_count);
        for(size_t tip_index = 0; tip_index < tip_count; tip_index++)
        {
            auto& tip_name = tip_names[tip_index];
            double chain_length = 0;
            for(auto* link_model = robot_model->getLinkModel(tip_name); link_model; link_model = link_model->getParentLinkModel())
            {
                chain_length += Frame(link_model->getJointOriginTransform()).pos.length();
            }
            chain_lengths[tip_index] = chain_length;
        }
    }
    double getInfluence(size_t variable_index, size_t tip_index) const
    {
        return table[variable_index * tip_count + tip_index];
    }
    double getChainLength(size_t tip_index) const
    {
        return chain_lengths[tip_index];
    }
};















template<class t>
class linear_int_distribution
{
    uniform_int_distribution<t> base;
    t n;
public:
    inline linear_int_distribution(t vrange) : n(vrange), base(0, vrange)
    {
    }
    template<class generator>
    inline t operator () (generator& g)
    {
        while(true)
        {
            t v = base(g) + base(g);
            if(v < n) return n - v - 1;
        }
    }
};







struct IKTipInfo
{
    bool position_only_ik;
    double weight;
    IKTipInfo() :
        position_only_ik(0),
        weight(1)
    {
    }
};


struct IKParams
{
    MoveItRobotModelConstPtr robot_model;
    vector<string> tip_frames;
    vector<size_t> active_variables;
    ros::NodeHandle node_handle;
    vector<IKTipInfo> tip_infos;
};






class Evolution
{

    struct Individual
    {
        vector<double> genes; 
        vector<double> gradients;
        double extinction;
        double fitness;
    };

    ModelFK model;
    ModelInfo modelInfo;
    HeuristicErrorTree heuristicErrorTree;
    vector<double> solution;
    vector<Individual> population;
    int populationSize, eliteCount;
    vector<Frame> tipObjectives;
    vector<Individual*> tempPool;
    vector<Individual> tempOffspring;
    vector<double> initialGuess;
    
    
    
    vector<size_t> mutable_genes;
    const vector<size_t>& getGenes() const { return mutable_genes; }
    
    /*static const bool opt_quadratic_error = 1;
    static const bool opt_final_adjustment = 1;
    static const bool opt_angular_scale_full_circle = 1;
    static const bool opt_no_wipeout = 1;
    
    static const int evolution_steps = 25;
    static const int adjustment_steps = 500;
    
    //static const auto fitness_randomization = 1.0;
    #define fitness_randomization 1.0
    //static const double fitness_randomization = 1;*/
    
    bool opt_quadratic_error;
    bool opt_final_adjustment;
    bool opt_angular_scale_full_circle;
    bool opt_no_wipeout;
    int evolution_steps;
    int adjustment_steps;
    double fitness_randomization;
    bool opt_adjustment2;
    double velocity_cost;
    
    void setParams(const IKParams& p)
    {
        auto& n = p.node_handle;
        
        /*n.param("quadratic_error", opt_quadratic_error, true);
        n.param("final_adjustment", opt_final_adjustment, true);
        n.param("full_circle", opt_angular_scale_full_circle, true);
        n.param("no_wipeout", opt_no_wipeout, false);
        n.param("evolution_steps", evolution_steps, 25);
        n.param("adjustment_steps", adjustment_steps, 50);
        n.param("fitness_randomization", fitness_randomization, 0.6);*/
         
        n.param("quadratic", opt_quadratic_error, true);
        
        //n.param("final_adjustment", opt_final_adjustment, false);
        
        n.param("full_circle", opt_angular_scale_full_circle, true);
        n.param("no_wipeout", opt_no_wipeout, false);
        //n.param("evolution_steps", evolution_steps, 25);
        //n.param("adjustment_steps", adjustment_steps, 50);
        n.param("fitness_randomization", fitness_randomization, 0.0);
        
        n.param("bisection", opt_adjustment2, true);
        
        //n.param("population_size", populationSize, 12);
        //n.param("elite_count", eliteCount, 4);
        
        n.param("population_size", populationSize, 8);
        n.param("elite_count", eliteCount, 4);
        
        //n.param("population_size", populationSize, 8);
        //n.param("elite_count", eliteCount, 0);
        
        //n.param("velocity_cost", velocity_cost, 0.01);
        
        //exit(-1);
    }
    
    bool in_final_adjustment_loop;
    
    
    random_device rdev;
    //mt19937 rng;
    minstd_rand rng;
    //ranlux24 rng;
    //knuth_b rng;
    
    inline double random(double min, double max)
    {
        return uniform_real_distribution<double>(min, max)(rng);
    }
    
    inline double random() 
    {
        return uniform_real_distribution<double>(0, 1)(rng);
    } 
    
    
    
    template<class t>
    inline t select(const vector<t>& v)
    {
        //FNPROFILE();
        linear_int_distribution<size_t> d(v.size());
        size_t index = d(rng);
        return v[index];
    }
    
    inline double clip(double v, size_t i)
    {
        return modelInfo.clip(v, i);
    }
    
    inline double getMutationStrength(size_t i, const Individual& parentA, const Individual& parentB)
    {
        double extinction = 0.5 * (parentA.extinction + parentB.extinction);
        double span = modelInfo.getSpan(i);
        return span * extinction;
    }
    
    double computeAngularScale(size_t tip_index, const Frame& tip_frame)
    {
        double angular_scale = sqrt(heuristicErrorTree.getChainLength(tip_index) * tip_frame.pos.length()) / M_PI;
        if(opt_angular_scale_full_circle) angular_scale *= 0.5;
        return angular_scale;
    }
    
    double computeTipFitness(size_t tip_index, const Frame& tip_frame, bool balanced, bool for_heuristic_error)
    {
        // TODO: other goal types
            
        const auto& ta = tipObjectives[tip_index];
        const auto& tb = tip_frame;
        
        double tdist, rdist;

        //if(for_heuristic_error || !opt_quadratic_error || in_final_adjustment_loop)
        if(for_heuristic_error || !opt_quadratic_error)
        {
            tdist = ta.pos.distance(tb.pos) / computeAngularScale(tip_index, tip_frame);
            rdist = ta.rot.angle(tb.rot); 
        } 
        else 
        {
            // TODO: scaling ????????????????????????????????????

            /*tdist = ta.pos.distance2(tb.pos);
            rdist = M_PI - ta.rot.dot(tb.rot) * M_PI;*/

            double angularScale = computeAngularScale(tip_index, tip_frame);
            tdist = ta.pos.distance2(tb.pos) / (angularScale * angularScale); // square distance
            rdist = (1.0 - ta.rot.dot(tb.rot)) * 2.0; // small-angle approximation for square angle
        }
        
        if(params.tip_infos[tip_index].position_only_ik) rdist = 0;

        return mix(tdist, rdist, (balanced || in_final_adjustment_loop) ? 0.5 : mix(0.5, random(), fitness_randomization));
        //return mix(tdist, rdist, (balanced || in_final_adjustment_loop) ? 0.5 : random(0.2, 0.8));
    }
    
    double getHeuristicError(size_t variable_index, bool balanced)
    {
        //return modelInfo.getSpan(variable_index) * random(-1, 1) * (1 << uniform_int_distribution<int>(0, 4)(rng)) * (1.0 / (1 << 4));
        //return modelInfo.getSpan(variable_index) * (1 << uniform_int_distribution<int>(0, 20)(rng)) * (1.0 / (1 << 20));
                
        //return modelInfo.getSpan(variable_index) * random(-1, 1) * random() * random() * random();
        
        //if(in_final_adjustment_loop) if(random() < 0.5) return random();
        double heuristic_error = 0;
        for(size_t tip_index = 0; tip_index < tipObjectives.size(); tip_index++)
           heuristic_error += heuristicErrorTree.getInfluence(variable_index, tip_index) * computeTipFitness(tip_index, model.getTipFrame(tip_index), balanced, true); 
        //if(in_final_adjustment_loop) heuristic_error *= random() * random() * 5;
        //if(in_final_adjustment_loop) heuristic_error *= 2;
        //heuristic_error *= 2;
        //heuristic_error *= random() * random() * 5;
        return heuristic_error;
    }
    
    bool in_adjustment_2, in_get_solution_fitness;
    
    double computeFitness(const vector<double>& genes, bool balanced)
    {
        FNPROFILE();
        model.applyConfiguration(genes);
        double fitness_sum = 0.0;
        
        double w_sum = 0;
        
        for(size_t tip_index = 0; tip_index < tipObjectives.size(); tip_index++)
        {
            double f = computeTipFitness(tip_index, model.getTipFrame(tip_index), balanced, false);
            
            double w = params.tip_infos[tip_index].weight;
            if(opt_quadratic_error) w *= w;
            f *= w;
            w_sum += w;
            
            fitness_sum += f;
        }
        
        fitness_sum /= w_sum;
        
        if(!in_adjustment_2 && !in_get_solution_fitness && velocity_cost > 0) 
        {
            double velcost = 0;
            for(auto i : getGenes())
            {
                auto* joint = params.robot_model->getJointOfVariable(i);
                auto& bounds = joint->getVariableBounds()[i - joint->getFirstVariableIndex()];
                if(bounds.velocity_bounded_)
                {
                    auto a = genes[i];
                    auto b = initialGuess[i];
                    auto d = a - b;
                    d /= bounds.max_velocity_;
                    d *= d;
                    velcost = max(velcost, d);
                    //velcost += d;
                }
            }
            if(opt_quadratic_error)
                fitness_sum += velcost * velocity_cost * velocity_cost;
            else
                fitness_sum += sqrt(velcost) * velocity_cost;
        }
        
        return fitness_sum;
    }
    
    void reroll(Individual& offspring)
    {
        FNPROFILE();
        //for(size_t i = 0; i < offspring.genes.size(); i++)
        for(auto i : getGenes())
        {
            offspring.genes[i] = random(modelInfo.getMin(i), modelInfo.getMax(i));
            offspring.gradients[i] = 0;
        }
        offspring.fitness = computeFitness(offspring.genes, false);
    }
    
    bool checkWipeout()
    {
        FNPROFILE();
        auto& genes = population[0].genes;
        //for(size_t i = 0; i < genes.size(); i++)
        for(auto i : getGenes())
        {
            double v0 = genes[i];
            double fitness = computeFitness(genes, true);
            double heuristicError = getHeuristicError(i, true);
            //double heuristicError = 0.001;
            genes[i] = modelInfo.clip(v0 + random(0, heuristicError), i);
            double incFitness = computeFitness(genes, true);
            genes[i] = modelInfo.clip(v0 - random(0, heuristicError), i);
            double decFitness = computeFitness(genes, true);
            genes[i] = v0;
            if(incFitness < fitness || decFitness < fitness) 
            {
                //LOG("no wipeout");
                return false;
            }
        }
        //LOG("wipeout 1");
        return true;
    }
    
    void computeExtinctions()
    {
        double min = population.front().fitness;
        double max = population.back().fitness;
        for(size_t i = 0; i < populationSize; i++)
        {
            double grading = (double)i / (double)(populationSize - 1);
            population[i].extinction = (population[i].fitness + min * (grading - 1)) / max;
        }
    }
    
    bool tryUpdateSolution()
    {
        FNPROFILE();
        double solutionFitness = computeFitness(solution, true);
        double candidateFitness = computeFitness(population[0].genes, true);
        //LOG_VAR(solutionFitness);
        //LOG_VAR(candidateFitness);
        if(candidateFitness < solutionFitness)
        {
            solution = population[0].genes;
            //solution = initialGuess;
            //for(auto i : getGenes())
            //    solution[i] = population[0].genes[i];
            return true;
        }
        return false;
    }
    
    double getMutationProbability(const Individual& parentA, const Individual& parentB) 
    {
        double extinction = 0.5 * (parentA.extinction + parentB.extinction);
        double inverse = 1.0 / parentA.genes.size();
        return extinction * (1.0 - inverse) + inverse;
    }
    
    void sortByFitness()
    {
        FNPROFILE();
        sort(population.begin(), population.end(), [] (const Individual& a, const Individual& b) 
        {
            return a.fitness < b.fitness;
        });
    }
    
    void reproduce(Individual& offspring, const Individual& parentA, const Individual& parentB, const Individual& prototype)
    {
        FNPROFILE();
        for(size_t i = 0; i < offspring.genes.size(); i++)
        //for(auto i : getGenes())
        {
            offspring.genes[i] = mix(parentA.genes[i], parentB.genes[i], random());
            offspring.genes[i] += parentA.gradients[i] * random();
            offspring.genes[i] += parentB.gradients[i] * random();
            
            double storage = offspring.genes[i];
            
            if(random() < getMutationProbability(parentA, parentB))
                offspring.genes[i] += random(-1, 1) * getMutationStrength(i, parentA, parentB);
                
            offspring.genes[i] += mix(
                random() * (0.5 * (parentA.genes[i] + parentB.genes[i]) - offspring.genes[i]),
                random() * (prototype.genes[i] - offspring.genes[i]),
                random());
                
            offspring.genes[i] = clip(offspring.genes[i], i);
            
            offspring.gradients[i] = random() * offspring.gradients[i] + offspring.genes[i] - storage;
        }
        
        offspring.fitness = computeFitness(offspring.genes, false);
    }
    
    
    
    void exploit(Individual& individual)
    {
        FNPROFILE();
        double fitness_sum = 0;
        //for(size_t i = 0; i < individual.genes.size(); i++)
        model.incrementalBegin(individual.genes);
        for(auto i : getGenes())
        {
            double fitness = computeFitness(individual.genes, false);
            double heuristicError = getHeuristicError(i, false);
            double v_0 = individual.genes[i];
            double v_inc = modelInfo.clip(v_0 + random(0, heuristicError), i);
            double v_dec = modelInfo.clip(v_0 - random(0, heuristicError), i);
            individual.genes[i] = v_inc;
            double inc_fitness = computeFitness(individual.genes, false);
            individual.genes[i] = v_dec;
            double dec_fitness = computeFitness(individual.genes, false);
            if(inc_fitness < fitness && inc_fitness <= dec_fitness)
            {
                individual.genes[i] = v_inc;
                individual.gradients[i] = v_0 * random() + v_inc - v_0;
                fitness_sum += inc_fitness;
            } else 
            if(dec_fitness < fitness && dec_fitness <= inc_fitness) 
            {
                individual.genes[i] = v_dec;
                individual.gradients[i] = v_0 * random() + v_dec - v_0;
                fitness_sum += dec_fitness;
            } else
            {
                individual.genes[i] = v_0;
                fitness_sum += fitness;
            }
        }
        model.incrementalEnd();
        individual.fitness = fitness_sum / individual.genes.size();
    }
    
    
    
    IKParams params;
    
    
public:

    Evolution(const IKParams& p) : 
        model(p.robot_model, p.tip_frames), 
        populationSize(12), 
        eliteCount(4), 
        rdev(), 
        rng(rdev()), 
        modelInfo(p.robot_model, p.tip_frames),
        heuristicErrorTree(p.robot_model, p.tip_frames),
        in_final_adjustment_loop(false),
        params(p),
        in_get_solution_fitness(false),
        in_adjustment_2(false)
    {
        setParams(p);
        /*for(size_t i : p.active_variables)
            for(size_t j : modelInfo.getActiveVariables())
                if(i == j)
                    mutable_genes.push_back(i);*/
        mutable_genes = p.active_variables;
    }
    
    
    
    void initialize(const vector<double>& initialGuess, const vector<Frame>& tipObjectives)
    {
        // TODO: [0]:solution [1...n-1]:random
        
        /*{
            this->initialGuess = initialGuess;
            solution = initialGuess;
            this->tipObjectives = tipObjectives;
            population.resize(populationSize);
            for(auto& p : population)
            {
                p.genes = solution;
                p.gradients.clear();
                p.gradients.resize(p.genes.size(), 0);
                p.fitness = computeFitness(p.genes, false);
            }
            sortByFitness();
            computeExtinctions();
        }*/
        
        {
            this->initialGuess = initialGuess;
            solution = initialGuess;
            this->tipObjectives = tipObjectives;
            
            population.resize(populationSize);
            
            {
                auto& p = population[0];
                p.genes = solution;
                p.gradients.clear();
                p.gradients.resize(p.genes.size(), 0);
                p.fitness = computeFitness(p.genes, false);
            }
            
            for(int i = 1; i < populationSize; i++)
            {
                auto& p = population[i];
                p.genes = solution;
                p.gradients.clear();
                p.gradients.resize(p.genes.size(), 0);
                reroll(p);
            }
                
            sortByFitness();
            computeExtinctions();
        }
        
    }
    
    const vector<double>& getSolution()
    {
        return solution;
    }

    double getSolutionFitness()
    {
        in_get_solution_fitness = true;
        double f = computeFitness(solution, true);
        in_get_solution_fitness = false;
        return f;
    }
    
    const vector<Frame>& getSolutionTipFrames()
    {
        model.applyConfiguration(solution);
        return model.getTipFrames();
    }
    
    bool evolve()
    {
        FNPROFILE();
        
        auto& offspring = tempOffspring;
        offspring = population;
    
        for(size_t i = 0; i < eliteCount; i++)
        {
            offspring[i] = population[i];
            exploit(offspring[i]);
        }
        
        auto& pool = tempPool;
        pool.resize(populationSize);
        iota(pool.begin(), pool.end(), &population[0]);
        
        for(size_t i = eliteCount; i < populationSize; i++)
        {
            if(pool.size() > 0)
            {
                auto& parentA = *select(pool);
                auto& parentB = *select(pool);
                auto& prototype = *select(pool);
                reproduce(offspring[i], parentA, parentB, prototype);
                if(offspring[i].fitness < parentA.fitness) pool.erase(remove(pool.begin(), pool.end(), &parentA), pool.end());
                if(offspring[i].fitness < parentB.fitness) pool.erase(remove(pool.begin(), pool.end(), &parentB), pool.end());
            } 
            else 
            {
                reroll(offspring[i]);
            }
        }
        
        population = offspring;
        
        sortByFitness();
        
        computeExtinctions();
        
        if(tryUpdateSolution()) return true;
        
        if(opt_no_wipeout) return false;
        
        if(!checkWipeout()) return false;
        
        initialize(initialGuess, tipObjectives);
        return tryUpdateSolution();
    }

    void step()
    {
        in_adjustment_2 = false;
        
        evolve();
        
        if(opt_adjustment2)
        {
            in_adjustment_2 = true;
            model.incrementalBegin(solution);
            for(int j = 0; j < 4; j++) adjust2();
            //for(int j = 0; j < 4; j++) adjust3();
            //for(int j = 0; j < 4; j++) adjust2(), adjust3();
            model.incrementalEnd();
            in_adjustment_2 = false;
        }
    }
    
    /*
    void adjust3()
    {
        double solution_fitness = computeFitness(solution, true);
        //model.incrementalBegin(solution);
        for(auto ivar : params.active_variables)
        {
            for(int iter = 0; iter < 8; iter++)
            {
                temp = solution;

                double f = modelInfo.getSpan(ivar);

                f *= (1 << uniform_int_distribution<int>(0, 20)(rng)) * (1.0 / (1 << 20));

                temp[ivar] = modelInfo.clip(solution[ivar] + f, ivar);
                double up_fitness = computeFitness(temp, true);
                
                temp[ivar] = modelInfo.clip(solution[ivar] - f, ivar);
                double dn_fitness = computeFitness(temp, true);
                
                if(up_fitness < solution_fitness && up_fitness < dn_fitness)
                {
                    temp[ivar] = modelInfo.clip(solution[ivar] + f, ivar);
                    solution = temp;
                    solution_fitness = up_fitness;
                }
                
                if(dn_fitness < solution_fitness && dn_fitness < up_fitness)
                {
                    temp[ivar] = modelInfo.clip(solution[ivar] - f, ivar);
                    solution = temp;
                    solution_fitness = dn_fitness;
                }
            }
        }
        //model.incrementalEnd();
    }
    */
    
    vector<double> temp;
    
    void adjust2()
    {
        FNPROFILE();
        double solution_fitness = computeFitness(solution, true);
        for(int i = 0; i < params.active_variables.size(); i++)
        {
            auto ivar = params.active_variables[i];
            temp = solution;
            double dir;
            {
                double f = 0.00001;

                temp[ivar] = modelInfo.clip(solution[ivar] + f, ivar);
                double up_fitness = computeFitness(temp, true);
                
                temp[ivar] = modelInfo.clip(solution[ivar] - f, ivar);
                double dn_fitness = computeFitness(temp, true);
                
                dir = (up_fitness > dn_fitness) ? 1 : -1;
            }
            double f = modelInfo.getSpan(ivar);
            while(f > 0.00001)
            {
                temp[ivar] = modelInfo.clip(solution[ivar] - f * dir, ivar);
                double temp_fitness = computeFitness(temp, true);
                if(temp_fitness < solution_fitness)
                {
                    solution = temp;
                    solution_fitness = temp_fitness;
                }
                f *= 0.5;
            }
        }
    }
    
    static int concurrency() { return 4; }
};










double smoothstep(float a, float b, float v)
{
    v = clamp((v - a) / (b - a), 0.0, 1.0); 
    return v * v * (3 - 2 * v);
}






class IKBase
{
protected:
    ModelFK model;
    ModelInfo modelInfo;
    HeuristicErrorTree heuristicErrorTree;
    vector<double> solution, temp;
    vector<Frame> tipObjectives;
    random_device rdev;
    minstd_rand rng;
    IKParams params;
    
    inline double random(double min, double max) { return uniform_real_distribution<double>(min, max)(rng); }
    inline double random() { return uniform_real_distribution<double>(0, 1)(rng); } 
    
    double computeFitness(const vector<double>& genes)
    {
        model.applyConfiguration(genes);
        double fitness_sum = 0.0;
        for(size_t tip_index = 0; tip_index < tipObjectives.size(); tip_index++)
        {
            const auto& ta = tipObjectives[tip_index];
            const auto& tb = model.getTipFrame(tip_index);
            double tdist, rdist;
            tdist = ta.pos.distance2(tb.pos); // square distance
            rdist = (1.0 - ta.rot.dot(tb.rot)) * 2.0; // small-angle approximation for square angle
            fitness_sum += tdist + rdist;
            //fitness_sum += tdist;
        }
        return fitness_sum;
    }

public:

    IKBase(const IKParams& p) : 
        model(p.robot_model, p.tip_frames), 
        modelInfo(p.robot_model, p.tip_frames),
        heuristicErrorTree(p.robot_model, p.tip_frames),
        rdev(), 
        rng(rdev()),
        params(p)
    {
    }

    const vector<double>& getSolution()
    {
        return solution;
    }

    double getSolutionFitness()
    {
        return computeFitness(solution);
    }
    
    const vector<Frame>& getSolutionTipFrames()
    {
        model.applyConfiguration(solution);
        return model.getTipFrames();
    }
    
    void initialize(const vector<double>& initialGuess, const vector<Frame>& tipObjectives)
    {
        this->solution = initialGuess;
        this->tipObjectives = tipObjectives;
        
        temp.resize(solution.size());
        
        for(auto i : params.active_variables)
            solution[i] = random(modelInfo.getMin(i), modelInfo.getMax(i));
    }
};

class IKStep : public IKBase
{
public:
    IKStep(const IKParams& p) : IKBase(p)
    {
    }
    void step()
    {
        double solution_fitness = computeFitness(solution);

        model.incrementalBegin(solution);
        for(auto ivar : params.active_variables)
        {
            temp = solution;
            double dir;
            {
                double f = 0.0001;
                
                temp[ivar] = modelInfo.clip(solution[ivar] + f, ivar);
                double up_fitness = computeFitness(temp);
                
                temp[ivar] = modelInfo.clip(solution[ivar] - f, ivar);
                double dn_fitness = computeFitness(temp);
                
                dir = (up_fitness > dn_fitness) ? 1 : -1;
            }
            double f = modelInfo.getSpan(ivar);
            while(f > 0.0001)
            {
                temp[ivar] = modelInfo.clip(solution[ivar] - f * dir, ivar);
                double temp_fitness = computeFitness(temp);
                if(temp_fitness < solution_fitness)
                {
                    solution = temp;
                    solution_fitness = temp_fitness;
                }
                f *= 0.5;
            }
        }
        model.incrementalEnd();
    }
    static int concurrency() { return 8; }
};


class IKStepExp : public IKBase
{
public:
    IKStepExp(const IKParams& p) : IKBase(p)
    {
    }
    void step()
    {
        double solution_fitness = computeFitness(solution);
        model.incrementalBegin(solution);
        for(auto ivar : params.active_variables)
        {
            for(int iter = 0; iter < 8; iter++)
            {
                temp = solution;

                double f = modelInfo.getSpan(ivar);

                f *= (1 << uniform_int_distribution<int>(0, 20)(rng)) * (1.0 / (1 << 20));

                temp[ivar] = modelInfo.clip(solution[ivar] + f, ivar);
                double up_fitness = computeFitness(temp);
                
                temp[ivar] = modelInfo.clip(solution[ivar] - f, ivar);
                double dn_fitness = computeFitness(temp);
                
                if(up_fitness < solution_fitness && up_fitness < dn_fitness)
                {
                    temp[ivar] = modelInfo.clip(solution[ivar] + f, ivar);
                    solution = temp;
                    solution_fitness = up_fitness;
                }
                
                if(dn_fitness < solution_fitness && dn_fitness < up_fitness)
                {
                    temp[ivar] = modelInfo.clip(solution[ivar] - f, ivar);
                    solution = temp;
                    solution_fitness = dn_fitness;
                }
            }
        }
        model.incrementalEnd();
    }
    static int concurrency() { return 8; }
};


class IKStepExpP : public IKBase
{
public:
    IKStepExpP(const IKParams& p) : IKBase(p)
    {
    }
    void step()
    {
        double solution_fitness = computeFitness(solution);

        for(int iter = 0; iter < 32; iter++)
        {
            temp = solution;
            for(auto ivar : params.active_variables)
            {
                for(int i2 = 0; i2 < 1; i2++)
                {
                    double f = modelInfo.getSpan(ivar);
                    f *= (1 << uniform_int_distribution<int>(0, 20)(rng)) * (1.0 / (1 << 20));
                    if(uniform_int_distribution<int>(0, 1)(rng)) f = -f;
                    temp[ivar] = modelInfo.clip(solution[ivar] + f, ivar);
                }
            }
                
            double temp_fitness = computeFitness(temp);
     
            if(temp_fitness < solution_fitness)
            {
                solution = temp;
                solution_fitness = temp_fitness;
            }
        }
    }
    static int concurrency() { return 8; }
};




typedef Evolution IKSolver;

//typedef IKStep IKSolver;

//typedef IKStepExp IKSolver;

//typedef IKStepExpP IKSolver;














void test()
{
    {
        LOG("rand test");
        int n = 10;
        vector<int> v(n);
        linear_int_distribution<size_t> d(n);
        random_device rdev;
        mt19937 rng(rdev());
        int iter = 1000000;
        for(int i = 0; i < iter; i++)
            v[d(rng)]++;
        for(int i = 0; i < n; i++)
            LOG(i, v[i]);
    }
}















struct BioIKKinematicsPlugin : kinematics::KinematicsBase
{
    vector<string> joint_names, link_names;
    MoveItRobotModelConstPtr robot_model;
    const moveit::core::JointModelGroup* joint_model_group;
    mutable random_numbers::RandomNumberGenerator rng;
    mutable vector<Frame> tipFrames;
    mutable vector<unique_ptr<IKSolver>> solvers;

    BioIKKinematicsPlugin()
    {
        test();
    }

    virtual const vector<string>& getJointNames() const
    {
        LOG_FNC();
        return joint_names;
    }
    
    virtual const vector<string>& getLinkNames() const
    {
        LOG_FNC();
        return link_names;
    }
    
    virtual bool getPositionFK(
                        const vector<string>& link_names, 
                        const vector<double>& joint_angles, 
                        vector<geometry_msgs::Pose>& poses) const
    {
        LOG_FNC();
        return false;
    }
    
    virtual bool getPositionIK(
                        const geometry_msgs::Pose& ik_pose, 
                        const vector<double>& ik_seed_state, 
                        vector<double>& solution, 
                        moveit_msgs::MoveItErrorCodes& error_code, 
                        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }
    
    /*virtual bool getPositionIK(
                        const vector<geometry_msgs::Pose>& ik_poses,
                        const vector<double>& ik_seed_state,
                        vector<vector<double>>&solutions,
                        kinematics::KinematicsResult& result,
                        const kinematics::KinematicsQueryOptions& options) const
    {
        LOG_FNC();
        return false;
    }*/
    
    vector<Eigen::Affine3d> tip_reference_frames;

    IKParams ikparams;
    
    bool load(string robot_description, string group_name)
    {
        LOG_FNC();

        rdf_loader::RDFLoader rdf_loader(robot_description_);
        auto srdf = rdf_loader.getSRDF();
        auto urdf_model = rdf_loader.getURDF();

        if (!urdf_model || !srdf)
        {
            LOG("URDF and SRDF must be loaded for KDL kinematics solver to work.");
            return false;
        }
        
        robot_model.reset(new robot_model::RobotModel(urdf_model, srdf));
        
        joint_model_group = robot_model->getJointModelGroup(group_name);
        if (!joint_model_group)
        {
            LOG("failed to get joint model group");
            return false;
        }

        joint_names.clear();
        
        for(auto* joint_model : joint_model_group->getJointModels())
            if(joint_model->getName() != base_frame_ && joint_model->getType() != moveit::core::JointModel::UNKNOWN && joint_model->getType() != moveit::core::JointModel::FIXED)
                joint_names.push_back(joint_model->getName());

        auto tips2 = tip_frames_;
        joint_model_group->getEndEffectorTips(tips2);
        if(!tips2.empty()) tip_frames_ = tips2;

        link_names = tip_frames_; 

        for(auto& n : joint_names) LOG("joint", n);
        for(auto& n : link_names) LOG("link", n);

        moveit::core::RobotState robot_state(robot_model);
        tip_reference_frames.clear();
        for(auto& tip : tip_frames_)
        {
            auto ref = robot_state.getGlobalLinkTransform(getBaseFrame());
            tip_reference_frames.push_back(ref);
        }
        
        solvers.clear();
        
        ros::NodeHandle node_handle("~");
        string rdesc;
        node_handle.searchParam(robot_description_, rdesc);
        node_handle = NodeHandle(rdesc + "_kinematics/" + group_name_);
        
        //LOG_VAR(node_handle.param("kinematics_solver_search_resolution", 0.01234));
        
        /*vector<size_t> mutable_genes;
        for(auto& vname : joint_model_group->getVariableNames())
            mutable_genes.push_back(robot_model->getVariableIndex(vname));*/
        
        vector<size_t> mutable_genes;
        ModelInfo model_info(robot_model, tip_frames_);
        
        /*for(auto& vname : joint_model_group->getVariableNames())
        {
            size_t i = robot_model->getVariableIndex(vname);
            for(size_t j : model_info.getActiveVariables())
            {
                if(i == j)
                {
                    mutable_genes.push_back(i);
                }
            }
        }*/
        
        for(size_t j : model_info.getActiveVariables())
        {
            for(auto& vname : joint_model_group->getVariableNames())
            {
                size_t i = robot_model->getVariableIndex(vname);
                if(i == j)
                {
                    mutable_genes.push_back(i);
                }
            }
        }
            
        ikparams.robot_model = robot_model;
        ikparams.node_handle = node_handle;
        ikparams.tip_frames = tip_frames_;
        ikparams.active_variables = mutable_genes;
        
        
        
        
        
        /*
        for(auto& tip : tip_frames_)
            LOG_VAR(tip);
        
        vector<const moveit::core::JointModelGroup*> sub_groups;
        joint_model_group->getSubgroups(sub_groups);
        sub_groups.push_back(joint_model_group);
        for(auto* g : sub_groups)
        {
            LOG_VAR(g->getName());
            LOG_VAR(g->getEndEffectorName());
            LOG_VAR(g->isEndEffector());
            
            vector<string> tips;
            g->getEndEffectorTips(tips);
            for(auto& t : tips)
                LOG_VAR(t);
        }
        
        
        for(auto& end_effector : robot_model->getSRDF()->getEndEffectors())
        {
            bool ok = false;
            for(auto& tip_name : tip_frames_)
                if(tip_name == end_effector.parent_link)
                    ok = true;
            if(!ok) continue;
            
            LOG_VAR(end_effector.name_);
            LOG_VAR(end_effector.component_group_);
            LOG_VAR(end_effector.parent_group_);
            LOG_VAR(end_effector.parent_link_);
        }
        */
        
        ikparams.tip_infos.clear();
        for(auto& tip_name : tip_frames_)
        {
            for(auto& end_effector : robot_model->getSRDF()->getEndEffectors())
            {
                if(tip_name == end_effector.parent_link_)
                {
                    LOG_VAR(end_effector.name_);
                    LOG_VAR(end_effector.component_group_);
                    LOG_VAR(end_effector.parent_group_);
                    LOG_VAR(end_effector.parent_link_);
                    
                    ros::NodeHandle node_handle("~");
                    string rdesc;
                    node_handle.searchParam(robot_description_, rdesc);
                    node_handle = NodeHandle(rdesc + "_kinematics/" + end_effector.component_group_);
            
                    ikparams.tip_infos.emplace_back();
                    
                    //ikparams.tip_infos.back().position_only_ik = node_handle.param("position_only_ik", false);
                    //ikparams.tip_infos.back().weight = node_handle.param("weight", 1.0);
                    
                    node_handle.param("position_only_ik", ikparams.tip_infos.back().position_only_ik, false);
                    node_handle.param("weight", ikparams.tip_infos.back().weight, 1.0);
                    
                    LOG_VAR(ikparams.tip_infos.back().position_only_ik);
                }
            }
        }
        
        
        
        
        
        //ERROR("A");
        

        return true;
    }
    
    
    
    
    
    
    virtual bool initialize(const std::string &robot_description, const std::string &group_name, const std::string &base_frame, const std::string &tip_frame, double search_discretization)
    {
        LOG_FNC();
        std::vector<std::string> tip_frames;
        tip_frames.push_back(tip_frame);
        initialize(robot_description, group_name, base_frame, tip_frames, search_discretization);
        return true;
    }
    
    virtual bool initialize(const std::string& robot_description,
                          const std::string& group_name,
                          const std::string& base_frame,
                          const std::vector<std::string>& tip_frames,
                          double search_discretization)
    {
        LOG_FNC();
        setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);
        load(robot_description, group_name);
        return true;
    }

    // TODO: implement
    virtual bool searchPositionIK(
        const geometry_msgs::Pose& ik_pose, 
        const vector<double>& ik_seed_state, 
        double timeout, std::vector<double>& solution, 
        moveit_msgs::MoveItErrorCodes& error_code, 
        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }
    
    virtual bool searchPositionIK(
        const geometry_msgs::Pose& ik_pose, 
        const vector<double>& ik_seed_state, 
        double timeout, 
        const vector<double> &consistency_limits, 
        vector<double>& solution, 
        moveit_msgs::MoveItErrorCodes &error_code, 
        const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }
    
    virtual bool searchPositionIK(
        const geometry_msgs::Pose& ik_pose, 
        const vector<double>& ik_seed_state,
        double timeout, 
        vector<double> &solution, 
        const IKCallbackFn& solution_callback, 
        moveit_msgs::MoveItErrorCodes& error_code, 
        const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }
    
    virtual bool searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector< double > &ik_seed_state, double timeout, const std::vector< double > &consistency_limits, std::vector< double > &solution, const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code, const kinematics::KinematicsQueryOptions &options=kinematics::KinematicsQueryOptions()) const
    {
        LOG_FNC();
        return false;
    }

    mutable vector<double> state;
    mutable vector<vector<double>> results;
    mutable vector<double> last_solution;

    virtual bool searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                const std::vector<double>& ik_seed_state,
                                double timeout,
                                const std::vector<double>& consistency_limits,
                                std::vector<double>& solution,
                                const IKCallbackFn& solution_callback,
                                moveit_msgs::MoveItErrorCodes& error_code,
                                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions(),
                                const moveit::core::RobotState* context_state = NULL) const
    {
        LOG_FNC();

        FNPROFILE();
        
        

        // get init state
        state.resize(robot_model->getVariableCount());
        robot_model->getVariableDefaultPositions(state);
        LOG_VAR(context_state);
        if(context_state)
            for(size_t i = 0; i < robot_model->getVariableCount(); i++)
                state[i] = context_state->getVariablePositions()[i];
        
        // temporary compatability hack
        // ik_seed_state depends on MoveIt version
        /*solution = ik_seed_state;
        if(last_solution.size() == solution.size())
        {
            solution = last_solution;
        }
        {
            int i = 0;
            for(auto& joint_name : getJointNames())
            {
                auto* joint_model = robot_model->getJointModel(joint_name);
                if(!joint_model) continue;
                for(size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
                {
                    double v = solution.at(i++);
                    if(!isfinite(v)) ERROR("joint init value invalid", joint_name, vi, v);
                    state.at(joint_model->getFirstVariableIndex() + vi) = v;
                }
            }
        }*/
        
        // transform tips to baseframe
        tipFrames.clear();
        LOG_VAR(getBaseFrame());
        for(size_t i = 0; i < ik_poses.size(); i++)
        {
            //auto p = Frame(ik_poses[i]);
            //auto r = Frame(tip_reference_frames[i]);
            Eigen::Affine3d p, r;
            tf::poseMsgToEigen(ik_poses[i], p);
            if(context_state)
                r = context_state->getGlobalLinkTransform(getBaseFrame());
            else
                r = tip_reference_frames[i];
            tipFrames.emplace_back(r * p);
        }
        
        // init parallel solvers
        //auto concurrency = thread::hardware_concurrency();
        //auto concurrency = omp_get_num_threads();
        //int concurrency = 8;
        //int concurrency = 4;
        //int concurrency = 64;
        int concurrency = IKSolver::concurrency();
        while(solvers.size() < concurrency)
        {
            solvers.emplace_back(new IKSolver(ikparams));
        }
        results.resize(solvers.size());
        
        
        
        
        
        double t0 = ros::Time::now().toSec();
        
        while(true)
        {
        
            atomic<int> finished;
            finished = 0;

            #pragma omp parallel for
            for(int i = 0; i < solvers.size(); i++)
            {
                solvers[i]->initialize(state, tipFrames);
                
                //solvers[i]->solve();
                
                for(; ros::Time::now().toSec() - t0 < timeout * 0.5 && finished == 0; )
                //for(int iter = 0; iter < 25; iter++)
                {
                    solvers[i]->step();
                    
                    //double p_dist_2 = 0;
                    //double r_dist_cos = 0;
                    
                    double p_dist = 0;
                    double r_dist = 0;
                    
                    auto& solution_tip_frames = solvers[i]->getSolutionTipFrames();
                    
                    for(int i = 0; i < tipFrames.size(); i++)
                    {
                        auto& a = solution_tip_frames[i];
                        auto& b = tipFrames[i];
                        //LOG_VAR(a);
                        //LOG_VAR(b);
                        p_dist = max(p_dist, (b.pos - a.pos).length());
                        r_dist = max(r_dist, b.rot.angle(a.rot));
                    }
                    
                    double max_p_dist = 0.0001;
                    double max_r_dist = 0.5;
                    
                    //double max_p_dist_2 = max_p_dist * max_p_dist;
                    //double max_r_dist_cos = cos(max_r_dist / 180 * M_PI);
                    
                    if(p_dist <= max_p_dist && r_dist >= max_r_dist)
                    //if(p_dist_2 <= max_p_dist && r_dist_cos >= max_r_dist_cos)
                    //if(solvers[i]->getSolutionFitness() < 0.00001)
                    //if(solvers[i]->getSolutionFitness() < 0.000001)
                    {
                        finished = 1;
                        break;
                    }
                }
                
                {
                    auto& state = solvers[i]->getSolution();
                    auto& solution = results[i];
                    solution.clear();
                    for(auto& joint_name : getJointNames())
                    {
                        auto* joint_model = robot_model->getJointModel(joint_name);
                        if(!joint_model) continue;
                        for(size_t vi = 0; vi < joint_model->getVariableCount(); vi++)
                        {
                            solution.push_back(state.at(joint_model->getFirstVariableIndex() + vi));
                        }
                    }
                }
            }

            double best_fitness = solvers[0]->getSolutionFitness();
            int best_solution = 0;
            for(int i = 1; i < solvers.size(); i++)
            {
                if(solvers[i]->getSolutionFitness() < best_fitness)
                {
                    best_fitness = solvers[i]->getSolutionFitness();
                    best_solution = i;
                }
            }
            solution = results[best_solution];
            last_solution = solution;

            {
                if(!solution_callback.empty())
                    solution_callback(ik_poses.front(), solution, error_code);
                else
                    error_code.val = error_code.SUCCESS;
            }
            
            //if(!solution_callback.empty() && !options.return_approximate_solution && error_code.val != error_code.SUCCESS)
            //if(!solution_callback.empty())
            //{
            //    LOG("REPEAT");
            //    continue;
            //}
            
            break;
        
        }
        
        profilerlog();
        
        return error_code.val == error_code.SUCCESS;
    }
    
    // MoveIt version compatability
    // API changed from "const bool" to "bool"
    // Automatically select correct return type
    typedef decltype(((kinematics::KinematicsBase*)0)->supportsGroup(0)) supportsGroup_Result; 
    
    //virtual const bool supportsGroup(const moveit::core::JointModelGroup* jmg, std::string* error_text_out = NULL) const
    virtual supportsGroup_Result supportsGroup(const moveit::core::JointModelGroup* jmg, string* error_text_out = 0) const
    {
        LOG_FNC();
        LOG_VAR(jmg->getName());
        return true;
    }
    
};

}

#undef LOG
#undef ERROR

PLUGINLIB_EXPORT_CLASS(bio_ik_kinematics_plugin::BioIKKinematicsPlugin, kinematics::KinematicsBase);




