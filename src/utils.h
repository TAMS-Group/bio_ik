// Bio IK for ROS
// (c) 2016-2017 Philipp Ruppel

#pragma once

#include <iostream>
#include <csignal>

#include <ros/ros.h>

#include <atomic>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <typeindex>

#include <stdlib.h>
#include <malloc.h>

#include <tf_conversions/tf_kdl.h>

#include <XmlRpcException.h>

//#include <link.h>

//#include <boost/align/aligned_allocator.hpp>
//#include <Eigen/Eigen>

namespace bio_ik
{



//#define ENABLE_CPP_OPTLIB


#define ENABLE_LOG


#define ENABLE_PROFILER




// logging

//#define LOG_STREAM (std::cerr << std::fixed)
//#define LOG_STREAM (std::cerr << std::scientific)
#define LOG_STREAM (std::cerr)

template<class T>
inline void vprint(std::ostream& s, const T& a) {
    s << a << std::endl;
}
template<class T, class... AA>
inline void vprint(std::ostream& s, const T& a, const AA&... aa) {
    s << a << " ";
    vprint(s, aa...);
};


#define LOG2(...) vprint(LOG_STREAM, "ikbio ", __VA_ARGS__)

#ifdef ENABLE_LOG
#define LOG(...) LOG2(__VA_ARGS__)
#else
#define LOG(...)
#endif

#define LOG_VAR(v) LOG2(#v, (v));



//#define LOG_FNC() LOG("fun", __func__, __LINE__)
#define LOG_FNC()



// show error and abort
// #define ERROR(...) { LOG("ERROR", __VA_ARGS__); exit(-1); }
// #define ERROR(a, ...) { LOG(a, __VA_ARGS__); LOG_STREAM.flush(); throw std::runtime_error(a); }
#define ERROR(...) { LOG2(__VA_ARGS__); LOG_STREAM.flush(); std::stringstream ss; vprint(ss, __VA_ARGS__); throw std::runtime_error(ss.str()); }
// #define ERROR(...) { LOG_ALWAYS(__VA_ARGS__); std::raise(SIGINT); }







// profiler

#ifdef ENABLE_PROFILER

    // embeddable sampling profiler

    // profiled block or function
    struct ProfilerBin
    {
        const char* volatile name; // name of scope or function, also used as indicator if it is currently being executed
        std::atomic<int> counter; // only used by CounterScope / COUNTERPROFILER
        ProfilerBin() : name(0) { }
    };

    // allocate globally unique profiler buffer via template
    template<class force_weak_linker_symbol = void>
    ProfilerBin* getProfilerBuffer()
    {
        static std::vector<ProfilerBin> buffer(10000);
        return buffer.data();
    }

    // reserve profiler buffer segment for current compilation unit
    template<class force_weak_linker_symbol = void>
    ProfilerBin* getProfilerSegment()
    {
        static size_t index = 0;
        return getProfilerBuffer() + (index++) * 20;
    }
    static ProfilerBin* profiler_segment = getProfilerSegment();

    // identifies currently profiled thread
    // null if profiler is disabled
    struct ProfilerInfo
    {
        void* stack_begin;
        void* stack_end;
    };

    // declare globally unique profiler info via template
    template<class force_weak_linker_symbol = void>
    ProfilerInfo& getProfilerInfo()
    {
        static ProfilerInfo info;
        return info;
    }
    static ProfilerInfo& profiler_info = getProfilerInfo();

    // profiles a scope or function
    template<size_t ID>
    struct ProfilerScope
    {
        __attribute__((always_inline)) inline ProfilerScope(const char* name)
        {
            if(profiler_info.stack_begin == 0) return;
            if(this < profiler_info.stack_begin || this > profiler_info.stack_end) return;
            profiler_segment[ID].name = name;
        }
        __attribute__((always_inline)) inline ~ProfilerScope()
        {
            if(profiler_info.stack_begin == 0) return;
            if(this < profiler_info.stack_begin || this > profiler_info.stack_end) return;
            profiler_segment[ID].name = 0;
        }
    };
    #define FNPROFILER() volatile ProfilerScope<__COUNTER__> _profilerscope(__func__);
    #define BLOCKPROFILER(name) volatile ProfilerScope<__COUNTER__> _profilerscope(name);

    // per-thread profiling
    struct ThreadScope
    {
        size_t id;
        __attribute__((always_inline)) inline ThreadScope(const char* name, size_t id) : id(id)
        {
            if(profiler_info.stack_begin == 0) return;
            profiler_segment[id].name = name;
        }
        __attribute__((always_inline)) inline ~ThreadScope()
        {
            if(profiler_info.stack_begin == 0) return;
            profiler_segment[id].name = 0;
        }
    };
    #define THREADPROFILER(name, id) static const char* _threadscope_names[] = {name "0", name "1", name "2", name "3"}; volatile ThreadScope _threadscope(_threadscope_names[id], __COUNTER__ + id); (__COUNTER__,__COUNTER__,__COUNTER__,__COUNTER__,__COUNTER__);

    // profiling across multiple threads
    struct CounterScope
    {
        size_t id;
        __attribute__((always_inline)) inline CounterScope(const char* name, size_t id) : id(id)
        {
            if(profiler_info.stack_begin == 0) return;
            if((profiler_segment[id].counter++) == 0)
                profiler_segment[id].name = name;
        }
        __attribute__((always_inline)) inline ~CounterScope()
        {
            if(profiler_info.stack_begin == 0) return;
            if((--profiler_segment[id].counter) == 0)
                profiler_segment[id].name = 0;
        }
    };
    #define COUNTERPROFILER(name) volatile CounterScope _counterscope(name, __COUNTER__);

    // starts profiler and periodically writes results to log
    struct Profiler
    {
        std::thread thread;
        volatile int exit_flag;
        Profiler()
        {
            pthread_attr_t attr;
            pthread_getattr_np(pthread_self(), &attr);
            void* stack_addr;
            size_t stack_size;
            pthread_attr_getstack(&attr, &stack_addr, &stack_size);
            profiler_info.stack_begin = stack_addr;
            profiler_info.stack_end = (char*)stack_addr + stack_size;
            const size_t maxbin = 1000;
            static std::mutex mutex;
            static std::unordered_map<const char*, size_t> samples;
            exit_flag = 0;
            std::thread t([this] ()
            {
                auto* profiler_bins = getProfilerBuffer();
                while(true)
                {
                    for(int iter = 0; iter < 100; iter++)
                    {
                        for(int iter = 0; iter < 100; iter++)
                        {
                            int i = rand() % maxbin;
                            const char* p = profiler_bins[i].name;
                            if(p) samples[p]++;
                        }
                        if(exit_flag) break;
                        std::this_thread::sleep_for(std::chrono::duration<size_t, std::micro>(rand() % 1000));
                    }
                    {
                        double thistime = ros::WallTime::now().toSec();
                        static double lasttime = 0.0;
                        if(thistime < lasttime + 1) continue;
                        lasttime = thistime;
                        static std::vector<std::pair<const char*, size_t>> data;
                        data.clear();
                        for(auto& p : samples)
                            data.push_back(p);
                        std::sort(data.begin(), data.end(), [] (const std::pair<const char*, size_t>& a, const std::pair<const char*, size_t>& b) { return a.second > b.second; });
                        LOG("");
                        LOG("profiler");
                        for(auto& d : data)
                        {
                            double v = d.second * 100.0 / data[0].second;
                            char s[32];
                            sprintf(s, "%6.2f%%", v);
                            LOG("p", s, d.first);
                        }
                        LOG("");
                    }
                    if(exit_flag) break;
                }
            });
            std::swap(thread, t);
        }
        ~Profiler()
        {
            exit_flag = true;
            thread.join();
        }
        static void start()
        {
            static Profiler profiler;
        }
    };

#else

    #define FNPROFILER()
    #define BLOCKPROFILER(name)
    #define THREADPROFILER(name, id)
    #define COUNTERPROFILER(name)

    struct Profiler
    {
        static void start()
        {
        }
    };

#endif



















__attribute__((always_inline))
inline double mix(double a, double b, double f)
{
    return a * (1.0 - f) + b * f;
}




__attribute__((always_inline))
inline double clamp(double v, double lo, double hi)
{
    if(v < lo) v = lo;
    if(v > hi) v = hi;
    return v;
}

__attribute__((always_inline))
inline double clamp2(double v, double lo, double hi)
{
    if(__builtin_expect(v < lo, 0)) v = lo;
    if(__builtin_expect(v > hi, 0)) v = hi;
    return v;
}




__attribute__((always_inline))
inline double smoothstep(float a, float b, float v)
{
    v = clamp((v - a) / (b - a), 0.0, 1.0);
    return v * v * (3.0 - 2.0 * v);
}


__attribute__((always_inline))
inline double sign(double f)
{
    if(f < 0.0) f = -1.0;
    if(f > 0.0) f = +1.0;
    return f;
}







template<class t>
class linear_int_distribution
{
    std::uniform_int_distribution<t> base;
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





struct XORShift64
{
    uint64_t v;
public:
    XORShift64() : v(88172645463325252ull)
    {
    }
    __attribute__((always_inline))
    inline uint64_t operator () ()
    {
        v ^= v << 13;
        v ^= v >> 7;
        v ^= v << 17;
        return v;
    }
};








// class factory
//
// registering a class:
//   static Factory<Base>::Class<Derived> reg("Derived");
//
// instantiation:
//   Base* obj = Factory<Base>::create("Derived");
//
// cloning and object:
//   p = Factory<Base>::clone(o);
//
template<class BASE, class... ARGS>
class Factory
{
    typedef BASE* (*Constructor)(ARGS...);
    struct ClassBase
    {
        std::string name;
        std::type_index type;
        virtual BASE* create(ARGS... args) const = 0;
        virtual BASE* clone(const BASE*) const = 0;
        ClassBase() : type(typeid(void))
        {
        }
    };
    typedef std::set<ClassBase*> MapType;
    static MapType& classes()
    {
        static MapType ff;
        return ff;
    }
public:
    template<class DERIVED>
    struct Class : ClassBase
    {
        BASE* create(ARGS... args) const
        {
            return new DERIVED(args...);
        }
        BASE* clone(const BASE* o) const
        {
            return new DERIVED(*(const DERIVED*)o);
        }
        Class(const std::string& name)
        {
            this->name = name;
            this->type = typeid(DERIVED);
            classes().insert(this);
        }
        ~Class()
        {
            classes().erase(this);
        }
    };
    static BASE* create(const std::string& name, ARGS... args)
    {
        for(auto* f : classes())
            if(f->name == name)
                return f->create(args...);
        ERROR("class not found", name);
    }
    template<class DERIVED>
    static DERIVED* clone(const DERIVED* o)
    {
        for(auto* f : classes())
            if(f->type == typeid(*o))
                return (DERIVED*)f->clone(o);
        ERROR("class not found", typeid(*o).name());
    }
};





// Alloctes memory properly aligned for SIMD operations
template<class T, size_t A>
struct aligned_allocator : public std::allocator<T>
{
    typedef size_t size_type;
    typedef ptrdiff_t difference_type;
    typedef T* pointer;
    typedef const T* const_pointer;
    typedef T& reference;
    typedef const T& const_reference;
    typedef T value_type;
    T* allocate(size_t s, const void* hint = 0)
    {
        void* p;
        if(posix_memalign(&p, A, sizeof(T) * s + 64)) throw std::bad_alloc();
        return (T*)p;
    }
    void deallocate(T* ptr, size_t s)
    {
        free(ptr);
    }
    template<class U>
    struct rebind
    {
        typedef aligned_allocator<U, A> other;
    };
};


// std::vector typedef with proper memory alignment for SIMD operations
template<class T>
struct aligned_vector : std::vector<T, aligned_allocator<T, 32>>
{
};






// Helper class for reading structured data from ROS parameter server
class XmlRpcReader
{
    typedef XmlRpc::XmlRpcValue var;
    var& v;
public:
    XmlRpcReader(var& v) : v(v)
    {
    }
private:
    XmlRpcReader at(int i)
    {
        return v[i];
    }
    void conv(bool& r)
    {
        r = (bool)v;
    }
    void conv(double& r)
    {
        r = (v.getType() == var::TypeInt) ? ((double)(int)v) : ((double)v);
    }
    void conv(tf::Vector3& r)
    {
        double x, y, z;
        at(0).conv(x);
        at(1).conv(y);
        at(2).conv(z);
        r = tf::Vector3(x, y, z);
    }
    void conv(tf::Quaternion& r)
    {
        double x, y, z, w;
        at(0).conv(x);
        at(1).conv(y);
        at(2).conv(z);
        at(3).conv(w);
        r = tf::Quaternion(x, y, z, w).normalized();
    }
    void conv(std::string& r)
    {
        r = (std::string)v;
    }
public:
    template<class T>
    void param(const char* key, T& r)
    {
        if(!v.hasMember(key)) return;
        try
        {
            XmlRpcReader(v[key]).conv(r);
        }
        catch(const XmlRpc::XmlRpcException& e)
        {
            LOG(key);
            throw;
        }
    }
};










}
