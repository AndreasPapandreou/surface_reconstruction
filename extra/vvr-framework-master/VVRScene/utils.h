#ifndef VVR_UTILS_H
#define VVR_UTILS_H

#include "vvrscenedll.h"
#include <string>
#include <iostream>
#include <sstream>
#include <vector>

#define echo(x) std::cout<<#x<<" = "<<x<<std::endl
#define msg(x) std::cout<<x<<std::endl
#define SQUARE(x) ((x)*(x))
#define SSTR(x) dynamic_cast< std::ostringstream & >(( std::ostringstream() << std::dec << x ) ).str()

#ifdef VVR_USE_BOOST

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#define VAR_CLASS_DEFS(x)                                                              \
    typedef boost::shared_ptr< x > Ptr;                                                 \
    typedef boost::shared_ptr<const x > ConstPtr;                                        \
    static inline boost::shared_ptr<x> Make() {                                           \
    return boost::make_shared<x>(); }                                                      \
    template<typename T0>                                                                   \
    static inline boost::shared_ptr<x> Make(const T0& t0) {                                  \
    return boost::make_shared<x>(t0); }                                                       \
    template<typename T0, typename T1>                                                         \
    static inline boost::shared_ptr<x> Make(const T0& t0, const T1& t1) {                       \
    return boost::make_shared<x>(t0, t1); }                                                      \
    template<typename T0, typename T1, typename T2>                                               \
    static inline boost::shared_ptr<x> Make(const T0& t0, const T1& t1, const T2& t2) {            \
    return boost::make_shared<x>(t0, t1, t2); }                                                     \
    template<typename T0, typename T1, typename T2, typename T3>                                     \
    static inline boost::shared_ptr<x> Make(const T0& t0, const T1& t1, const T2& t2, const T3& t3) { \
    return boost::make_shared<x>(t0, t1, t2, t3); }
#endif

namespace vvr
{
float VVRScene_API getSeconds();
double VVRScene_API normalizeAngle(double angle);
std::string VVRScene_API getExePath();
std::string VVRScene_API getBasePath();
bool VVRScene_API mkdir(const std::string &path);
bool VVRScene_API fileExists(const std::string &filename);
bool VVRScene_API dirExists(const std::string &dirname);
void VVRScene_API split(const std::string &s, char delim, std::vector<std::string> &elems);
std::vector<std::string> VVRScene_API split(const std::string &s, char delim);
std::string VVRScene_API zpn(int num, int len);
}

#endif // VVR_UTILS_H
