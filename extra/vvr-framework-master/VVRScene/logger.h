#ifdef VVR_USE_BOOST

#ifndef VVR_LOGGER_H
#define VVR_LOGGER_H


#define BOOST_ALL_NO_LIB
#define BOOST_LOG_DYN_LINK 1 // necessary when linking the boost_log library dynamically

#include "vvrscenedll.h"
#include <boost/log/trivial.hpp>
#include <boost/log/sources/global_logger_storage.hpp>

// the logs are also written to LOGFILE
#define LOGFILE "logfile.log"

// just log messages with severity >= SEVERITY_THRESHOLD are written
#define SEVERITY_THRESHOLD logging::trivial::info

// register a global logger
BOOST_LOG_GLOBAL_LOGGER(logger, boost::log::sources::severity_logger_mt<boost::log::trivial::severity_level>)

// just a helper macro used by the macros below - don't use it in your code
#define LOG(severity) BOOST_LOG_SEV(logger::get(),boost::log::trivial::severity)

// ===== log macros =====
#define LOG_TRACE   LOG(trace)
#define LOG_DEBUG   LOG(debug)
#define LOG_INFO    LOG(info)
#define LOG_WARNING LOG(warning)
#define LOG_ERROR   LOG(error)
#define LOG_FATAL   LOG(fatal)

namespace vvr {
    void VVRScene_API loge(const std::string &msg);
    void VVRScene_API logw(const std::string &msg);
    void VVRScene_API logi(const std::string &msg);
}

#endif // VVR_LOGGER_H
#endif // VVR_USE_BOOST
