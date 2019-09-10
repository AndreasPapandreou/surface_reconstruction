cmake_minimum_required(VERSION 2.8.11)
project(VVRFramework)

### Warning options #####################################################################
option(DISABLE_WARNINGS "Disable Warnings MSVC/GCC" ON)
if(DISABLE_WARNINGS)
 set(MSVC_WARN_LEVEL /W0)
 set(GCC_WARN_LEVEL -w)
else()
 set(MSVC_WARN_LEVEL /W3)
 set(GCC_WARN_LEVEL -Wall -Wno-long-long -pedantic)
endif(DISABLE_WARNINGS)
if(MSVC)
  if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
    string(REGEX REPLACE "/W[0-4]" "${MSVC_WARN_LEVEL}" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${MSVC_WARN_LEVEL}")
  endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GCC_WARN_LEVEL} -std=c++11")
endif()
#########################################################################################

### Find Boost ##########################################################################
option(VVR_USE_BOOST "Use boost components" OFF)
option(VVR_COPY_BOOST_HEADERS "Copy Boost headers to installation dir" OFF)
option(VVR_COPY_BOOST_LIBS "Copy Boost libs to installation dir" OFF)
if(${VVR_USE_BOOST})
  set(Boost_NO_BOOST_CMAKE ON)
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
  set(Boost_NEEDED_LIBS log log_setup system filesystem date_time thread chrono regex)
  find_package(Boost 1.55.0 COMPONENTS ${Boost_NEEDED_LIBS} REQUIRED)
  add_definitions(-DVVR_USE_BOOST)
endif()
#########################################################################################

### Build subdirs #######################################################################
#add_subdirectory(GLFW)
add_subdirectory(GeoLib)
add_subdirectory(MathGeoLib)
add_subdirectory(VVRScene)
#########################################################################################

#########################################################################################
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(TARGET VVRScene PROPERTY FOLDER "Libraries")
set_property(TARGET GeoLib PROPERTY FOLDER "Libraries")
set_property(TARGET MathGeoLib PROPERTY FOLDER "Libraries")
#set_property(TARGET glfw PROPERTY FOLDER "Libraries")
#########################################################################################

#########################################################################################
set_target_properties(VVRScene PROPERTIES DEBUG_POSTFIX _d)
set_target_properties(MathGeoLib PROPERTIES DEBUG_POSTFIX _d)
set_target_properties(GeoLib PROPERTIES DEBUG_POSTFIX _d)
#set_target_properties(glfw PROPERTIES DEBUG_POSTFIX _d)
#########################################################################################

### INSTALL #############################################################################
if(MSVC)
  # Gather and install dependencies
  IF(${VVR_COPY_BOOST_LIBS})
      FOREACH(module ${Boost_NEEDED_LIBS})
              STRING(TOUPPER ${module} module_u)
              SET(lib_name_r Boost_${module_u}_LIBRARY_RELEASE)
              SET(lib_name_d Boost_${module_u}_LIBRARY_DEBUG)
              STRING(REPLACE ".lib" ".dll" lib_r_dlls ${${lib_name_r}})
              STRING(REPLACE ".lib" ".dll" lib_d_dlls ${${lib_name_d}})
              SET(deps ${deps} ${lib_r_dlls} ${lib_d_dlls} )
      ENDFOREACH(module ${Boost_NEEDED_LIBS})
  ENDIF()
   IF(${VVR_COPY_BOOST_HEADERS})
      install (DIRECTORY ${Boost_INCLUDE_DIR}/boost DESTINATION "include/")
  ENDIF()
endif(MSVC)

### Install headers
install (DIRECTORY "GeoLib" DESTINATION "include" FILES_MATCHING PATTERN "*.h")
install (DIRECTORY "VVRScene" DESTINATION "include" FILES_MATCHING PATTERN "*.h")
install (FILES ${deps} DESTINATION "lib")

### msvc runtime 
set(CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS_SKIP TRUE)
include(InstallRequiredSystemLibraries)
install(PROGRAMS ${CMAKE_INSTALL_SYSTEM_RUNTIME_LIBS} DESTINATION "lib")

INCLUDE(CPack)
#########################################################################################
