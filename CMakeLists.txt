cmake_minimum_required(VERSION 2.8)
set(SOLUTIONTITLE Lab0 CACHE TYPE STRING)
project(${SOLUTIONTITLE})
file(GLOB FILES_SRC
        "src/*.cpp"
        "src/*.h"
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(VVRFRAMEWORK_DIR "" CACHE PATH "Location of VVR Framework")

MESSAGE( STATUS "variable is: " ${VVRFRAMEWORK_DIR} )

include_directories(${CMAKE_SOURCE_DIR})

#MESSAGE( STATUS "variable is: " ${VVRFRAMEWORK_DIR} )

#set(CMAKE_INCLUDE_CURRENT_DIR ON)
#set(CMAKE_PREFIX_PATH "/home/andreas/Libraries/Qt/5.12.0/gcc_64/")
#set(CMAKE_AUTOMOC  ON)
#
#find_package(Qt5Core REQUIRED)
#find_package(Qt5Widgets REQUIRED)
#find_package(Qt5Gui REQUIRED)
#find_package(Qt5Network REQUIRED)
#find_package(Qt5Sql REQUIRED)
#find_package(Qt5Xml REQUIRED)
#find_package(Qt5PrintSupport REQUIRED)
#
#target_link_libraries(Qt5::Core)
#target_link_libraries(Qt5::Widgets)
#target_link_libraries(Qt5::Network)
#target_link_libraries(Qt5::Gui)
#target_link_libraries(Qt5::Sql)
#target_link_libraries(Qt5::PrintSupport)

include_directories(include)
include_directories(/usr/local/include)
include_directories(/usr/local/include/opencv4)
include_directories(/usr/local/include/VVRScene)
include_directories(/usr/local/include/GeoLib)
include_directories(/usr/local/include/MathGeoLib)
include_directories(/usr/include/eigen3)
include_directories(/home/andreas/Libraries/Qt/5.12.0/gcc_64/lib/cmake)

add_executable(${SOLUTIONTITLE} ${FILES_SRC} src/surfaceReconstruction.cpp src/surfaceReconstruction.h src/ImageRGBD.cpp src/ImageRGBD.h src/PointCloud.cpp src/PointCloud.h src/ImageRGB.cpp src/ImageRGB.h src/generic.h src/kdTree.cpp src/kdTree.h)
target_link_libraries(
        ${SOLUTIONTITLE}
        -lVVRScene
        -lGeoLib
        -lMathGeoLib
        -lopencv_core
        -lopencv_highgui
        -lopencv_imgproc
        -lopencv_imgcodecs
)


#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_DIR}/lib" ${CMAKE_BINARY_DIR}/$<CONFIG>)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_DIR}/GeoLib/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_DIR}/MathGeoLib/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)


#include_directories(${CMAKE_SOURCE_DIR})
#include_directories(${VVRFRAMEWORK_DIR})
#include_directories(${VVRFRAMEWORK_DIR}/GeoLib)
#include_directories(${VVRFRAMEWORK_DIR}/MathGeoLib/src)
#add_executable(${SOLUTIONTITLE} ${FILES_SRC})
#target_link_libraries(
#${SOLUTIONTITLE}
#${VVRFRAMEWORK_LIBRARY_DIR}/VVRScene/Debug/VVRScene_d.lib
#${VVRFRAMEWORK_LIBRARY_DIR}/GeoLib/Debug/GeoLib_d.lib
#${VVRFRAMEWORK_LIBRARY_DIR}/MathGeoLib/Debug/MathGeoLib_d.lib
#)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_LIBRARY_DIR}/VVRScene/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_LIBRARY_DIR}/GeoLib/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)
#add_custom_command(TARGET ${SOLUTIONTITLE} POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy_directory "${VVRFRAMEWORK_LIBRARY_DIR}/MathGeoLib/$<CONFIG>" ${CMAKE_BINARY_DIR}/$<CONFIG>)