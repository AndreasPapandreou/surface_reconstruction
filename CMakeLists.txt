cmake_minimum_required(VERSION 2.8)
set(SOLUTIONTITLE Lab0 CACHE TYPE STRING)
project(${SOLUTIONTITLE})
file(GLOB FILES_SRC
        "src/*.cpp"
        "src/*.h"
        )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(VVRFRAMEWORK_DIR "" CACHE PATH "Location of VVR Framework")

include_directories(${CMAKE_SOURCE_DIR})
include_directories(include)
include_directories(/usr/local/include)
include_directories(/usr/local/include/opencv4)
include_directories(/usr/local/include/VVRScene)
include_directories(/usr/local/include/GeoLib)
include_directories(/usr/local/include/MathGeoLib)
include_directories(/usr/include/eigen3)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(${SOLUTIONTITLE} ${FILES_SRC})
target_link_libraries(
        ${SOLUTIONTITLE}
        ${OpenCV_LIBS}
        -lVVRScene
        -lGeoLib
        -lMathGeoLib
)