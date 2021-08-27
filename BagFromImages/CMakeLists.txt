cmake_minimum_required(VERSION 2.4.6)
message(STATUS "check env=" $ENV{ROS_ROOT})
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

project(BagFromImages)

#rosbuild_init()

IF (NOT ROS_BUILD_TYPE)
    SET(ROS_BUILD_TYPE Release)
ENDIF ()
MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
    add_definitions(-DCOMPILEDWITHC11)
    message(STATUS "Using flag -std=c++11.")
elseif (COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
    add_definitions(-DCOMPILEDWITHC0X)
    message(STATUS "Using flag -std=c++0x.")
else ()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

FIND_PACKAGE(OpenCV 4.5 REQUIRED)
message(STATUS "Opencv=" ${OpenCV_VERSION})
message("OpenCV:" ${OpenCV_INCLUDE_DIRS})
message("LIBS:" ${OpenCV_LIBS})
FIND_PACKAGE(rosbag REQUIRED)
message("ROSBAG:" ${rosbag_INCLUDE_DIRS})
message("LIBS:" ${rosbag_LIBRARIES})
FIND_PACKAGE(cv_bridge REQUIRED)
message("cv_bridge: " ${cv_bridge_INCLUDE_DIRS})

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
        ${rosbag_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
)

rosbuild_add_executable(${PROJECT_NAME}
        main.cc
        Thirdparty/DLib/FileFunctions.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${rosbag_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        )
