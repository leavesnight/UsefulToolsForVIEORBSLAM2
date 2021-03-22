cmake_minimum_required(VERSION 2.6)
project(test)

# C++11 support
IF(NOT MSVC)
  include(CheckCXXCompilerFlag)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
  CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
  if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
  endif()
endif()

#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/home/leavesnight/zzh/g2o/cmake_modules/")
#find_package(G2O REQUIRED)
#include_directories(SYSTEM ${G2O_INCLUDE_DIR} ${G2O_INCLUDE_DIRS})

set(OpenCV_DIR "/usr/local/opencv320/share/OpenCV") 
find_package(OpenCV 3.0 QUIET)

message("${OpenCV_INCLUDE_DIRS}\n${OpenCV_LIBS}")
#here include_directories can be omitted for already used target_link_libraries and find_package!

include_directories(
${OpenCV_INCLUDE_DIRS}
)

#needed part
find_package(Eigen3 REQUIRED)
include_directories(SYSTEM 
${EIGEN3_INCLUDE_DIR}
)

add_executable(get_estimate test.cpp)

target_link_libraries(get_estimate
${OpenCV_LIBS}
)

install(TARGETS get_estimate RUNTIME DESTINATION bin)
