cmake_minimum_required(VERSION 2.6)
project(test)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  add_definitions(-DCOMPILEDWITHC11)
  message(STATUS "Using flag -std=c++11.")
else()
  message(FATAL_ERROR "The compiler has no C++11 support!")
endif()

find_package(OpenCV REQUIRED)

include_directories(include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(test test.cpp SerialPort.cpp)
target_link_libraries(test ${OpenCV_LIBRARIES} pthread)

install(TARGETS test RUNTIME DESTINATION bin)
