cmake_minimum_required(VERSION 2.8)

project(INSAlgorithm)
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

include_directories(include)
AUX_SOURCE_DIRECTORY(src DIR_SRCS)
include_directories("/usr/include/eigen3")
add_executable(a ${DIR_SRCS})