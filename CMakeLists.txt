# first command in a make file should always be specifying min cmake version required
cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

# required languages are (C, C++ and CUDA)
project(Particle-Filter LANGUAGES C CXX CUDA VERSION 1.0) # lets CMake identify and verify the compiler it needs

# let CUDA files be compiled according to C++14
set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# specify C++ standard 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# separately compile CUDA files 
set(CMAKE_CUDA_SEPARABLE_COMPILATION ON)

# choose CUDA Architecture
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    set(CMAKE_CUDA_ARCHITECTURES 60)
endif()

set(pf_cpu particle_filter_cpu)
set(pf_gpu particle_filter_gpu)
set(robot robot_lib)

# include dirs
set(INCLUDE_DIR "${PROJECT_SOURCE_DIR}/Helper")

# add libraries
add_subdirectory(Helper)

# create a list of libraries
set(LIBS ${robot})

add_executable(${pf_cpu} particle_filter.cpp)
target_link_libraries(${pf_cpu} ${LIBS})
target_include_directories(${pf_cpu} PUBLIC ${INCLUDE_DIR})
if (CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${pf_cpu} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-g>)
endif ()

add_executable(${pf_gpu} particle_filter.cu)
target_link_libraries(${pf_gpu} ${LIBS})
target_include_directories(${pf_gpu} PUBLIC ${INCLUDE_DIR})

# if debug build, add debugging flag for CUDA compiling 
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  target_compile_options(${pf_gpu} PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-g -G>)
endif()
