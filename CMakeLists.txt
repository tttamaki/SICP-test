cmake_minimum_required(VERSION 2.8)
set(CMAKE_VERBOSE_MAKEFILE 1) # to see all make process

project( icp_samples )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse3 -openmp -O3 ")

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(SCIP_INCLUDE_DIRS ../sparseicp/)
set(nanoflann_INCLUDE_DIRS ../sparseicp/include/)

include_directories(${SCIP_INCLUDE_DIRS})
include_directories(${nanoflann_INCLUDE_DIRS})


add_executable (sicp1_simple_icp src/sicp1_simple_icp.cpp)
target_link_libraries (sicp1_simple_icp ${PCL_LIBRARIES})


add_executable (sicp2_with_normal_iterative_view src/sicp2_with_normal_iterative_view.cpp)
target_link_libraries (sicp2_with_normal_iterative_view ${PCL_LIBRARIES})

