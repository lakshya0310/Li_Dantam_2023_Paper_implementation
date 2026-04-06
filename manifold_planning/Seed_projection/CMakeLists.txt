cmake_minimum_required(VERSION 3.10)
project(manifold_projection)

set(CMAKE_CXX_STANDARD 17)

# Find Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# Find NLopt
find_path(NLOPT_INCLUDE_DIRS nlopt.hpp)
find_library(NLOPT_LIBRARIES nlopt)

add_executable(projection_test seed_projection.cpp)
target_include_directories(projection_test PRIVATE ${NLOPT_INCLUDE_DIRS})
target_link_libraries(projection_test PRIVATE ${NLOPT_LIBRARIES})