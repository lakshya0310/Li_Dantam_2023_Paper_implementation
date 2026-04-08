cmake_minimum_required(VERSION 3.10)
project(FCL_Collision_Checker)

set(CMAKE_CXX_STANDARD 14)


find_package(fcl REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(collision_test main.cpp)

target_include_directories(collision_test PRIVATE ${EIGEN3_INCLUDE_DIRS})

target_link_libraries(collision_test PRIVATE fcl)