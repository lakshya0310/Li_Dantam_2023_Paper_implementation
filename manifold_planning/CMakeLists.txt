cmake_minimum_required(VERSION 3.10)
project(manifold_planner)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(planner
    src/main.cpp
    src/CoxeterTriangulation.cpp
    src/Manifold.cpp
    src/ElasticUpdate.cpp
    src/Graph.cpp
    src/AStar.cpp
)
