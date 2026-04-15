cmake_minimum_required(VERSION 3.10)
project(manifold_tracing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GUDHI REQUIRED)
find_package(ompl REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(include ../include)
link_directories(/usr/local/lib)

add_executable(trace
    main.cpp
    src/man_trace.cpp
    src/prm_sampler.cpp
    src/robot_world.cpp
    src/out.c
)

target_include_directories(trace PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${OMPL_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
    /usr/include/gudhi
    /usr/local/include/amino-1.0
    ../libcuckoo
)

target_link_libraries(trace PRIVATE
    OpenMP::OpenMP_CXX
    Eigen3::Eigen
    ${OMPL_LIBRARIES}
    ${SDL2_LIBRARIES}
    amino
    amino-planning
    amino-collision
    amino-gl
)