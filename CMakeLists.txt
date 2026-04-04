cmake_minimum_required(VERSION 3.10)
project(manifold_tracing)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(GUDHI REQUIRED)

include_directories(include)

add_executable(trace
    main.cpp
    src/man_trace.cpp
)

target_include_directories(trace PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    /usr/include/gudhi
    ../libcuckoo
)

target_link_libraries(trace PRIVATE
    OpenMP::OpenMP_CXX
    Eigen3::Eigen
)