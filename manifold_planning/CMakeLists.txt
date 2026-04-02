cmake_minimum_required(VERSION 3.4)
project(infeasibility_proof)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)
find_package(CUDAToolkit REQUIRED)

# ThunderSVM
find_path(THUNDERSVM_INCLUDE_DIR
    NAMES thundersvm/thundersvm.h
    PATHS /usr/local/include
)

find_library(THUNDERSVM_LIBRARY
    NAMES thundersvm
    PATHS /usr/local/lib
)

if(NOT THUNDERSVM_INCLUDE_DIR OR NOT THUNDERSVM_LIBRARY)
    message(FATAL_ERROR "ThunderSVM not found")
endif()

pybind11_add_module(infeasibility_proof
    infeasibility_planner.cpp
    coxeter_algo.cpp
)

target_include_directories(infeasibility_proof PRIVATE
    ${EIGEN3_INCLUDE_DIR}
    ${THUNDERSVM_INCLUDE_DIR}
    ${CUDAToolkit_INCLUDE_DIRS}
    /usr/include/gudhi
)

target_link_libraries(infeasibility_proof PRIVATE
    pybind11::module
    Eigen3::Eigen
    thundersvm
    CUDA::cudart
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(infeasibility_proof PRIVATE OpenMP::OpenMP_CXX)
endif()