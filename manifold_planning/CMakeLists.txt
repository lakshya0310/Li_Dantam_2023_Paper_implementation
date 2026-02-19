cmake_minimum_required(VERSION 3.4)
project(infeasibility_proof)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# 1. Find Dependencies
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP)

# 2. Locate ThunderSVM Source Headers (dataset.h etc)
find_path(THUNDERSVM_SRC_INCLUDE_DIR
    NAMES thundersvm/dataset.h
    HINTS "${CMAKE_SOURCE_DIR}/external/thundersvm/include"
    DOC "Path to ThunderSVM source headers"
)

# 3. Locate ThunderSVM Generated Headers (config.h) - UPDATED
# We now look in both 'build/include' AND just 'build' to catch your specific case
find_path(THUNDERSVM_GEN_INCLUDE_DIR
    NAMES thundersvm/config.h
    HINTS 
        "${CMAKE_SOURCE_DIR}/external/thundersvm/build/include"
        "${CMAKE_SOURCE_DIR}/external/thundersvm/build" 
    DOC "Path to ThunderSVM generated headers (config.h)"
)

if(NOT THUNDERSVM_SRC_INCLUDE_DIR OR NOT THUNDERSVM_GEN_INCLUDE_DIR)
    message(FATAL_ERROR "Could not find ThunderSVM headers. \n \
    Source found: ${THUNDERSVM_SRC_INCLUDE_DIR} \n \
    Generated found: ${THUNDERSVM_GEN_INCLUDE_DIR} \n \
    Please verify external/thundersvm/build contains thundersvm/config.h")
else()
    message(STATUS "ThunderSVM Source Headers: ${THUNDERSVM_SRC_INCLUDE_DIR}")
    message(STATUS "ThunderSVM Generated Headers: ${THUNDERSVM_GEN_INCLUDE_DIR}")
endif()

# 4. Locate Library
find_library(THUNDERSVM_LIB 
    NAMES thundersvm
    HINTS 
        "${CMAKE_SOURCE_DIR}/external/thundersvm/build/lib" 
        "${CMAKE_SOURCE_DIR}/external/thundersvm/lib"
)

if(NOT THUNDERSVM_LIB)
    message(FATAL_ERROR "Could not find libthundersvm.a/.so. Build it in external/thundersvm/build first.")
endif()

# 5. Include Directories
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${THUNDERSVM_GEN_INCLUDE_DIR}) # Points to build/ or build/include
include_directories(${THUNDERSVM_SRC_INCLUDE_DIR}) # Points to source include
include_directories(/usr/include/gudhi) 

# 6. Build Extension
pybind11_add_module(infeasibility_proof
    infeasibility_planner.cpp
    coxeter_algo.cpp
)

# 7. Link Libraries
target_link_libraries(infeasibility_proof PRIVATE ${THUNDERSVM_LIB})

if(OpenMP_CXX_FOUND)
    target_link_libraries(infeasibility_proof PRIVATE OpenMP::OpenMP_CXX)
endif()