cmake_minimum_required(VERSION 3.16)
project(infeasibility_planner LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------
# Dependencies
# -----------------------
find_package(pybind11 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(NLopt REQUIRED)

# ThunderSVM (manual include if needed)
find_path(THUNDERSVM_INCLUDE_DIR thundersvm/svm.h)
find_library(THUNDERSVM_LIB thundersvm)

# GUDHI
find_package(GUDHI REQUIRED)

# -----------------------
# Includes
# -----------------------
include_directories(
    include
    ${THUNDERSVM_INCLUDE_DIR}
    ${GUDHI_INCLUDE_DIRS}
)

# -----------------------
# Sources
# -----------------------
file(GLOB SRC_FILES
    src/*.cpp
)

# -----------------------
# Executable
# -----------------------
add_executable(infeasibility_planner ${SRC_FILES})

# -----------------------
# Linking
# -----------------------
target_link_libraries(infeasibility_planner
    pybind11::embed
    ${THUNDERSVM_LIB}
    NLopt::nlopt
    GUDHI::gudhi
    OpenMP::OpenMP_CXX
)
