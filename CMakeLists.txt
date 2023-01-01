cmake_minimum_required(VERSION 3.24)
project(SuspensionOptimization)

set(CMAKE_CXX_STANDARD 17)

add_executable(SuspensionOptimizationSA
        src/SuspensionOptimizationSA.cpp
        src/Solution.cpp
        src/SimulatedAnnealing.cpp
        src/trilaterate.cpp
        )

target_include_directories(SuspensionOptimizationSA PRIVATE inc/ json_struct/include/json_struct/)
