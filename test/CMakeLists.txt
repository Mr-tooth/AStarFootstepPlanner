cmake_minimum_required(VERSION 3.22)
project(FootstepPlannerTest LANGUAGES CXX)

find_package(Heuclid REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(matplotlib_cpp QUIET)

add_executable(test test.cpp)
target_link_libraries(test PRIVATE FootstepPlannerLJH)
if(matplotlib_cpp_FOUND)
    target_include_directories(test PRIVATE ${matplotlib_cpp_INCLUDE_DIRS})
    target_link_libraries(test PRIVATE ${matplotlib_LIBS})
endif()
