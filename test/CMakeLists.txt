cmake_minimum_required(VERSION 3.0)
project(FootstepPlannerTest)

add_compile_options(/MT)

find_package(matplotlib_cpp REQUIRED)
find_package(Heuclid REQUIRED)
find_package(FootstepPlannerLJH REQUIRED)

link_libraries(FootstepPlanner.lib Heuclid.lib ${matplotlib_LIBS})

message("include ${FOOTSTEPPLANNER_INCLUDE_DIRS}")
set(BUILD_TEST_LJH 1)

if(${BUILD_TEST_LJH})
    message("--[Test] Build FootstepPlanner basic test!")
    add_executable(test test.cpp)
    
endif()