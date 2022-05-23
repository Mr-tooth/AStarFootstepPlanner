cmake_minimum_required(VERSION 3.0)
set(PROJECT_NAME FootstepPlannerLJH)
set(LIB_NAME FootstepPlanner)
project(${PROJECT_NAME} VERSION 1.0.0)

# if need to Modify and Debug , set BUILD_TEST_LJH 1
set(BUILD_TEST_LJH 0)
if(${BUILD_TEST_LJH})
    set(CMAKE_BUILD_TYPE Debug)
    # message("Build type is Debug! ")
else()
    set(CMAKE_BUILD_TYPE Release)
    add_compile_options(/O2)
    # message("Build type is Release! ")
endif()

message("Build type is ${CMAKE_BUILD_TYPE}")

# add_compile_options(/std=c++11)
add_compile_options(/MT)

find_package(matplotlib_cpp REQUIRED)
find_package(Heuclid REQUIRED)

include_directories(./include)
# include_directories(include)
#include_directories(./include/FootstepPlannerLJH)

aux_source_directory(src/FootstepPlannerLJH SRC_FILES)
aux_source_directory(src/FootstepPlannerLJH/StepExpansion SRC_FILES)
aux_source_directory(src/FootstepPlannerLJH/StepCost SRC_FILES)
aux_source_directory(src/FootstepPlannerLJH/StepCheck SRC_FILES)
aux_source_directory(src/FootstepPlannerLJH/PlotCheck SRC_FILES)
aux_source_directory(src/FootstepPlannerLJH/StepConstraints SRC_FILES)

link_libraries(Heuclid.lib  ${matplotlib_LIBS})


# add_library(${LIB_NAME} STATIC ${SRC_FILES})



if(${BUILD_TEST_LJH})
    message("--[Test] Build FootstepPlannerLJH basic test by Source Code!")
    add_executable(test test/test.cpp ${SRC_FILES})
else()
    message("--Build Final Static Lib!")
    add_library(${LIB_NAME} STATIC ${SRC_FILES})
endif()

# set install configuration
message("Install path is ${CMAKE_INSTALL_PREFIX}")

set(FootstepPlanner_include_dirs ${CMAKE_INSTALL_PREFIX}/include)
set(FootstepPlanner_link_dirs ${CMAKE_INSTALL_PREFIX}/lib)

configure_file(FootstepPlanner.cmake.in ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/FootstepPlannerLJHConfig.cmake @ONLY)
install(FILES ${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/FootstepPlannerLJHConfig.cmake DESTINATION share/FootstepPlannerLJH/cmake)
install(FILES ${PROJECT_BINARY_DIR}/FootstepPlanner.lib DESTINATION lib)
install(DIRECTORY include/FootstepPlannerLJH DESTINATION include)

