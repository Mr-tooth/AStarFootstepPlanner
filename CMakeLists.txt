cmake_minimum_required(VERSION 3.22)
set(PROJECT_AUTO_RUN_FINALIZE FALSE)
project(FootstepPlannerLJH VERSION 2.0.0 LANGUAGES CXX)

# --- jrl-cmakemodules: three-tier lookup ---
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake/jrl-cmakemodules")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/jrl-cmakemodules/' (submodule)")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_target_property(JRL_CMAKE_MODULES jrl-cmakemodules::jrl-cmakemodules INTERFACE_INCLUDE_DIRECTORIES)
  else()
    include(FetchContent)
    FetchContent_Declare("jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git"
      GIT_TAG "v0.8.0")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()
include("${JRL_CMAKE_MODULES}/base.cmake")

# --- C++11 ---
check_minimal_cxx_standard(11 ENFORCE)

# --- 依赖 ---
find_package(Heuclid REQUIRED)
find_package(Eigen3 REQUIRED)
# LBlocks: header-only, use submodule path directly
set(LBlocks_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/LBlocks")
find_package(LBlocks REQUIRED PATHS ${LBlocks_DIR} NO_DEFAULT_PATH)
find_package(matplotlib_cpp QUIET)

# --- 核心源文件（显式列表） ---
set(SOURCES
    src/FootstepPlannerLJH/AStarFootstepPlanner.cpp
    src/FootstepPlannerLJH/FootstepPlannerBasic.cpp
    src/FootstepPlannerLJH/parameters.cpp
    src/FootstepPlannerLJH/Block.cpp
    src/FootstepPlannerLJH/Block/Block.cpp
    src/FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.cpp
    src/FootstepPlannerLJH/StepExpansion/ParameterBasedStepExpansion.cpp
    src/FootstepPlannerLJH/StepExpansion/PartialExpansionManager.cpp
    src/FootstepPlannerLJH/StepCost/FootstepCostCalculator.cpp
    src/FootstepPlannerLJH/StepCost/HeuristicCalculator.cpp
    src/FootstepPlannerLJH/StepCheck/FootstepCompletionChecker.cpp
    src/FootstepPlannerLJH/StepConstraints/StepConstraintCheck.cpp
    src/FootstepPlannerLJH/PlotCheck/FootPolygon.cpp  # 纯几何计算，被 StepConstraints 依赖
)

# --- PlotCheck 源文件（仅 matplotlib 相关） ---
set(PLOT_SOURCES
    src/FootstepPlannerLJH/PlotCheck/PlotChecker.cpp
)

# --- 库 target ---
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC Heuclid Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PRIVATE ${LBlocks_DIR}/include)

# --- matplotlib_cpp: 条件启用 ---
if(matplotlib_cpp_FOUND)
    target_sources(${PROJECT_NAME} PRIVATE ${PLOT_SOURCES})
    target_include_directories(${PROJECT_NAME} PRIVATE ${matplotlib_cpp_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${matplotlib_LIBS})
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_MATPLOTLIB)
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /MT)
    endif()
    message(STATUS "matplotlib_cpp enabled — PlotCheck module included")
else()
    message(STATUS "matplotlib_cpp not found — PlotCheck module disabled")
endif()

# --- 测试 ---
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
    enable_testing()
    add_executable(footstep_test test/test.cpp)
    target_link_libraries(footstep_test PRIVATE ${PROJECT_NAME})
    if(matplotlib_cpp_FOUND)
        target_link_libraries(footstep_test PRIVATE ${matplotlib_LIBS})
    endif()
    add_test(NAME footstep_test COMMAND footstep_test)

    add_executable(block_test test/testBlock.cpp)
    target_link_libraries(block_test PRIVATE ${PROJECT_NAME})
    add_test(NAME block_test COMMAND block_test)
endif()

# --- jrl-cmakemodules finalize ---
setup_project_finalize()
