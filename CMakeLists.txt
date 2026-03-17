cmake_minimum_required(VERSION 3.22)
set(PROJECT_AUTO_RUN_FINALIZE FALSE)
project(FootstepPlannerLJH VERSION 2.0.0 LANGUAGES CXX)

# --- jrl-cmakemodules required variables ---
set(PROJECT_URL "https://github.com/Mr-tooth/AStarFootstepPlanner")
set(PROJECT_DESCRIPTION "A* algorithm-based footstep planner for humanoid robots")

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
      GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()
include("${JRL_CMAKE_MODULES}/base.cmake")
include(GNUInstallDirs)

# --- C++11 ---
check_minimal_cxx_standard(11 ENFORCE)

# --- Dependencies ---

# Heuclid: three-tier lookup (find_package → FetchContent)
find_package(Heuclid QUIET CONFIG)
if(NOT Heuclid_FOUND)
  message(STATUS "Heuclid not found, fetching from GitHub...")
  include(FetchContent)
  FetchContent_Declare(Heuclid
    GIT_REPOSITORY "https://github.com/Mr-tooth/Heuclid.git"
    GIT_TAG "v2.2"
    GIT_SHALLOW TRUE)
  set(BUILD_TESTING_HEUCID OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(Heuclid)
  message(STATUS "Heuclid fetched successfully")
else()
  message(STATUS "Heuclid found: ${Heuclid_DIR}")
endif()

# Eigen3: header-only, INTERFACE target. Follow Heuclid's proven pattern.
find_package(Eigen3 3.3 QUIET CONFIG)
if(NOT TARGET Eigen3::Eigen)
  message(STATUS "Eigen3 not found, fetching 3.4.0 via FetchContent")
  include(FetchContent)
  FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY "https://gitlab.com/libeigen/eigen.git"
    GIT_TAG "3.4.0"
    GIT_SHALLOW TRUE
  )
  FetchContent_GetProperties(Eigen3)
  if(NOT eigen3_POPULATED)
    FetchContent_Populate(Eigen3)
  endif()
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${eigen3_SOURCE_DIR}"
  )
  set(Eigen3_FOUND TRUE)
  message(STATUS "Eigen3 fetched: ${eigen3_SOURCE_DIR}")
else()
  message(STATUS "Eigen3 found: ${Eigen3_VERSION}")
endif()

# LBlocks: header-only, use submodule path directly (no find_package needed)
set(LBlocks_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/LBlocks/include")

# matplotlib_cpp: optional (for visualization)
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

target_link_libraries(${PROJECT_NAME} PUBLIC heuclid Eigen3::Eigen)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${LBlocks_INCLUDE_DIR}>)

# --- matplotlib_cpp: 条件启用 ---
# Try find_package first, then fallback to external/ submodule
set(MATPLOTLIB_CPP_AVAILABLE FALSE)
if(matplotlib_cpp_FOUND)
    set(MATPLOTLIB_CPP_AVAILABLE TRUE)
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/matplotlib-cpp/matplotlibcpp.h")
    set(MATPLOTLIB_CPP_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/external/matplotlib-cpp")
    set(MATPLOTLIB_CPP_AVAILABLE TRUE)
    message(STATUS "matplotlib_cpp found: external/matplotlib-cpp/")
endif()

if(MATPLOTLIB_CPP_AVAILABLE)
    target_sources(${PROJECT_NAME} PRIVATE ${PLOT_SOURCES})
    if(matplotlib_cpp_FOUND)
        target_include_directories(${PROJECT_NAME} PRIVATE ${matplotlib_cpp_INCLUDE_DIRS})
        target_link_libraries(${PROJECT_NAME} PRIVATE ${matplotlib_LIBS})
    else()
        target_include_directories(${PROJECT_NAME} PRIVATE ${MATPLOTLIB_CPP_INCLUDE})
        find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
        target_link_libraries(${PROJECT_NAME} PRIVATE Python3::Python Python3::NumPy)
    endif()
    target_compile_definitions(${PROJECT_NAME} PRIVATE HAS_MATPLOTLIB)
    if(MSVC)
        target_compile_options(${PROJECT_NAME} PRIVATE /MT)
    endif()
    message(STATUS "matplotlib_cpp enabled — PlotCheck module included")

    # --- Demo GIF generator (data export only, visualization in Python) ---
    add_executable(demo_export demo/demo_export.cpp)
    target_link_libraries(demo_export PRIVATE ${PROJECT_NAME})
    add_executable(demo_obstacle demo/demo_obstacle.cpp)
    target_link_libraries(demo_obstacle PRIVATE ${PROJECT_NAME})
    add_executable(demo_stairs demo/demo_stairs.cpp)
    target_link_libraries(demo_stairs PRIVATE ${PROJECT_NAME})
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

# --- 安装 ---
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}Targets)
install(DIRECTORY include/FootstepPlannerLJH
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# --- jrl-cmakemodules finalize ---
setup_project_finalize()
