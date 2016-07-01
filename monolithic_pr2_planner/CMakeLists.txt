# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(monolithic_pr2_planner)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS pr2_collision_checker angles pviz bfs3d ikfast_pr2 nav_msgs moveit_msgs costmap_2d sbpl_mha rostest)
#find_package(sbpl)
# CATKIN_MIGRATION: removed during catkin migration

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS pr2_collision_checker angles pviz bfs3d ikfast_pr2 nav_msgs moveit_msgs costmap_2d rostest
    CATKIN_DEPENDS sbpl_mha # TODO
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${sbpl_mha}#${sbpl_INCLUDE_DIRS}# TODO include
    LIBRARIES ${catkin_LIBRARIES}# TODO
)

#find_package(PkgConfig REQUIRED)
#pkg_check_modules(sbpl REQUIRED sbpl)
#include_directories(${SBPL_INCLUDE_DIRS})
#link_directories(${SBPL_LIBRARY_DIRS})

# include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})
# CATKIN_MIGRATION: removed during catkin migration
# cmake_minimum_required(VERSION 2.4.6)

# CATKIN_MIGRATION: removed during catkin migration
# include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(CATKIN_BUILD_TYPE Release)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()
SET(CMAKE_CXX_FLAGS "-std=c++0x")
# set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")


# find_package(Boost 1.46.0 COMPONENTS thread system filesystem REQUIRED)

#add_definitions(-DUSE_IKFAST_SOLVER)
add_definitions(-DUSE_KDL_SOLVER)

# Depend on sbpl-ros package for now
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(SBPL REQUIRED sbpl)
#include_directories(${SBPL_INCLUDE_DIRS})
#link_directories(${SBPL_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(include ${catkin_INCLUDE_DIRS})# ${sbpl_INCLUDE_DIRS})
#common commands for building c++ executables and libraries
add_library(monolithic_pr2_planner src/StateReps/RobotState.cpp
                                            src/StateReps/ContArmState.cpp
                                            src/StateReps/ContBaseState.cpp
                                            src/StateReps/DiscBaseState.cpp
                                            src/StateReps/DiscObjectState.cpp
                                            src/StateReps/ContObjectState.cpp
                                            src/StateReps/GraphState.cpp
                                            src/StateReps/GoalState.cpp
                                            
                                            src/MotionPrimitives/ArmMotionPrimitive.cpp
                                            src/MotionPrimitives/BaseMotionPrimitive.cpp
                                            src/MotionPrimitives/MotionPrimitive.cpp
                                            src/MotionPrimitives/FileParser.cpp
                                            src/MotionPrimitives/MotionPrimitivesMgr.cpp
                                            src/MotionPrimitives/BaseAdaptiveMotionPrimitive.cpp
                                            src/MotionPrimitives/ArmAdaptiveMotionPrimitive.cpp
                                            src/MotionPrimitives/ArmTuckMotionPrimitive.cpp
                                            src/MotionPrimitives/ArmUntuckMotionPrimitive.cpp
                                            src/MotionPrimitives/TorsoMotionPrimitive.cpp

                                            src/Heuristics/BFS3DHeuristic.cpp
                                            src/Heuristics/BFS2DHeuristic.cpp
                                            src/Heuristics/BFS2DRotFootprintHeuristic.cpp
                                            src/Heuristics/EndEffectorHeuristic.cpp
                                            src/Heuristics/EndEffLocalHeuristic.cpp
                                            src/Heuristics/BaseWithRotationHeuristic.cpp
                                            src/Heuristics/AbstractHeuristic.cpp
                                            src/Heuristics/ArmAnglesHeuristic.cpp
                                            src/Heuristics/BFS3DWithRotationHeuristic.cpp
                                            src/Heuristics/EndEffOnlyRotationHeuristic.cpp
                                            src/Heuristics/HeuristicMgr.cpp
                                            src/Heuristics/2Dgridsearch.cpp

                                            src/SearchRequest.cpp
                                            src/Environment.cpp
                                            src/OccupancyGridUser.cpp
                                            src/CollisionSpaceMgr.cpp
                                            src/ParameterCatalog.cpp
                                            src/HashManager.cpp
                                            src/TransitionData.cpp
                                            src/Visualizer.cpp
                                            src/PathPostProcessor.cpp

                                            src/ExperimentFramework/randomStartGoalGenerator.cpp
                                            src/StatsWriter.cpp
                                            )
target_link_libraries(monolithic_pr2_planner ${catkin_LIBRARIES})# ${sbpl_LIBRARIES})
#
# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
#target_link_libraries(example ${PROJECT_NAME})

catkin_add_gtest(conversions test/conversions.cpp)
#add_executable(conversions test/conversions.cpp)
target_link_libraries(conversions monolithic_pr2_planner)

add_rostest(test/tests.launch)
