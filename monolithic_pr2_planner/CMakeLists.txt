cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Release)

rosbuild_init()
SET(CMAKE_CXX_FLAGS "-std=c++0x")
# set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")


# find_package(Boost 1.46.0 COMPONENTS thread system filesystem REQUIRED)

#add_definitions(-DUSE_IKFAST_SOLVER)
add_definitions(-DUSE_KDL_SOLVER)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(monolithic_pr2_planner src/StateReps/RobotState.cpp
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
                                            src/MotionPrimitives/TorsoMotionPrimitive.cpp

                                            src/Heuristics/BFS3DHeuristic.cpp
                                            src/Heuristics/BFS2DHeuristic.cpp
                                            src/Heuristics/EndEffectorHeuristic.cpp
                                            src/Heuristics/BaseWithRotationHeuristic.cpp
                                            src/Heuristics/AbstractHeuristic.cpp
                                            src/Heuristics/ArmAnglesHeuristic.cpp
                                            src/Heuristics/BFS3DWithRotationHeuristic.cpp
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
target_link_libraries(monolithic_pr2_planner ${SBPL_LIBRARIES} bfs3d)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_executable(conversions test/conversions.cpp)
target_link_libraries(conversions monolithic_pr2_planner)
rosbuild_add_gtest_build_flags(conversions)
rosbuild_add_rostest(test/tests.launch)
