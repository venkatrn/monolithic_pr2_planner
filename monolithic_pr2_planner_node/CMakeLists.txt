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
rosbuild_gensrv()

rosbuild_add_boost_directories()
#common commands for building c++ executables and libraries
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_link_boost(${PROJECT_NAME} thread)
rosbuild_add_executable(node src/node.cpp src/EnvInterfaces.cpp
                             src/CollisionSpaceInterface.cpp
                             src/ompl_pr2_planner.cpp
                             src/ompl_collision_checker.cpp
                        )
# TODO figure out why i have to include bfs3d here
target_link_libraries(node monolithic_pr2_planner bfs3d ${SBPL_LIBRARIES})

rosbuild_add_executable(testPlanningRequest src/testPlanningRequest.cpp)
target_link_libraries(testPlanningRequest monolithic_pr2_planner)

rosbuild_add_executable(testExecute src/testExecute.cpp)
target_link_libraries(testExecute monolithic_pr2_planner ${SBPL_LIBRARIES})

rosbuild_add_executable(stlToOctomap src/publishSTLToOctomap.cpp)
rosbuild_add_executable(run_simulation src/run_simulations.cpp)
rosbuild_add_executable(run_sim_file src/runSimulationFromFile.cpp)
rosbuild_add_executable(run_demo src/run_demo.cpp)
