# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(monolithic_pr2_planner_node)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(catkin REQUIRED COMPONENTS roscpp common_rosdeps geometry_msgs monolithic_pr2_planner leatherman pcl_ros orocos_kdl geometry_msgs sensor_msgs arm_navigation_msgs ompl costmap_2d sbpl_manipulation_components_pr2 interactive_markers sbpl_mha yaml-cpp)

#uncomment if you have defined messages
#add_message_files(
  #FILES
  # TODO: List your msg files here
#)
#uncomment if you have defined services
add_service_files(
  FILES
  DIRECTORY sr  
# TODO: List your msg files here
)
## Generate added messages and services with any dependencies listed here
generate_messages(
    #TODO DEPENDENCIES geometry_msgs std_msgs
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp common_rosdeps geometry_msgs monolithic_pr2_planner leatherman pcl_ros orocos_kdl geometry_msgs sensor_msgs arm_navigation_msgs ompl costmap_2d sbpl_manipulation_components_pr2 interactive_markers sbpl_mha yaml-cpp
    CATKIN_DEPENDS # TODO
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}# TODO include
    LIBRARIES ${catkin_LIBRARIES}# TODO
)
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
set(ROS_BUILD_TYPE Debug)


# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()
SET(CMAKE_CXX_FLAGS "-std=c++0x")
# set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")


# find_package(Boost 1.46.0 COMPONENTS thread system filesystem REQUIRED)
#find_package(PkgConfig REQUIRED)
#pkg_check_modules(sbpl REQUIRED sbpl)
#include_directories(${SBPL_INCLUDE_DIRS})
#link_directories(${SBPL_LIBRARY_DIRS})

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_add_boost_directories()
#common commands for building c++ executables and libraries
#target_link_libraries(${PROJECT_NAME} another_library)
#find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
add_executable(node src/node.cpp src/EnvInterfaces.cpp #(Not required for simulation?)
                             src/CollisionSpaceInterface.cpp
                             src/ompl_pr2_planner.cpp
                             src/ompl_collision_checker.cpp
                        )
# TODO figure out why i have to include bfs3d here
target_link_libraries(node monolithic_pr2_planner bfs3d)

add_executable(testPlanningRequest src/testPlanningRequest.cpp)
target_link_libraries(testPlanningRequest monolithic_pr2_planner)

add_executable(testExecute src/testExecute.cpp)
target_link_libraries(testExecute monolithic_pr2_planner)

add_executable(stlToOctomap src/publishSTLToOctomap.cpp)
add_executable(run_simulation src/run_simulations.cpp)
add_executable(run_sim_file src/runSimulationFromFile.cpp)
add_executable(run_demo src/run_demo.cpp)

add_executable(interactFullBodyPlanner src/interactFullBodyPlanner.cpp)
add_executable(runTests src/run_tests.cpp)
add_executable(generateExperimentsNode src/generateExperimentsNode.cpp)


