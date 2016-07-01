# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html

cmake_minimum_required(VERSION 2.8.3)
project(monolithic_pr2_planner_node)

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
#find_package(ompl REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs monolithic_pr2_planner leatherman pcl_ros geometry_msgs sensor_msgs arm_navigation_msgs costmap_2d
                             sbpl_manipulation_components_pr2 interactive_markers sbpl_mha message_generation pviz tf2 pcl_conversions gridmap_2d)
find_package(orocos_kdl)
#uncomment if you have defined services
add_service_files(
  DIRECTORY srv
# TODO: List your msg files here
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES trajectory_msgs geometry_msgs std_msgs 
)

# catkin_package parameters: http://ros.org/doc/groovy/api/catkin/html/dev_guide/generated_cmake_api.html#catkin-package
# TODO: fill in what other packages will need to use this package
catkin_package(
    DEPENDS roscpp geometry_msgs costmap_2d sbpl_manipulation_components_pr2 interactive_markers
             orocos_kdl
             CATKIN_DEPENDS pcl_conversions tf2 message_runtime pviz sbpl_manipulation_components_pr2 monolithic_pr2_planner leatherman pcl_ros  geometry_msgs sensor_msgs arm_navigation_msgs sbpl_mha gridmap_2d# TODO
    INCLUDE_DIRS include ${catkin_INCLUDE_DIRS} ${ompl_INCLUDE_DIRS}# TODO include
    LIBRARIES ${catkin_LIBRARIES} ${ompl_LIBRARIES} # TODO
)

find_package(OpenCV REQUIRED)

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
set(CATKIN_BUILD_TYPE Release )

# CATKIN_MIGRATION: removed during catkin migration
# rosbuild_init()
SET(CMAKE_CXX_FLAGS "-std=c++0x")
# set(CMAKE_CXX_COMPILER "/usr/bin/g++-4.8")

# find_package(Boost 1.46.0 COMPONENTS thread system filesystem REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(ompl REQUIRED ompl)
include_directories(${ompl_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})
link_directories(${ompl_LIBRARY_DIRS} )

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${Boost_INCLUDE_DIRS} include ${catkin_INCLUDE_DIRS} ${ompl_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${OpenCV}_INCLUDE_DIRS)
#target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES} ${ompl_LIBRARIES})

add_executable(node src/node.cpp src/EnvInterfaces.cpp #(Not required for simulation?)
                             src/CollisionSpaceInterface.cpp
                             src/ompl_pr2_planner.cpp
                             src/ompl_collision_checker.cpp
                        )

# TODO I don't know why I have to list the packages explicitly.
target_link_libraries(node monolithic_pr2_planner ${tf2_LIBRARIES} bfs3d ompl ikfast_pr2 sbpl_manipulation_components_pr2 pviz ompl pr2_collision_checker ${catkin_LIBRARIES} ${ompl_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(testPlanningRequest src/testPlanningRequest.cpp)
target_link_libraries(testPlanningRequest monolithic_pr2_planner)

add_executable(interactFullBodyPlanner src/interactFullBodyPlanner.cpp)
add_dependencies(interactFullBodyPlanner ${catkin_EXPORTED_TARGETS})
# TODO I don't know why I have to list the packages explicitly.
target_link_libraries(interactFullBodyPlanner monolithic_pr2_planner sbpl_manipulation_components_pr2 pviz ${catkin_LIBRARIES} ${ompl_LIBRARIES})

add_executable(testExecute src/testExecute.cpp)
add_dependencies(testExecute ${catkin_EXPORTED_TARGETS})
# TODO I don't know why I have to list the packages explicitly.
target_link_libraries(testExecute monolithic_pr2_planner sbpl_manipulation_components sbpl_manipulation_components_pr2 pviz pr2_collision_checker bfs3d ikfast_pr2 )

add_executable(stlToOctomap src/publishSTLToOctomap.cpp)
target_link_libraries(stlToOctomap monolithic_pr2_planner ${pcl_conversions_LIBRARIES})

add_executable(run_simulation src/run_simulations.cpp)
target_link_libraries(run_simulation monolithic_pr2_planner)

add_executable(run_sim_file src/runSimulationFromFile.cpp)
target_link_libraries(run_sim_file monolithic_pr2_planner)

add_executable(run_demo src/run_demo.cpp)
target_link_libraries(run_demo monolithic_pr2_planner ${tf2_LIBRARIES})

add_executable(runTests src/run_tests.cpp)
target_link_libraries(runTests monolithic_pr2_planner)

add_executable(generateExperimentsNode src/generateExperimentsNode.cpp)
add_dependencies(generateExperimentsNode ${catkin_EXPORTED_TARGETS})
target_link_libraries(generateExperimentsNode monolithic_pr2_planner)

add_executable(saveProjectedMap src/saveProjectedMap.cpp)
target_link_libraries(saveProjectedMap monolithic_pr2_planner ${OpenCV_LIBS} ${catkin_LIBRARIES})



