#!/bin/bash

echo "Installing dependencies from github"
wstool init ../.. .rosinstall;
cd ../../;
wstool update;

echo "Installing debian packages using apt-get"
rosdep install --from-paths . --ignore-src --rosdistro=indigo

# orocos_kdl (Not a catkin package, so add it as a separate library in cmakelist)
# Eigen
# livgsl0-dev
# ompl
# libsdl1.2-dev 
# pcl_conversions

