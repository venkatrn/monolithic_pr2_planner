#!/bin/bash

echo "Installing dependencies from github"
wstool init ../.. .rosinstall;
cd ../../;
wstool update;

echo "Installing debian packages using apt-get"
rosdep install --from-paths . --ignore-src --rosdistro=indigo

