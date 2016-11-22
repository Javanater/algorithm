cmake_minimum_required(VERSION 3.5)
project(Algorithms)
set(SOURCE_FILES AStar.hpp GridAStar.cpp GridAStar.hpp DStarLite.hpp LPAStar.hpp)
add_library(${PROJECT_NAME} ${SOURCE_FILES})