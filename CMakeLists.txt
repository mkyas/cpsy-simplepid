cmake_minimum_required(VERSION 3.16)

project(simplepid VERSION 1.0)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(waterboiler waterboiler.cpp simplepid.hpp)
