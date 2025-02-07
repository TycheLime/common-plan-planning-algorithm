cmake_minimum_required(VERSION 3.25)
project(links)

set(CMAKE_CXX_STANDARD 23)

include_directories(./inc)

file(GLOB_RECURSE SOURCES "src/*.cpp")

add_executable(silicon ${SOURCES} main.cpp)
