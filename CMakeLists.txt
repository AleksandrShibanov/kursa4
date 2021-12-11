cmake_minimum_required(VERSION 3.16)
project(DelaunayMG)

set(CMAKE_CXX_STANDARD 20)

add_executable(DelaunayMG main.cpp incremental.cpp Edge.cpp triangle.cpp Point.cpp)
include_directories(../)
target_link_libraries(DelaunayMG sfml-graphics sfml-window sfml-system boost_system boost_thread pthread)