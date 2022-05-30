cmake_minimum_required(VERSION 3.16)
project(DelaunayMG)

set(CMAKE_CXX_STANDARD 20)
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)

find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CGAL REQUIRED)

add_executable(DelaunayMG main.cpp triangulation3D.cpp)
target_compile_options(DelaunayMG PRIVATE -Wall -Wextra -Wpedantic -Werror)
include_directories(../)
target_link_libraries(DelaunayMG sfml-graphics sfml-window sfml-system boost_system boost_thread pthread OpenMP::OpenMP_CXX Eigen3::Eigen CGAL)