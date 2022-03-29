cmake_minimum_required(VERSION 3.10)

set (CMAKE_CXX_STANDARD 17)
# set the project name
project(mrenv)

find_package(Eigen3 REQUIRED)
include_directories( ${EIGEN3_INCLUDE_DIR} )
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_ROOT})
include_directories(.)
include_directories( ${PROJECT_SOURCE_DIR} )
include_directories( ${CMAKE_SOURCE_DIR} )


include_directories(${CMAKE_SOURCE_DIR}/external/iris-distro/include/)
include_directories(${CMAKE_SOURCE_DIR}/external/iris-distro/lib/)
link_directories(${CMAKE_SOURCE_DIR}/external/iris-distro/lib/)


add_subdirectory(include/mrenv/)
add_subdirectory(src/)
