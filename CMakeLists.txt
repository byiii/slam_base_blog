CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(slam_base)

set(CMAKE_CXX_COMPILER "g++")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(PCL_DIR /home/jyi/Devel/pcl/180/share/pcl-1.8)
set(OpenCV_DIR /home/jyi/Devel/OpenCV/2412/share/OpenCV)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )

link_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
