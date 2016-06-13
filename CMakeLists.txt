CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
project(slam_base)

set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_COMPILER "g++")

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#set(PCL_DIR /home/jyi/Devel/pcl/180/share/pcl-1.8)
set(OpenCV_DIR /home/jyi/Devel/OpenCV/2412/share/OpenCV)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

SET(G2O_ROOT /home/jyi/Devel/g2o)
LIST( APPEND CMAKE_MODULE_PATH ${G2O_ROOT}/cmake_modules )
FIND_PACKAGE(G2O)
# CSparse
FIND_PACKAGE(CSparse)
INCLUDE_DIRECTORIES( ${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR} )

set(Libconfig_INCLUDE_DIRS /home/jyi/Devel/0000Small/libconfig/include)
set(Libconfig_LIB_DIR /home/jyi/Devel/0000Small/libconfig/lib)
set(Libconfig_LIBS libconfig.a libconfig++.a)

ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ${Libconfig_INCLUDE_DIRS})
LINK_DIRECTORIES(${Libconfig_LIB_DIR})
LINK_LIBRARIES(${PCL_LIBRARY_DIRS} ${Libconfig_LIBS})

link_directories(${PROJECT_SOURCE_DIR}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)

add_subdirectory(${PROJECT_SOURCE_DIR}/src)
