cmake_minimum_required(VERSION 2.6)

SET(PROJECT_NAME tinystretch)
PROJECT(${PROJECT_NAME})
get_filename_component(PROJECT ${CMAKE_SOURCE_DIR} NAME)

find_package(Eigen3)

add_compile_definitions(__USE_SINGLE_PRECISION__)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 -pthread" )
# set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../../build)

include_directories(
    "./build"
    "./include_vmc"
    "./include_wb"
    "./vmc_inc"
    "./mpc"
    "./vision_location"
    "./math_src"
    "./cube_vision"
    "./source"
    "./rbdl_src/urdf"
    "./rbdl_src/rbdl"
    "./rbdl_src/model"
    "./rbdl_src/common_utils"
    "./qpOASES/include"
    "/home/pi/webots/webots/include/controller/c"
    "/home/pi/webots/webots/include/controller/cpp/"
)

link_directories("/usr/local/lib")
include_directories("/usr/local/include")

LINK_LIBRARIES("/home/pi/webots/webots/lib/controller/libController.so")
LINK_LIBRARIES("/home/pi/webots/webots/lib/controller/libCppController.so")


AUX_SOURCE_DIRECTORY(gait_src DIR_SRCS)
AUX_SOURCE_DIRECTORY(locomotion_src DIR_SRCS1)
AUX_SOURCE_DIRECTORY(math_src DIR_SRCS2)
AUX_SOURCE_DIRECTORY(mpc DIR_SRCS3)
AUX_SOURCE_DIRECTORY(share_mem DIR_SRCS4)
AUX_SOURCE_DIRECTORY(src_sim DIR_SRCS5)
AUX_SOURCE_DIRECTORY(rbdl_src/urdf  DIR_SRCS6)
AUX_SOURCE_DIRECTORY(rbdl_src/rbdl  DIR_SRCS7)
AUX_SOURCE_DIRECTORY(rbdl_src/model DIR_SRCS8)
AUX_SOURCE_DIRECTORY(rbdl_src/common_utils DIR_SRCS9)

ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS} ${DIR_SRCS1}  ${DIR_SRCS2} ${DIR_SRCS3} ${DIR_SRCS4} ${DIR_SRCS5} ${DIR_SRCS6} ${DIR_SRCS7} ${DIR_SRCS8} ${DIR_SRCS9})

add_subdirectory(yaml-cpp lib/ EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

add_subdirectory(qpOASES 1lib/ EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} qpOASES)

link_directories($ENV{WEBOTS_HOME}/lib/controller)
link_directories($ENV{WEBOTS_HOME}/lib/webots)
file(GLOB C_SOURCES *.c)
file(GLOB CPP_SOURCES *.cpp)
set(SOURCES ${C_SOURCES} ${CPP_SOURCES})

if (NOT CPP_SOURCES STREQUAL "")  # Sources contain C++ files
  set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX} ${CMAKE_SHARED_LIBRARY_PREFIX}CppController${CMAKE_SHARED_LIBRARY_SUFFIX})
  include_directories($ENV{WEBOTS_HOME}/include/controller/c $ENV{WEBOTS_HOME}/include/controller/cpp)
else()  # C
  set (LIBRARIES ${CMAKE_SHARED_LIBRARY_PREFIX}Controller${CMAKE_SHARED_LIBRARY_SUFFIX})
  include_directories($ENV{WEBOTS_HOME}/include/controller/c)
endif()

add_custom_command(TARGET ${PROJECT} POST_BUILD COMMAND ${CMAKE_COMMAND} -E
  copy ${CMAKE_BINARY_DIR}/${PROJECT} ${CMAKE_SOURCE_DIR}
)

find_package(pinocchio REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen pinocchio::pinocchio)

# 将选项的值传递给C++源代码中的宏
add_definitions(-DRUN_WEBOTS=1)
