cmake_minimum_required(VERSION 2.6)
SET(PROJECT_NAME control_task2)
PROJECT(${PROJECT_NAME})
find_package(Eigen3)
add_compile_definitions(__USE_SINGLE_PRECISION__)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3 -pthread" )
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/../build)

include_directories( 
    "/home/pi/Documents/tinystrech/controllers/tinystretch/include_vmc"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/include_wb"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/vmc_inc"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/mpc"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/vision_location"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/math_src"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/cube_vision"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/share_mem"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/qpOASES/include"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/rbdl_src/urdf"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/rbdl_src/rbdl"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/rbdl_src/model"
    "/home/pi/Documents/tinystrech/controllers/tinystretch/rbdl_src/common_utils"
)

link_directories("/usr/local/lib")
include_directories("/usr/local/include")
#link_directories("/home/pi/Downloads/cross-gcc/aarch64-linux-gnu/lib")
include_directories("/home/pi/Downloads/cross-gcc/aarch64-linux-gnu/include")


AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/gait_src DIR_SRCS)
AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/locomotion_src DIR_SRCS1)
AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/math_src DIR_SRCS2)
AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/mpc DIR_SRCS3)
AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/share_mem DIR_SRCS4)
AUX_SOURCE_DIRECTORY(./src DIR_SRCS5)
AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/rbdl_src/urdf  DIR_SRCS6)
AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/rbdl_src/rbdl  DIR_SRCS7)
AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/rbdl_src/model DIR_SRCS8)
AUX_SOURCE_DIRECTORY(/home/pi/Documents/tinystrech/controllers/tinystretch/rbdl_src/common_utils DIR_SRCS9)

ADD_EXECUTABLE(${PROJECT_NAME} ${DIR_SRCS} ${DIR_SRCS1}  ${DIR_SRCS2} ${DIR_SRCS3} ${DIR_SRCS4} ${DIR_SRCS5}
    ${DIR_SRCS6} ${DIR_SRCS7} ${DIR_SRCS8} ${DIR_SRCS9}
    )

add_subdirectory(/home/pi/Documents/tinystrech/controllers/tinystretch/qpOASES lib/ EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} qpOASES)

add_subdirectory(/home/pi/Documents/tinystrech/controllers/tinystretch/yaml-cpp lib/ EXCLUDE_FROM_ALL)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

#add_subdirectory(./pinocchio lib/ EXCLUDE_FROM_ALL)
#target_link_libraries(${PROJECT_NAME} pinocchio)

set(CMAKE_COLOR_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 14)

#find_package(pinocchio REQUIRED)
#target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen pinocchio::pinocchio)

# 将选项的值传递给C++源代码中的宏
add_definitions(-DRUN_WEBOTS=0)
