# CMakeList to create a project with an executable, linked to the SGCore C++ Library as a .dll file

cmake_minimum_required (VERSION 3.8)
message(status "Project Directory: ${PROJECT_SOURCE_DIR}")
# project ("SGCore_Example" VERSION 1.0.0)
project ("ik_7dof" VERSION 1.0.0)
# Our main project will be an executable with the main.cpp in the same folder as this file.
# add_executable(${PROJECT_NAME} main_sonny.cpp)
# find_package(Eigen3 REQUIRED)
# target_include_directories(${PROJECT_NAME} PRIVATE ${EIGEN3_INCLUDE_DIR})
# add_executable(exxx SGCoreCpp_TestProgram.cpp)
# target_link_libraries(exxx SGCoreCpp)
# add_executable(for_kr for_kr.cpp)
# target_link_libraries(for_kr SGCoreCpp)
# add_executable(hands hands.cpp)
# target_link_libraries(hands SGCoreCpp)
# target_include_directories(hands PRIVATE ${EIGEN3_INCLUDE_DIR})
# add_executable(trackerss trackerss.cpp)
# target_link_libraries(trackerss SGCoreCpp)
# # target_link_libraries(trackerss Eigen3)
# # find_package(Eigen3 REQUIRED)
# target_include_directories(trackerss PRIVATE ${EIGEN3_INCLUDE_DIR})

add_executable(jiesuan jiesuan.cpp)
# target_include_directories(exxx PRIVATE ${EIGEN3_INCLUDE_DIR})
#Make sure that it is the startup project if we're using VS
# set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})

# message(STATUS "Attempting to add the SGCoreCpp project at ${CMAKE_CURRENT_SOURCE_DIR}../../")
# include(${CMAKE_CURRENT_SOURCE_DIR}/../../LinkSGCoreCpp.cmake) # Runs the module that include SGCoreCpp into this project
message(STATUS "Done!")
set(CMAKE_CXX_FLAGS -pthread)
