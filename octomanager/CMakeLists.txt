project(octomanager)
cmake_minimum_required(VERSION 3.1.0)

# ####################
# # Project Options ##
# ####################

# # Force some variables that could be defined in the command line to be written to cache
option(WARNINGS_ARE_ERRORS "Treat warnings as errors" OFF)
option(WARNINGS_ANSI_ISO "Issue all the mandatory diagnostics listed in C standard" ON)
option(WARNINGS_EFFCPP "Issue all the warnings listed in the book of Scot Meyers" OFF)

if(${WARNINGS_ANSI_ISO})
   add_compile_options(-Wcast-align)
   add_compile_options(-Wpedantic)
   add_compile_options(-Wextra)
else()
   add_compile_options(-Wno-narrowing)
   add_compile_options(-Wno-delete-non-virtual-dtor)
   add_compile_options(-Wno-unnamed-type-template-args)
endif()

if(${WARNINGS_ARE_ERRORS})
   add_compile_options(-Werror)
endif()

if(${WARNINGS_EFFCPP})
   add_compile_options(-Weffc++)
endif()

# #################################
# # Configure CATKIN dependecies ##
# #################################
find_package(
   catkin REQUIRED
   COMPONENTS std_msgs
   sensor_msgs
   pcl_ros
   pcl_conversions
   roscpp
   rospy
   nav_msgs
   visualization_msgs
   geometry_msgs
   octomap_ros
   octomap_msgs
   tf2
   tf2_ros
   nodelet
   mean_shift_clustering
)

add_service_files(DIRECTORY srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

# ==== Options ====
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-std=c++17)
add_compile_options(-Wall -Wextra)

# ==== Catkin ====
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS
   std_msgs
   sensor_msgs
   geometry_msgs
   pcl_ros
   roscpp
   visualization_msgs
   octomap_ros
   octomap_msgs
)

# ##########
# # Build ##
# ##########
include_directories(include)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
include_directories(${OCTOMAP_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

# # Build a C++ library
file(GLOB_RECURSE SRC_INCS "include/${PROJECT_NAME}/*.h*")
file(GLOB_RECURSE SRC_SRCS "src/*.c*")

add_library(${PROJECT_NAME} ${SRC_SRCS} ${SRC_INCS})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OCTOMAP_LIBRARIES} stdc++fs pcl_visualization)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} yaml-cpp)

# ########################
# # Print configuration ##
# ########################
message(STATUS)
message(STATUS "------------ General configuration for - ${PROJECT_NAME} ------------")
message(STATUS)
message(STATUS "WARNINGS_ANSI_ISO   = ${WARNINGS_ANSI_ISO}")
message(STATUS "WARNINGS_ARE_ERRORS = ${WARNINGS_ARE_ERRORS}")
message(STATUS "WARNINGS_EFFCPP     = ${WARNINGS_EFFCPP}")
message(STATUS)
message(STATUS "BUILD_SHARED_LIBS   = ${BUILD_SHARED_LIBS}")
message(STATUS)
message(STATUS "-------------------------- Dependencies --------------------------")
message(STATUS)
message(STATUS "Change a value with: cmake -D<Variable>=<Value>")
message(STATUS)