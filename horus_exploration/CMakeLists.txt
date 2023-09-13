cmake_minimum_required(VERSION 3.0.2)
project(horus_exploration)

find_package(catkin REQUIRED COMPONENTS
  mrs_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/generate_traj.cpp)


target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
