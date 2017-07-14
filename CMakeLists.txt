cmake_minimum_required(VERSION 2.8.3)
project(ground_filter)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_libraries(${PCL_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   CATKIN_DEPENDS cv_bridge sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/ground_filter_Timimg.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
)

