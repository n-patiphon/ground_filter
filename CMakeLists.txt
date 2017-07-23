cmake_minimum_required(VERSION 2.8.3)
project(ground_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_ros
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS sensor_msgs cv_bridge
)

###########
## Build ##
###########
find_package(PCL 1.7)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

SET(CMAKE_CXX_FLAGS "-std=c++11 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

link_directories(${PCL_LIBRARY_DIRS})


#Ground Filter
add_executable(ground_filter nodes/ground_filter/ground_filter.cpp)
target_link_libraries(ground_filter ${catkin_LIBRARIES} ${PCL_LIBRARIES})
