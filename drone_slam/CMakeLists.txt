cmake_minimum_required(VERSION 3.0.2)
project(drone_slam)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros laser_geometry)

catkin_package()

install(DIRECTORY
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pcl_icp src/pcl_icp.cpp)
target_link_libraries(pcl_icp ${catkin_LIBRARIES})
add_dependencies(pcl_icp ${pcl_icp_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
