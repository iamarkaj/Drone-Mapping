cmake_minimum_required(VERSION 3.0.2)
project(drone_slam)

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package()

install(DIRECTORY
  models
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(laser_test src/laser_test.cpp)
target_link_libraries(laser_test ${catkin_LIBRARIES})
add_dependencies(laser_test ${laser_test_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
