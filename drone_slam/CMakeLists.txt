cmake_minimum_required(VERSION 3.0.2)
project(drone_slam)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros laser_geometry)

catkin_package()

include_directories(include 
    ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY
    models
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(pcl_icp_node
    src/pcl_icp_node.cpp
    src/pcl_icp.cpp
)

add_dependencies(pcl_icp_node
    ${pcl_icp_EXPORTED_TARGETS} 
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(pcl_icp_node
    ${catkin_LIBRARIES}
)
