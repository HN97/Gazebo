cmake_minimum_required(VERSION 3.0.2)
project(testing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs roscpp rospy std_msgs tf2 tf2_ros  std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS geometry_msgs std_msgs roscpp rospy std_msgs tf2 tf2_ros)

catkin_install_python(PROGRAMS 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

###########
## Build ##
###########

# add_definitions(-DLOCAL -DPID)
add_definitions(-DLOCAL)

include_directories(
        ${catkin_INCLUDE_DIRS} ../../kalman_filter
)
file(GLOB SOURCES src/*.cpp ../../kalman_filter/*.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCES} )
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


