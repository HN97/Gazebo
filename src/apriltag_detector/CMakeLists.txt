cmake_minimum_required(VERSION 2.8.3)
project(apriltag_detector)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        cv_bridge geometry_msgs roscpp rospy std_msgs tf2 tf2_ros image_transport std_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS cv_bridge geometry_msgs std_msgs roscpp rospy std_msgs tf2 tf2_ros image_transport)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS} ../../apriltag
)

add_executable(${PROJECT_NAME}_node src/apriltag_detector.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}   "/usr/local/lib/libapriltag.so"
        )
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
