cmake_minimum_required(VERSION 2.8.3)
project(controlps)

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
## Declare ROS messages and services
# add_message_files(DIRECTORY msg FILES Num.msg)
# add_service_files(DIRECTORY srv FILES AddTwoInts.srv)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS} ../../kalman_filter
)
file(GLOB SOURCES src/*.cpp ../../kalman_filter/*.cpp)
add_executable(${PROJECT_NAME}_node ${SOURCES} )
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        )
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")


#############
## Install ##
#############

## Mark executables and/or libraries for installation
#install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#        )

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#        launch/detector.launch
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        )

