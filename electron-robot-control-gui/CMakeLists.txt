cmake_minimum_required(VERSION 2.8)

project (ros_addon)
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs motor_control)

include_directories(${CMAKE_JS_INC} ${catkin_INCLUDE_DIRS} node_modules/nan/)
file(GLOB SOURCE_FILES "ros_addon.cpp")
add_library(${PROJECT_NAME} SHARED ${SOURCE_FILES})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "" SUFFIX ".node")
target_link_libraries(${PROJECT_NAME}
					${CMAKE_JS_LIB}
					${catkin_LIBRARIES})
