cmake_minimum_required(VERSION 2.8.3)
project(tf_server)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  geometry_msgs
  tf
)

# Generate services in the 'srv' folder
 add_service_files(
   FILES
   LookupTransform.srv
   TransformPoint.srv
   TransformPose.srv
 )

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_server src/tf_server.cpp)
target_link_libraries(tf_server ${catkin_LIBRARIES})

