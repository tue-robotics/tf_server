cmake_minimum_required(VERSION 2.8.3)
project(tf_server)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs tf message_generation)

catkin_python_setup()

add_service_files( FILES LookupTransform.srv TransformPoint.srv TransformPose.srv WaitForTransform.srv)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS roscpp geometry_msgs tf message_runtime)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_server src/tf_server.cpp)
target_link_libraries(tf_server ${catkin_LIBRARIES})
add_dependencies(tf_server ${PROJECT_NAME}_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/tf_client.test)
endif()