cmake_minimum_required(VERSION 3.0.2)
project(ros_bag_adapter)
add_compile_options(-std=c++11)

# include(FindProtobuf)

find_package(
  catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  std_msgs
  topic_tools
  ros_type_introspection
  rosbag
)

# This forces all the protofiles to be built to
# object files for linking. 
file(
  GLOB
  INCLUDES
  ${PROJECT_SOURCE_DIR}/../../protos/protos/*/v1/*
)

find_package(protobuf)# CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES adapter
  # CATKIN_DEPENDS ros_type_introspection topic_tools roscpp std_msgs# std_msgs
  # DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/../../protos
  ${PROJECT_SOURCE_DIR}/../../protos/protos
  ${PROTOBUF_INCLUDE_DIR}
)

add_executable(
  main
  src/main.cpp
  ${INCLUDES}
  ${PROJECT_SOURCE_DIR}/../../protos
  ${PROJECT_SOURCE_DIR}/../../protos/protos
)

target_link_libraries(
  main
  ${catkin_LIBRARIES}
  gRPC::grpc++
  protobuf::libprotobuf
)

