cmake_minimum_required(VERSION 3.10)
project(remote_controller_cpp_server LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(Franka REQUIRED CONFIG)
find_package(Eigen3 REQUIRED)

find_path(XMLRPC_INCLUDE_DIR NAMES xmlrpc-c/base.hpp REQUIRED)

find_library(XMLRPC_SERVERPP_LIB NAMES xmlrpc_server++ REQUIRED)
find_library(XMLRPC_ABYSSPP_LIB NAMES xmlrpc_server_abyss++ REQUIRED)
find_library(XMLRPCPP_LIB NAMES xmlrpc++ REQUIRED)

add_executable(remote_controller_server
  src/main.cpp
  src/moveJ.cpp
  src/graspO.cpp
  src/udp_state_publisher.cpp
  src/gripperhome.cpp
  src/gripperrelease.cpp
  src/robot_init.cpp
  src/robot_manager.cpp
  src/rpc_methods.cpp
  src/moveC.cpp
  src/moveJ_path.cpp
  src/moveC_path.cpp
  src/trackJ.cpp
  src/trackJ2.cpp
  src/trackJ_udp_receiver.cpp
  src/trackC.cpp
  src/trackC2.cpp
  src/trackC_udp_receiver.cpp
)

target_include_directories(remote_controller_server PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${XMLRPC_INCLUDE_DIR}
)

target_link_libraries(remote_controller_server PRIVATE
  Franka::Franka
  Eigen3::Eigen
  ${XMLRPC_SERVERPP_LIB}
  ${XMLRPC_ABYSSPP_LIB}
  ${XMLRPCPP_LIB}
  Threads::Threads
)
