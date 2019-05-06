cmake_minimum_required(VERSION 2.8.3)
project(roscoap)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Qt5 COMPONENTS Core Gui)

include_directories(
  ${catkin_INCLUDE_DIRS}
  /home/arsonist/Qt/5.9.6/gcc_64/
)
add_executable(coap-client src/coap-client.cpp)
target_link_libraries(coap-client ${catkin_LIBRARIES} libcoap-2 Qt5::Widgets)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17") 

