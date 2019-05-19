cmake_minimum_required(VERSION 2.8.3)
project(roscoap)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package (Qt5Widgets REQUIRED)

include_directories(${Qt5Widgets_INCLUDES})

add_definitions(${Qt5Widgets_DEFINITIONS})

set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

link_directories(
  /usr/local/lib
)

include_directories(
  ${catkin_INCLUDE_DIRS} 
)

set(SOURCES src/coap.cpp src/trik.cpp)
set (MOC_HEADERS src/coap.h src/trik.h)
add_executable(trik ${SOURCES})

#add_executable(trik src/coap.cpp src/trik.cpp)
target_link_libraries(trik ${catkin_LIBRARIES} coap-2 Qt5::Widgets Qt5::Core)
