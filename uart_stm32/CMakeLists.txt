cmake_minimum_required(VERSION 2.8.3)
project(uart_stm32)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    serial
    std_msgs
    tf
)

catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
)

add_executable(uart_stm32 src/uart_stm32.cpp)
target_link_libraries(uart_stm32 ${catkin_LIBRARIES})
