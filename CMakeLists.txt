cmake_minimum_required(VERSION 3.8)
project(hex_cpp_template)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

message(STATUS "Your ROS version is ROS $ENV{ROS_VERSION}")
set(HEX_ROS_VERSION $ENV{ROS_VERSION})
add_definitions(-DHEX_ROS_VERSION=${HEX_ROS_VERSION}) 

if (${HEX_ROS_VERSION} STREQUAL "1")
    set(CMAKE_CXX_FLAGS "-std=c++14")

    find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
    )
    find_package(hex_cpp_utils REQUIRED)

    catkin_package()

    include_directories(
        ../
        ./
        include
        ${catkin_INCLUDE_DIRS}
    )

    add_executable(hex_cpp_template
        hex_cpp_template/main.cc
        hex_cpp_template/hex_cpp_template.cc
        hex_cpp_template/data_interface/ros1_interface.cc
    )

    target_link_libraries(hex_cpp_template
        ${catkin_LIBRARIES}
        hex_cpp_utils::hex_cpp_utils
    )

    install(
        TARGETS hex_cpp_template
        DESTINATION lib/${PROJECT_NAME}
    )

    install(
        DIRECTORY launch/ros1
        DESTINATION share/${PROJECT_NAME}/launch
    )

    install(
        DIRECTORY config/ros1
        DESTINATION share/${PROJECT_NAME}/config
    )

elseif (${HEX_ROS_VERSION} STREQUAL "2")
    set(CMAKE_CXX_FLAGS "-std=c++17")

    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(hex_cpp_utils REQUIRED)
    include_directories(
        ../
        ./
        include
        ${colcon_INCLUDE_DIRS}
    )

    ament_package()

    add_executable(hex_cpp_template
        hex_cpp_template/main.cc
        hex_cpp_template/hex_cpp_template.cc
        hex_cpp_template/data_interface/ros2_interface.cc
    )

    target_link_libraries(hex_cpp_template
        hex_cpp_utils::hex_cpp_utils
    )

    ament_target_dependencies(hex_cpp_template
        rclcpp
        std_msgs
        nav_msgs
    )

    install(
        TARGETS hex_cpp_template
        DESTINATION lib/${PROJECT_NAME}
    )

    install(
        DIRECTORY launch/ros2
        DESTINATION share/${PROJECT_NAME}/launch
    )

    install(
        DIRECTORY config/ros2
        DESTINATION share/${PROJECT_NAME}/config
    )

else ()
    message(FATAL_ERROR "Unknown ROS version, please install the ros_environment package")
endif ()
