cmake_minimum_required(VERSION 3.8)
project(collision_reward)


# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)

# Install All Directories (Part 1)
install(DIRECTORY
    urdf
    meshes
    meshes/human
    meshes/pirat
    launch
    worlds
    src
    scripts
    DESTINATION share/${PROJECT_NAME}/
    )



# Launch File Executable
install(
    PROGRAMS
    launch/robot_description_publisher.py
    scripts/spawn_object.py
    DESTINATION lib/${PROJECT_NAME}
    )


# Plugin

add_library(collision_reward_plugin SHARED
  src/collision_reward_plugin.cpp
)
target_include_directories(collision_reward_plugin PUBLIC include)
ament_target_dependencies(collision_reward_plugin
  "gazebo_ros"
  "std_msgs"
)
ament_export_libraries(collision_reward_plugin)


install(TARGETS
    collision_reward_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()

install(DIRECTORY include/
        DESTINATION include)

