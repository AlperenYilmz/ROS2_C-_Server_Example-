cmake_minimum_required(VERSION 3.8)
project(lesson69act4cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_interfaces REQUIRED)

add_executable(battery_node src/batarya.cpp)
ament_target_dependencies(battery_node rclcpp custom_interfaces)
add_executable(ledpanel_node src/ledpanel.cpp)
ament_target_dependencies(ledpanel_node rclcpp custom_interfaces)


install(TARGETS battery_node ledpanel_node
        DESTINATION lib/${PROJECT_NAME})

ament_package()