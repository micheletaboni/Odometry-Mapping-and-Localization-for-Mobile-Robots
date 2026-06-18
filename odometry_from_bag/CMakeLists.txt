cmake_minimum_required(VERSION 3.8)
project(first_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(bunker_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(first_project_custom_msgs REQUIRED)

set(srv_files
  "srv/ResetOdometry.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
)

ament_export_dependencies(rosidl_default_runtime)


add_executable(odometer src/odometer.cpp)
ament_target_dependencies(odometer rclcpp nav_msgs bunker_msgs tf2 tf2_ros geometry_msgs rosidl_default_generators first_project_custom_msgs)
add_executable(tf_error src/tf_error.cpp)
ament_target_dependencies(tf_error rclcpp nav_msgs bunker_msgs tf2 tf2_ros geometry_msgs rosidl_default_generators first_project_custom_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(odometer "${cpp_typesupport_target}")

install(TARGETS
  odometer
  tf_error
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz2
  DESTINATION share/${PROJECT_NAME}
)

ament_package()