cmake_minimum_required(VERSION 3.8)
project(second_project)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_index_cpp REQUIRED) #per trovare il percorso per il csv file


# Stage has appeared under different CMake/pkg-config layouts across distros.
find_package(stage QUIET)
if(NOT stage_FOUND)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(STAGE REQUIRED stage)
endif()

#first executable
add_executable(convert_scan src/convert_scan.cpp)
ament_target_dependencies(convert_scan rclcpp sensor_msgs)

#second executable
add_executable(stageros src/stageros.cpp)
ament_target_dependencies(stageros
  rclcpp
  geometry_msgs
  nav_msgs
  rosgraph_msgs
  sensor_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

#third executable
add_executable(pass_1_action src/pass_1_action.cpp)
ament_target_dependencies(pass_1_action rclcpp rclcpp_action tf2 nav2_msgs)

#fourth executable
add_executable(pass_poses src/pass_poses.cpp)
ament_target_dependencies(pass_poses rclcpp rclcpp_action tf2 nav2_msgs ament_index_cpp geometry_msgs)


if(stage_FOUND)
  target_include_directories(stageros PRIVATE ${STAGE_INCLUDE_DIRS})
  target_link_libraries(stageros ${STAGE_LIBRARIES})
else()
  target_include_directories(stageros PRIVATE ${STAGE_INCLUDE_DIRS})
  target_link_directories(stageros PRIVATE ${STAGE_LIBRARY_DIRS})
  target_link_libraries(stageros ${STAGE_LIBRARIES})
endif()

if(UNIX AND NOT APPLE)
  target_link_libraries(stageros dl)
endif()

install(TARGETS
  convert_scan
  stageros
  pass_1_action
  pass_poses
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  config
  rviz
  map
  world 
  csv
  DESTINATION share/${PROJECT_NAME}
)


ament_package()