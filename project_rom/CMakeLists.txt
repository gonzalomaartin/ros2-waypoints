cmake_minimum_required(VERSION 3.8)
project(project_rom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Encuentra dependencias necesarias ANTES de usarlas
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(project_rom REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WayPointPath.msg"
  DEPENDENCIES
    geometry_msgs
    std_msgs
 )

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
)

add_library(
  waypoint_tools_lib
  "src/waypoint_tools/WayPointPathTools.cpp"
)

target_include_directories(waypoint_tools_lib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_BINARY_DIR}/rosidl_generator_cpp
)

ament_target_dependencies(
  waypoint_tools_lib
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  rosidl_default_runtime
)

add_dependencies(waypoint_tools_lib ${cpp_typesupport_target})

# Compila el nodo C++
add_executable(pure_pursuit_node
src/pure_pursuit_odom.cpp)

ament_target_dependencies(
  pure_pursuit_node
  rclcpp
  std_msgs
  nav_msgs
  tf2_ros
  tf2
  tf2_geometry_msgs
  visualization_msgs
)

add_executable(waypoints_node
src/waypoints_node.cpp)

ament_target_dependencies(
  waypoints_node
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
)

add_executable(obstacle_avoider_node 
src/obstacle_avoider_node.cpp)

ament_target_dependencies(
  obstacle_avoider_node
  rclcpp
  std_msgs
  sensor_msgs
)


add_dependencies(pure_pursuit_node ${cpp_typesupport_target})
add_dependencies(waypoints_node ${cpp_typesupport_target})
add_dependencies(obstacle_avoider_node ${cpp_typesupport_target})

target_link_libraries(pure_pursuit_node waypoint_tools_lib "${cpp_typesupport_target}")
target_link_libraries(waypoints_node yaml-cpp "${cpp_typesupport_target}")

# Instala ejecutables
install(TARGETS
  waypoints_node
  pure_pursuit_node
  obstacle_avoider_node
  DESTINATION lib/${PROJECT_NAME}
)

# Instala carpetas de recursos
install(
  DIRECTORY scenes launch urdf rviz msg config 
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS waypoint_tools_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Testing opcional
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
