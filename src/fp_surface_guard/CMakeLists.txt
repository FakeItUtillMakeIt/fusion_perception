cmake_minimum_required(VERSION 3.16)
project(fp_surface_guard)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Surface2D.msg"
  "msg/SurfaceArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

add_library(surface_override_layer SHARED
  src/surface_override_layer.cpp)
target_include_directories(surface_override_layer PUBLIC
  include
  ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(surface_override_layer "${cpp_typesupport_target}")
ament_target_dependencies(surface_override_layer
  rclcpp nav2_costmap_2d pluginlib tf2_geometry_msgs std_msgs geometry_msgs)
target_compile_options(surface_override_layer PRIVATE -O2)
pluginlib_export_plugin_description_file(nav2_costmap_2d surface_override_layer.xml)
install(TARGETS surface_override_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
install(PROGRAMS scripts/surface_builder.py scripts/guard_check.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
