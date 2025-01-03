cmake_minimum_required(VERSION 3.8)
project(aether_app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(aether_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_subdirectory(../../../aether aether)

add_library(sensors_sync_component SHARED src/sensors_sync.cpp)
target_compile_features(sensors_sync_component PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(sensors_sync_component
  rclcpp
  aether_msgs
  sensor_msgs
  message_filters
  nav_msgs
  rclcpp_components
)
rclcpp_components_register_node(
  sensors_sync_component
  PLUGIN "SensorsSync"
  EXECUTABLE sensors_sync
)

add_executable(map_visualizer src/map_visualizer.cpp)
target_include_directories(map_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(map_visualizer PUBLIC c_std_99 cxx_std_17)
target_link_libraries(map_visualizer aether)
ament_target_dependencies(map_visualizer rclcpp visualization_msgs)

add_executable(path_visualizer src/path_visualizer.cpp)
target_include_directories(path_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(path_visualizer PUBLIC c_std_99 cxx_std_17)
target_link_libraries(path_visualizer aether)
ament_target_dependencies(path_visualizer
  rclcpp
  nav_msgs
  tf2_ros
  geometry_msgs
  tf2_geometry_msgs
)

add_library(mapped_localization_component SHARED src/mapped_localization_node.cpp)
target_include_directories(mapped_localization_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(mapped_localization_component PUBLIC c_std_99 cxx_std_17)
target_link_libraries(mapped_localization_component aether)
ament_target_dependencies(mapped_localization_component
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_ros
  geometry_msgs
  visualization_msgs
  aether_msgs
  rclcpp_components
)
rclcpp_components_register_node(
  mapped_localization_component
  PLUGIN "MappedLocalizationNode"
  EXECUTABLE mapped_localization_node
)

add_library(execute_path_component SHARED src/execute_path.cpp)
target_include_directories(execute_path_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(execute_path_component PUBLIC c_std_99 cxx_std_17)
target_link_libraries(execute_path_component aether)
ament_target_dependencies(execute_path_component
  rclcpp
  nav_msgs
  sensor_msgs
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  visualization_msgs
  aether_msgs
  rclcpp_components
  std_srvs
)
rclcpp_components_register_node(
  execute_path_component
  PLUGIN "ExecutePath"
  EXECUTABLE execute_path
)

ament_export_targets(export_sensors_sync_component)
install(TARGETS sensors_sync_component
        EXPORT export_sensors_sync_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_mapped_localization_component)
install(TARGETS mapped_localization_component aether
        EXPORT export_mapped_localization_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_targets(export_execute_path_component)
install(TARGETS execute_path_component
        EXPORT export_execute_path_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(TARGETS
  map_visualizer
  mapped_localization_node
  sensors_sync
  path_visualizer
  execute_path

  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_motion_model_test test/motion_model_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_map_test test/map_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_tof_reading_test test/tof_reading_test.cpp)
  target_include_directories(${PROJECT_NAME}_motion_model_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(${PROJECT_NAME}_map_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(${PROJECT_NAME}_tof_reading_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_motion_model_test aether)
  target_link_libraries(${PROJECT_NAME}_map_test aether)
  target_link_libraries(${PROJECT_NAME}_tof_reading_test aether)
endif()

ament_package()
