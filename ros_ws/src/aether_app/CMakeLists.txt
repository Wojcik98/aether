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

add_subdirectory(../../../aether aether)

add_executable(map_visualizer src/map_visualizer.cpp)
target_include_directories(map_visualizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(map_visualizer PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(map_visualizer aether)
ament_target_dependencies(map_visualizer rclcpp visualization_msgs)

install(TARGETS map_visualizer
  DESTINATION lib/${PROJECT_NAME})

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
