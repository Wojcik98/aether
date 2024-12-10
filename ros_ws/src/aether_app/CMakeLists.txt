cmake_minimum_required(VERSION 3.8)
project(aether_app)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_subdirectory(../../../aether aether)

add_executable(aether_test src/aether_test.cpp)
target_include_directories(aether_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(aether_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(aether_test aether)

install(TARGETS aether_test
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_localization_test test/localization_test.cpp)
  ament_add_gtest(${PROJECT_NAME}_map_test test/map_test.cpp)
  target_include_directories(${PROJECT_NAME}_localization_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(${PROJECT_NAME}_map_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME}_localization_test aether)
  target_link_libraries(${PROJECT_NAME}_map_test aether)
endif()

ament_package()
