cmake_minimum_required(VERSION 3.5)

project(aether_bringup)

find_package(ament_cmake REQUIRED)
find_package(aether_description REQUIRED)
find_package(aether_gazebo REQUIRED)
find_package(aether_app REQUIRED)

add_custom_command(
    OUTPUT
        ${CMAKE_CURRENT_SOURCE_DIR}/config/robot_config.yaml
    DEPENDS
        ${MONOREPO_DIR}/config/nominal_config.yaml
    COMMAND python3
        ${MONOREPO_DIR}/config/generate_full_config.py
        ${MONOREPO_DIR}/config/nominal_config.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/config/robot_config.yaml
)
add_custom_target(
    gen_config_target ALL
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/config/robot_config.yaml
)

# Install project launch files
install(
    DIRECTORY
        launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
    DIRECTORY
        config/
    DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
