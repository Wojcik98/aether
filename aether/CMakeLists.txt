cmake_minimum_required(VERSION 3.14)
project(aether)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_custom_command(
    OUTPUT
        ${CMAKE_CURRENT_SOURCE_DIR}/include/aether/robot_config.hpp
    DEPENDS
        ${MONOREPO_DIR}/config/nominal_config.yaml
        ${MONOREPO_DIR}/config/templates/robot_config.hpp.em
    COMMAND python3
        ${MONOREPO_DIR}/config/generate_full_config.py
        ${MONOREPO_DIR}/config/nominal_config.yaml
        ${CMAKE_CURRENT_SOURCE_DIR}/include/aether/robot_config.hpp
)

# Create the library target
add_library(${PROJECT_NAME} STATIC
    src/mapped_localization.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/aether/robot_config.hpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    include
    ../third_party/eigen
)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
