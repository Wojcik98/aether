idf_component_register()

# Create and import the library targets
add_subdirectory(../../../aether aether_build)

# Publicly link `foo` to `main` component
target_link_libraries(${COMPONENT_LIB} INTERFACE aether)
