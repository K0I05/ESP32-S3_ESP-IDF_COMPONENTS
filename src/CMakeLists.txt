# Validate CMake version
cmake_minimum_required(VERSION 3.5)

# Append component source files from CMake source directories
FILE(GLOB_RECURSE app_sources "${CMAKE_SOURCE_DIR}/src/*.c")

# Register the component
idf_component_register(
    SRCS ${app_sources} # Set component source files
    INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include # Append component header files from CMake include directories
)
