# CMakeLists.txt for ECE Buzzy Defender

# Version Requirement
cmake_minimum_required(VERSION 3.10)

# Project Name / Version
project(Lab1 VERSION 1.0)

# Specify C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Enable Warnings
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## Debugging
message(STATUS "Current Source Dir: = ${CMAKE_CURRENT_SOURCE_DIR}")

# Find SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

# Add Source Files from Code Dir
## file(GLOB_RECURSE SOURCES "${CMAKE_SOURCE_DIR}/code/*.cpp")
file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/code/*.cpp")

# Add Executable
## add_executable(Lab1 PRIVATE ${SOURCES})
add_executable(Lab1 ${SOURCES})

## include_directories(Lab1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/code)
target_include_directories(Lab1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/code)

# Link Executable to SFML Libraries
target_link_libraries(Lab1 PRIVATE sfml-graphics sfml-window sfml-system)