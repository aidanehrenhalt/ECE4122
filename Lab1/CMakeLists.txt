# CMakeLists.txt for Lab1
cmake_minimum_required(VERSION 3.10)

project(Lab1 VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Point to your SFML installation
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../SFML-2.6.1/install" ${CMAKE_PREFIX_PATH})

# Find SFML
find_package(SFML 2.6 COMPONENTS graphics window system REQUIRED)

# Get your source files
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/code/*.cpp")

if(NOT SOURCES)
    message(FATAL_ERROR "No source files found in code/ directory")
endif()

# Create executable
add_executable(Lab1 ${SOURCES})

# Link libraries
target_link_libraries(Lab1 PRIVATE sfml-graphics sfml-window sfml-system)

# Linux system libraries
if(UNIX AND NOT APPLE)
    target_link_libraries(Lab1 PRIVATE pthread)
endif()