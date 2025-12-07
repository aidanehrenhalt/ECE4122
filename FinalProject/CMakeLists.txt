cmake_minimum_required(VERSION 3.10)

# Project Title
project(UAV_Halftime_Show)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find Required Packages
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(Threads REQUIRED)

# Include Directories
include_directories(
    ${OPENGL_INCLUDE_DIRS}
    ${GLUT_INCLUDE_DIRS}
)

# Add Executable
add_executable(FinalProject FinalProject.cpp)

# Link Libraries
target_link_libraries(FinalProject
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    GL
    GLU
    glut
    Threads::Threads
    m   # Math Library
)

# Compiler Warnings
target_compile_options(FinalProject PRIVATE
    -Wall
    -Wextra
    -Wpedantic
)