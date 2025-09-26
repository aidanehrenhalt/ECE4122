# CMakeLists.txt for ECE Buzzy Defender

# Version Requirement
cmake_minimum_required(VERSION 3.10)

# Project Name / Version
project(Lab1 VERSION 1.0)

# Specify C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Locate SFML
find_package(SFML 2.5 COMPONENTS graphics window system REQUIRED)

## Debugging
message(STATUS "Current Source Dir: = ${CMAKE_CURRENT_SOURCE_DIR}")

# Add Source Files from Code Dir
# file(GLOB SOURCES "${PROJECT_SOURCE_DIR}/code/*.cpp")
file(GLOB SOURCES
    # "/home/hice1/aehrenhalt3/ECE4122/Lab1/code/ECE_Buzzy.cpp"
    # "/home/hice1/aehrenhalt3/ECE4122/Lab1/code/ECE_BuzzyDefender.cpp"
    # "/home/hice1/aehrenhalt3/ECE4122/Lab1/code/ECE_Enemy.cpp"
    # "/home/hice1/aehrenhalt3/ECE4122/Lab1/code/ECE_LaserBlast.cpp"

    "/home/aceaid/ECE4122/Lab1/code/ECE_Buzzy.cpp"
    "/home/aceaid/ECE4122/Lab1/code/ECE_BuzzyDefender.cpp"
    "/home/aceaid/ECE4122/Lab1/code/ECE_Enemy.cpp"
    "/home/aceaid/ECE4122/Lab1/code/ECE_LaserBlast.cpp"
)

# Add Executable
add_executable(Lab1 ${SOURCES})

include_directories(Lab1 PRIVATE ${PROJECT_SOURCE_DIR}/../SFML/include)
## target_include_directories(Lab1 PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/code)

link_directories(${PROJECT_SOURCE_DIR}/../SFML/lib)

# Link Executable to SFML Libraries
## target_link_libraries(Lab1 PRIVATE sfml-graphics sfml-window sfml-system)
target_link_libraries(Lab1 PUBLIC sfml-graphics sfml-system sfml-window)