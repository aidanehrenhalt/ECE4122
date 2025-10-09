# CMakeLists.txt for Lab2
cmake_minimum_required(VERSION 3.10)

project(Lab2 VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Point to your SFML installation
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../SFML/install" ${CMAKE_PREFIX_PATH})

# Find SFML
find_package(SFML 2.6 COMPONENTS graphics window system REQUIRED)

# Find OpenMP
find_package(OpenMP REQUIRED)
# Including OpenMP for later, not used yet

# Get your source files
file(GLOB SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

# Create executable
add_executable(Lab2 ${SOURCES})

# Link libraries
target_link_libraries(Lab2 PRIVATE sfml-graphics sfml-window sfml-system OpenMP::OpenMP_CXX)
# Including OpenMP for later, not used yet