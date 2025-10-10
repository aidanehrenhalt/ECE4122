# CMakeLists.txt for Lab2
cmake_minimum_required(VERSION 3.10)

project(Lab2 VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Point to your SFML installation
set(CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../SFML/install" ${CMAKE_PREFIX_PATH})

# Debugging for PACE-ICE
if(NOT EXISTS ${SFML_DIR})
    # set(SFML_DIR "/home/hice1/aehrenhalt3/SFML-2.6.1/build")
    set(SFML_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../SFML-2.6.1")

    if(EXISTS "${SFML_ROOT}/install/lib64/cmake/SFML/SFMLConfig.cmake")
        set(SFML_DIR "${SFML_ROOT}/install/lib64/cmake/SFML")
        message(STATUS "Found SFML in install directory")
    elseif(EXISTS "${SFML_ROOT}/build/SFMLConfig.cmake")
        set(SFML_DIR "${SFML_ROOT}/build")
        message(STATUS "Found SFML in build directory")
    elseif(EXISTS "${SFML_ROOT}/build/lib/cmake/SFML/SFMLConfig.cmake")
        set(SFML_DIR "${SFML_ROOT}/build/lib/cmake/SFML")
        message(STATUS "Found SFML in build/lib/cmake/SFML")
    else()
        message(WARNING "Could not find SFML cmake files. Searched in:")
        message(STATUS "  ${SFML_ROOT}/install/lib64/cmake/SFML")
        message(STATUS "  ${SFML_ROOT}/build")
        message(STATUS "  ${SFML_ROOT}/build/lib/cmake/SFML")
    endif()

    set(CMAKE_PREFIX_PATH "${SFML_DIR}" ${CMAKE_PREFIX_PATH})
endif()

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