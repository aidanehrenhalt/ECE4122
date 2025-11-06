# CMake Entry Point
cmake_minimum_required(VERSION 3.0)
project(Lab4 LANGUAGES CXX CUDA)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find CUDA
find_package(CUDAToolkit REQUIRED)

# Set CUDA Architecture (Adjust based on target GPU)
set(CMAKE_CUDA_ARCHITECTURES 70 75)

# Compiler Flags -- Note: Letter 'O' not Number '0'
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall")

# Source Files
set(SOURCES lab4.cu)

# Create Executable
add_executable(lab4 ${SOURCES})

# Set Cuda Properties
set_target_properties(lab4 PROPERTIES CUDA_SEPERABLE_COMPILATION ON CUDA_RESOLVE_DEVICE_SYMBOLS ON)

# Link CUDA Libraries
target_link_libraries(lab4 CUDA::cudart)

# Custom Target for Clean Up
add_custom_target(clean_all
    COMMAND ${CMAKE_COMMAND} -E remove lab4
    COMMAND ${CMAKE_COMMAND} -E remove finalTemperature.csv
    COMMAND ${CMAKE_COMMAND} -E remove_directory CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E remove CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E remove cmake_install.cmake
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

# End of CMakeLists.txt