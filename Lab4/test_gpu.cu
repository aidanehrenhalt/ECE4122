// File for Testing PACE-ICE Instance for GPU Device
// Created my own because I didn't realize there was an existing one provided...
#include <iostream>
#include <cuda_runtime.h>

int main() {
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    
    std::cout << "CUDA Error Code: " << error << std::endl;
    std::cout << "Error String: " << cudaGetErrorString(error) << std::endl;
    std::cout << "Device Count: " << deviceCount << std::endl;
    
    if (deviceCount > 0) {
        cudaDeviceProp prop;
        cudaGetDeviceProperties(&prop, 0);
        std::cout << "GPU Name: " << prop.name << std::endl;
        std::cout << "Compute Capability: " << prop.major << "." << prop.minor << std::endl;
        std::cout << "Total Memory: " << prop.totalGlobalMem / (1024*1024) << " MB" << std::endl;
    }
    
    return 0;
}