/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122
Last Date Modified: 11/06/2025

Description: CUDA program to compute 2D Steady State Heat Conduction Calculations for a Thin Plate
- Uses Jacobi Iteration method to perform Laplace equation calculations
*/

#include <iostream>
#include <vector>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <cuda_runtime.h>

// Kernel for Jacobi Iteration
// Updates interior points based on avg of 4 neighbors & preserves boundary conditions
__global__ void jacobiIterationKernel(const double* __restrict__ d_old, double* __restrict__ d_new, int gridSize)
{
    // Calculate global thread indices
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    int j = blockIdx.y * blockDim.y + threadIdx.y;

    // Check Boundaries
    if (i >= gridSize || j >= gridSize) 
    {
        return;
    }

    int idx = j * gridSize + i;

    // Boundary Points (Fixed)
    if (i == 0 || i == gridSize - 1 || j == 0 || j == gridSize - 1) 
    {
        d_new[idx] = d_old[idx];
    } 
    else 
    {
        // Interior Points (Update - Jacobi Iteration) - Left, Right, Up, Down
        d_new[idx] = 0.25 * (d_old[idx - 1] + d_old[idx + 1] + d_old[idx - gridSize] + d_old[idx + gridSize]);
    }
}

// Init Temperature Grid with Boundary Conditions
void initGrid(std::vector<double>& h, int gridSize)
{
    // Init all points to 20 degrees C
    for (int i = 0; i < gridSize * gridSize; i++)
    {
        h[i] = 20.0;
    }

    // Set hot section on top edge (100 degree C)
    // Hot section is 4 ft wide of 10 ft plate width (centered)
    int sectionStart = static_cast<int>(gridSize * 3 / 10);
    int sectionEnd = static_cast<int>(gridSize * 7 / 10);
    
    // Top Edge (j = 0)
    for (int i = sectionStart; i < sectionEnd; i++)
    {
        h[i] = 100.0;
    }
}

// Write Output to CSV
void writeOutputToCSV(const std::vector<double>& h, int gridSize, const char* filename)
{
    std::ofstream outFile(filename);

    if (!outFile.is_open())
    {
        std::cerr << "Error: Unable to open output file." << std::endl;
        return;
    }

    // Write Comma-Delimited Values for Individual Grid Rows
    for (int j = 0; j < gridSize; j++)
    {
        for (int i = 0; i < gridSize; i++)
        {
            outFile << h[j * gridSize + i];

            if (i < gridSize - 1)
            {
                outFile << ", ";
            }
        }
        outFile << "\n";
    }

    outFile.close(); // Make Sure We Close Output File
}

// Parse Cmd Line Args
bool parseArgs(int argc, char* argv[], int& N, int& iterations, bool& quit)
{
    N = 256;            // Default Grid Size
    iterations = 1000;  // Default Iterations
    quit = false;       // Default Quit Flag

    // Parse Args
    for (int i = 1; i < argc; i++)
    {
        if (strcmp(argv[i], "-N") == 0 && i + 1 < argc)
        {
            N = atoi(argv[++i]);
            if (N < 2)
            {
                std::cerr << "Error: N must be >= 2." << std::endl;
                return false;
            }
        }
        else if (strcmp(argv[i], "-I") == 0 && i + 1 < argc)
        {
            iterations = atoi(argv[++i]);
            
            if (iterations < 1)
            {
                std::cerr << "Error: Iterations must be >= 1." << std::endl;
                return false;
            }
        }
        else if (strcmp(argv[i], "-q") == 0)
        {
            quit = true;
        }
    }

    return true;
}

int main(int argc, char* argv[])
{
    int N;              // Grid Size
    int iterations;     // Number of Iterations
    bool quit;          // Quit Flag

    // Parse Cmd Line Args
    if (!parseArgs(argc, argv, N, iterations, quit))
    {
        std::cerr << "Usage: " << argv[0] << " [-N grid_size] [-I iterations] [-q quit]" << std::endl;
        return 0;
    }

    // Grid Size - Include boundaries: (N + 1) x (N + 1) = Total Points
    int gridSize = N + 1;
    int totalPoints = gridSize * gridSize;

    std::cout << "Running Head Conduction Sim with: " << std::endl;
    std::cout << "Grid Size: " << gridSize << "x" << gridSize << " (" << totalPoints << " points) " << std::endl;
    std::cout << "Iterations: " << iterations << std::endl;

    // Allocate / Init Host Array
    std::vector<double> h(totalPoints);
    initGrid(h, gridSize);

    // Allocate Device Mem for Old / New Arrays for Jacobi Iteration
    double *d_old;
    double *d_new;
    size_t bytes = totalPoints * sizeof(double);

    cudaMalloc(&d_old, bytes);
    cudaMalloc(&d_new, bytes);

    // Copy Initial Grid to Both Arrays
    cudaMemcpy(d_old, h.data(), bytes, cudaMemcpyHostToDevice);
    cudaMemcpy(d_new, h.data(), bytes, cudaMemcpyHostToDevice);

    // Kernel Launch Params Config
    dim3 blockSize(16, 16);
    dim3 gridSizeDim((gridSize + blockSize.x - 1) / blockSize.x, (gridSize + blockSize.y - 1) / blockSize.y);

    // Cuda Events for Timing
    cudaEvent_t start, stop;
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    // Start Timing
    cudaEventRecord(start);

    // Jacobi Iterations
    for (int i = 0; i < iterations; i++)
    {
        // Start Kernel
        jacobiIterationKernel<<<gridSizeDim, blockSize>>>(d_old, d_new, gridSize);

        // Swap Pointers for Next Iteration
        double* temp = d_old;
        d_old = d_new;
        d_new = temp;
    }

    // Stop Timing
    cudaEventRecord(stop);
    cudaEventSynchronize(stop);

    // Calculate Time Elapsed
    float ms = 0; // Milliseconds
    cudaEventElapsedTime(&ms, start, stop);

    // Output Time Elapsed
    std::cout << "Calculation Time: " << ms << " ms" << std::endl;

    // Copy Final Result Back to Host Array
    cudaMemcpy(h.data(), d_old, bytes, cudaMemcpyDeviceToHost); // d_old has final result because of last swap

    // Clean Up Device Mem
    cudaEventDestroy(start);
    cudaEventDestroy(stop);
    cudaFree(d_old);
    cudaFree(d_new);

    return 0;
}