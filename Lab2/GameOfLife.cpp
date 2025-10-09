/*
Author: Aidan Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 10/08/2025

Description: C++ file for GameOfLife class.
Contains logic for grid management, rules for Conway's Game of Life, and handling rendering.
*/

#include "GameOfLife.h"
#include <random>
#include <iostream>
#include <chrono>
#include <thread>
#include <omp.h>

GameOfLife::GameOfLife(int windowWidth, int windowHeight, int cellSize, int numThreads, const std::string &procType) 
    : windowWidth(windowWidth), windowHeight(windowHeight), cellSize(cellSize), numThreads(numThreads), procType(procType),
    totalTime(0), generationCount(0)
{
    // Grid Dimensions
    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    // Init current / next grid state
    currentGrid.resize(gridHeight, std::vector<bool>(gridWidth, false));
    nextGrid.resize(gridHeight, std::vector<bool>(gridWidth, false));

    // Setup cell shape for rendering
    cellShape.setSize(sf::Vector2f(cellSize, cellSize));
    cellShape.setFillColor(sf::Color::White);

    // Init grid with cells randomly set as alive or dead
    initGrid();
} // End GameOfLife()

void GameOfLife::initGrid()
{
    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    // Create RNG
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Randomize cell states alive / dead
    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            currentGrid[y][x] = (dis(gen) < 0.50); // 50% chance cell is alive
        } // End Grid Width Loop
    } // End Grid Height Loop
} // End initGrid()

int GameOfLife::countNeighbors(int x, int y) const
{
    int numNeighbors = 0;
    
    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    // Check all neighboring cells
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            if (dx == 0 && dy == 0)
                continue; // Skip cell itself

            int nx = (x + dx + gridWidth) % gridWidth;
            int ny = (y + dy + gridHeight) % gridHeight;

            if (currentGrid[ny][nx])
            {
                ++numNeighbors;
            }
        } // End Neighbor X Loop
    } // End Neighbor Y Loop

    return numNeighbors;
} // End countNeighbors()

void GameOfLife::updateSequential()
{
    // Sequential Implementation for now
    // Will add Multithreading and OpenMP later

    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            int numNeighbors = countNeighbors(x, y);
            bool currentState = currentGrid[y][x];

            if (currentState)
            {
                // Cell is alive - Apply rule: Stay alive if 2 or 3 neighbors
                nextGrid[y][x] = (numNeighbors == 2 || numNeighbors == 3);
            }
            else
            {
                // Cell is dead - Apply rule: Become alive if exactly 3 neighbors
                nextGrid[y][x] = (numNeighbors == 3);
            }
        } // End Grid Width Loop    
    } // End Grid Height Loop
    
    // Proceed to next generation (nextGrid becomes currentGrid)
    currentGrid.swap(nextGrid);
} // End updateSequential()

void GameOfLife::updateMultithreaded()
{
    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    // Vector for Multithreading
    std::vector<std::thread> threads;

    // Rows per thread
    int rowsPerThread = gridHeight / numThreads;
    int remainingRows = gridHeight % numThreads;

    // Lambda function for multithreading
    auto threadWork = [this, gridWidth](int startRow, int endRow)
    {
        for (int y = startRow; y < endRow; ++y)
        {
            for (int x = 0; x < gridWidth; ++x)
            {
                int numNeighbors = countNeighbors(x, y);
                bool currentState = currentGrid[y][x];

                if (currentState)
                {
                    // Cell is alive - Apply rule: Stay alive if 2 or 3 neighbors
                    nextGrid[y][x] = (numNeighbors == 2 || numNeighbors == 3);
                }
                else
                {
                    // Cell is dead - Apply rule: Become alive if exactly 3 neighbors
                    nextGrid[y][x] = (numNeighbors == 3);
                }
            }
        }
    };

    // Start threads
    int startRow = 0;

    for (int i = 0; i < numThreads; ++i)
    {
        // Distribute rows to threads
        int rows = rowsPerThread + (i < remainingRows ? 1 : 0);
        int endRow = startRow + rows;

        threads.emplace_back(threadWork, startRow, endRow);
        startRow = endRow;
    }

    // Join threads
    for (auto &thread : threads)
    {
        thread.join();
    }

    // Contine to next generation
    currentGrid.swap(nextGrid);
} // End updateMultithreaded()

void GameOfLife::updateOpenMP()
{
    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    // Set number of threads for OpenMP
    omp_set_num_threads(numThreads);

    // Parallelization with OpenMP
    #pragma omp parallel for schedule(static)
    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            int numNeighbors = countNeighbors(x, y);
            bool currentState = currentGrid[y][x];

            if (currentState)
            {
                // Cell is alive - Apply rule: Stay alive if 2 or 3 neighbors
                nextGrid[y][x] = (numNeighbors == 2 || numNeighbors == 3);
            }
            else
            {
                // Cell is dead - Apply rule: Become alive if exactly 3 neighbors
                nextGrid[y][x] = (numNeighbors == 3);
            }
        } // End Grid Width Loop    
    } // End Grid Height Loop

    // Continue to next generation
    currentGrid.swap(nextGrid);
} // End updateOpenMP()

void GameOfLife::update()
{
    // Start time for generation
    auto startTime = std::chrono::high_resolution_clock::now();

    // Call appropriate update function based on procType
    if (procType == "SEQ")
    {
        updateSequential();
    }
    else if (procType == "THRD")
    {
        updateMultithreaded();
    }
    else if (procType == "OMP")
    {
        updateOpenMP();
    }
    else
    {
        std::cerr << "Error: Unknown processing type '" << procType << "'. Defaulting to Sequential." << std::endl;
        updateSequential();
    }

    // End time for generation
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);

    // Time tracking / Generation counter
    totalTime += duration;
    generationCount++;

    // Print average time every 100 generations
    if (generationCount % 100 == 0)
    {
        if (procType == "SEQ")
        {
            std::cout << "Last 100 generations took " << totalTime.count() << " microseconds with a single thread." << std::endl;
        }
        else if (procType == "THRD" || procType == "OMP")
        {
            std::cout << "Last 100 generations took " << totalTime.count() << " microseconds with " << numThreads << " threads." << std::endl;
        }

        // Reset Time tracking / Generation counter
        generationCount = 0;
        totalTime = std::chrono::microseconds(0);
    } // End generationCount if
} // End update()

void GameOfLife::render(sf::RenderWindow &window)
{
    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    // Draw alive cells (white) on black grid
    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            if (currentGrid[y][x])
            {
                cellShape.setPosition(x * cellSize, y * cellSize);
                window.draw(cellShape);
            }
        } // End Grid Width Loop
    } // End Grid Height Loop
} // End render()

void GameOfLife::reset()
{
    initGrid();
} // End reset()