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
    totalTime(0), generationCount(0), gridWidth(windowWidth / cellSize), gridHeight(windowHeight / cellSize)
{
    // Init current / next grid state
    currGrid.resize(gridHeight, std::vector<uint8_t>(gridWidth, 0));
    nextGrid.resize(gridHeight, std::vector<uint8_t>(gridWidth, 0));
    
    // Setup cell shape for rendering
    cellShape.setSize(sf::Vector2f(cellSize, cellSize));
    cellShape.setFillColor(sf::Color::White);

    // Init grid with cells randomly set as alive or dead
    initGrid();
} // End GameOfLife()

void GameOfLife::initGrid()
{
    // Create RNG
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);

    // Randomize cell states alive / dead
    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            currGrid[y][x] = (dis(gen) < 0.50); // 50% chance cell is alive
        } // End Grid Width Loop
    } // End Grid Height Loop
} // End initGrid()

int GameOfLife::countNeighbors(int x, int y) const
{
    int numNeighbors = 0;
    
    // Check all neighboring cells
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            // Replace Modulo with More Efficient Calculation
            int count = 0;
            int xLeft = (x == 0) ? (gridWidth - 1) : (x - 1);
            int xRight = (x == gridWidth - 1) ? 0 : (x + 1);
            int yUp = (y == 0) ? (gridHeight - 1) : (y - 1);
            int yDown = (y == gridHeight - 1) ? 0 : (y + 1);

            count += currGrid[yUp][xLeft]   + currGrid[yUp][x]     + currGrid[yUp][xRight];
            count += currGrid[y][xLeft]     + 0  /* Cell Itself */ + currGrid[y][xRight];
            count += currGrid[yDown][xLeft] + currGrid[yDown][x]   + currGrid[yDown][xRight];

            return count;
        } // End Neighbor X Loop
    } // End Neighbor Y Loop

    return numNeighbors;
} // End countNeighbors()

void GameOfLife::updateSequential()
{
    // Sequential Implementation for now
    // Will add Multithreading and OpenMP later

    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            int numNeighbors = countNeighbors(x, y);
            nextGrid[y][x] = computeCellState(currGrid[y][x], numNeighbors);
        } // End Grid Width Loop    
    } // End Grid Height Loop
    
    // Proceed to next generation (nextGrid becomes currGrid)
    currGrid.swap(nextGrid);
} // End updateSequential()

void GameOfLife::updateMultithreaded()
{
    // Vector for Multithreading
    std::vector<std::thread> threads;

    // Rows per thread
    int rowsPerThread = gridHeight / numThreads;
    int remainingRows = gridHeight % numThreads;

    // Lambda function for multithreading
    auto threadWork = [this](int startRow, int endRow)
    {
        for (int y = startRow; y < endRow; ++y)
        {
            for (int x = 0; x < gridWidth; ++x)
            {
                int numNeighbors = countNeighbors(x, y);
                nextGrid[y][x] = computeCellState(currGrid[y][x], numNeighbors);
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
    currGrid.swap(nextGrid);
} // End updateMultithreaded()

void GameOfLife::updateOpenMP()
{
    // Set number of threads for OpenMP
    omp_set_num_threads(numThreads);

    // Tilize grid instead of Rows
    const int TILE_SIZE = 32; // Tile size for better performance (1kb each)

    // Create tiles for the grid
    int numTilesX = (gridWidth + TILE_SIZE - 1) / TILE_SIZE;
    int numTilesY = (gridHeight + TILE_SIZE - 1) / TILE_SIZE;

    // Parallelization with OpenMP
    #pragma omp parallel for collapse(2) schedule(dynamic, 1)
    for (int tileY = 0; tileY < numTilesY; ++tileY)
    {
        for (int tileX = 0; tileX < numTilesX; ++tileX)
        {
            // Calculate tile coords
            int startX = tileX * TILE_SIZE;
            int startY = tileY * TILE_SIZE;
            int endX = std::min(startX + TILE_SIZE, gridWidth);
            int endY = std::min(startY + TILE_SIZE, gridHeight);

            // Process each cell in tile
            for (int y = startY; y < endY; ++y)
            {
                for (int x = startX; x < endX; ++x)
                {
                    int numNeighbors = countNeighbors(x, y);
                    nextGrid[y][x] = computeCellState(currGrid[y][x], numNeighbors);
                }
            } // End Cell Processing Loop
        } // End Grid Width Loop    
    } // End Grid Height Loop

    // Continue to next generation
    currGrid.swap(nextGrid);
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
    // Draw alive cells (white) on black grid
    for (int y = 0; y < gridHeight; ++y)
    {
        for (int x = 0; x < gridWidth; ++x)
        {
            if (currGrid[y][x])
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