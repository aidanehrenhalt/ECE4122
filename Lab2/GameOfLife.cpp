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

GameOfLife::GameOfLife(int windowWidth, int windowHeight, int cellSize, int numThreads, const std::string &procType) 
    : windowWidth(windowWidth), windowHeight(windowHeight), cellSize(cellSize), numThreads(numThreads), procType(procType)
{
    // Grid Dimensions
    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    // Init current / next grid state
    currentGrid.resize(gridHeight, std::vector<bool>(gridWidth, false));
    nextGrid.resize(gridHeight, std::vector<bool>(gridWidth, false));

    // Setup cell shape for rendering
    cellShape.setSize(sf::Vector2f(cellSize, cellSize));
    cellShape.setFillColor(sf::Color::Green);

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

void GameOfLife::update()
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
} // End update()

void GameOfLife::render(sf::RenderWindow &window)
{
    int gridWidth = windowWidth / cellSize;
    int gridHeight = windowHeight / cellSize;

    // Draw alive cells (green) on black grid
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