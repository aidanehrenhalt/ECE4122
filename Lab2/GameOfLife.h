/*
Author: Aidan Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 10/08/2025

Description: Header file for GameOfLife class.
Definitions for game grid, cell states, and rendering. 
*/

#ifndef GAMEOFLIFE_H
#define GAMEOFLIFE_H

#include <SFML/Graphics.hpp>
#include <vector>
#include <string>
#include <chrono>

/*
Class Name: GameOfLife

Description: Manages Grid State / Updates and Rendering
*/
class GameOfLife
{
    private:
        int windowWidth;               // Window width (px)
        int windowHeight;              // Window height (px)
        int cellSize;                 // Cell size (px)
        int numThreads;               // Number of threads
        std::string procType;         // Processing type: "SEQ", "THRD", "OMP"

        std::vector<std::vector<bool>> currentGrid; // Current state of the grid
        std::vector<std::vector<bool>> nextGrid;    // Next state of the grid

        sf::RectangleShape cellShape; // Shape for rendering cells

        // Variables for measuring performance
        int generationCount;                        // Track generations
        std::chrono::microseconds totalUpdateTime;  // Total time for 100 generations

        void updateSequential();      // Update grid using Sequential
        void updateMultithreaded();   // Update grid using Multithreading
        void updateOpenMP();          // Update grid using OpenMP

    /*
    Function Name: initGrid
    
    Description: Initialize the grid with cells randomly set as alive or dead
    */
    void initGrid();

    /*
    Function Name: countNeighbors
    Inputs: int x, int y (cell coordinates)
    Return: Number of alive neighbors (0-8)

    Description: Count the number of neighboring cells that are alive
    */
    int countNeighbors(int x, int y) const;

    public:
        /*
        Constructor Name: GameOfLife
        Inputs: int windowWidth, int windowHeight, int cellSize, int numThreads, std::string procType

        Description: Initialize Conway's Game of Life using command-line args (or default values)
        Window Size / Cell Size used to determine Grid Size
        */
       GameOfLife(int windowWidth, int windowHeight, int cellSize, int numThreads, const std::string &procType);

        /*
        Function Name: update
        
        Description: Update the grid proceeding to next generation
        */
        void update();

        /*
        Function Name: render
        Inputs: sf::RenderWindow &window

        Description: Render and display the current grid state
        */
        void render(sf::RenderWindow &window);

        /*
        Function Name: reset

        Description: Reset the grid to new random starting state
        */
        void reset();
};

#endif // GAMEOFLIFE_H