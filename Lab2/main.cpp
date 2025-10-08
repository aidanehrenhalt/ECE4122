/*
Author: Aidan Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 10/08/2025

Description: Main file for Conway's Game of Life implementation. 
Parses command-line args, Sets up SFML window, runs main game loop. 
*/

#include <SFML/Graphics.hpp>
#include <iostream>
#include <string>
#include <random>
#include "GameOfLife.h"

/*
Function Name: parseArgs
Inputs: int argc, char *argv[], int &numThreads, int &cellSize, int &gridWidth, int &gridHeight, int std::string &procType) 
-- procType = "SEQ" | "THRD" | "OMP"
Return: true if successful, otherwise false 

Description: Parses command-line args for game config. 
Contains default values for args if not provided.
*/

bool parseArgs(int argc, char *argv[], int &numThreads, int &cellSize, int &windowWidth, int &windowHeight, std::string &procType)
{
    // Default Values
    numThreads = 8;
    cellSize = 5;
    windowWidth = 800;
    windowHeight = 600;
    // procType = "THRD";    
    
    // For Basic Implementation, Set Default to Sequential (SEQ)
    // Ensure Default is Multithreading (THRD) for final submission.
    procType = "SEQ";

    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];

        // Parse args - Convert string to int (std::stoi) when needed
        if (arg == "-n" && i + 1 < argc)
        {
            numThreads = std::stoi(argv[++i]);

            if (numThreads < 2)
            {
                std::cerr << "Error: Minimum Thread Count = 2." << std::endl;
                
                return false;
            }
        }
        else if (arg == "-c" && i + 1 < argc)
        {
            cellSize = std::stoi(argv[++i]);

            if (cellSize < 1)
            {
                std::cerr << "Error: Minimum Cell Size = 1." << std::endl;
                
                return false;
            }
        }
        else if (arg == "-x" && i + 1 < argc)
        {
            windowWidth = std::stoi(argv[++i]);
        }
        else if (arg == "-y" && i + 1 < argc)
        {
            windowHeight = std::stoi(argv[++i]);
        }
        else if (arg == "-t" && i + 1 < argc)
        {
            procType = argv[++i];

            if (procType != "SEQ" && procType != "THRD" && procType != "OMP")
            {
                std::cerr << "Error: Processing type must be Sequential 'SEQ', Multithreading 'THRD', or OpenMP 'OMP'." << std::endl;
                
                return false;
            }
        }
        else
        {
            std::cerr << "Unknown argument: " << arg << std::endl;
            
            return false;
        } // End if-else
    } // End for loop

    return true;
} // End parseArgs()

/*
Function Name: main
Inputs: int argc, char *argv[]

Return: 0 upon exit

Description: Main Function for implementing Conway's Game of Life.
Sets up SFML window, initializes game state, runs main game loop.
*/
int main(int argc, char *argv[])
{
    // Game Config Variables
    int numThreads;
    int cellSize;
    int windowWidth;
    int windowHeight;
    std::string procType;

    // Parse Command-Line Args - Check for errors
    if (!parseArgs(argc, argv, numThreads, cellSize, windowWidth, windowHeight, procType))
    {
        std::cerr << "Usage: " << argv[0] << " [-n numThreads] [-c cellSize] [-x windowWidth] [-y windowHeight] [-t procType]" << std::endl;
        std::cerr << "  -n numThreads   : Number of threads (default: 8, min: 2)" << std::endl;
        std::cerr << "  -c cellSize     : Cell size (px) (default: 5, min: 1)" << std::endl;
        std::cerr << "  -x windowWidth  : Window width (px) (default: 800)" << std::endl;
        std::cerr << "  -y windowHeight : Window height (px) (default: 600)" << std::endl;
        std::cerr << "  -t procType     : Processing type: 'SEQ' (Sequential), 'THRD' (Multithreading), 'OMP' (OpenMP) (default: 'SEQ' (Change to 'THRD' for final submission))" << std::endl;

        return EXIT_FAILURE;
    }

    // Display Config
    std::cout << "Conway's Game of Life Config:" << std::endl;
    std::cout << "Window Size: " << windowWidth << "x" << windowHeight << " px" << std::endl; // (width)x(height)
    std::cout << "Cell Size: " << cellSize << " px" << std::endl;
    std::cout << "Grid Size: " << (windowWidth / cellSize) << "x" << (windowHeight / cellSize) << " cells" << std::endl; // (Window Size [W or H] / Cell Size)
    std::cout << "Processing Type: " << procType << std::endl;

    // Display Number of Threads if using Multithreading or OpenMP
    if (procType != "SEQ")
    {
        std::cout << "Number of Threads: " << numThreads << std::endl;
    }

    std::cout << std::endl;

    // Initialize Game Instance
    GameOfLife game(windowWidth, windowHeight, cellSize, numThreads, procType); // Grid Size Processing in Constructor

    // Create Game Window
    sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Ace -- Conway's Game of Life");
    window.setFramerateLimit(60); // Cap at 60 FPS

    // Game Loop
    while (window.isOpen())
    {
        // Event Handling
        sf::Event event;

        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
            {
                window.close();
            }
            else if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::Escape)
                {
                    window.close();
                } // End Key Processing
            } // End Event Processing if-else
        } // End Event Polling Loop

        // Update Game State
        // -- Implement Game of Life Logic Later
        // -- Testing Window / Rendering for now
        game.update();

        // Window Rendering
        window.clear(sf::Color::Black); // Black Background
        game.render(window);              // Render Game State
        window.display();               // Display Frame
        

    } // End Game Loop

    return 0;
} // End main()