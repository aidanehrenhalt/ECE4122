/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 09/26/2025

Description: Main game file for Buzzy Defender

*/

#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>
#include <list>
#include <random>
#include "ECE_Buzzy.h"
#include "ECE_LaserBlast.h"
#include "ECE_Enemy.h"

// Game States
enum class GameState
{
    Start,
    Playing,
    GameOver
};

// Game Constants
const int WINDOW_WIDTH = 1920; // 1080p Resolution - Width
const int WINDOW_HEIGHT = 1080; // 1080p Resolution - Height
const int ENEMY_ROWS = 6; // Enemy Formation - Rows
const int ENEMY_COLS = 10; // Enemy Formation - Columns
const float ENEMY_SPACING_X = 80.0f; // Horizontal Spacing - Enemies
const float ENEMY_SPACING_Y = 70.0f; // Vertical Spacing - Enemies
const float FORMATION_START_X = 400.0f; // Starting Position - X - Enemy Formation
const float FORMATION_START_Y = 150.0f; // Starting Position - Y - Enemy Formation (near top)
const float FORMATION_SPEED = 100.0f; // Movement Speed - Enemy Formation
const float FORMATION_ADVANCE_STEP = 30.0f; // Shift - Enemy Formation

/**
 * Reset game / all entities to initial state
 */
void resetGame(
    ECE_Buzzy& buzzy, 
    std::vector<ECE_Enemy>& enemies, 
    std::list<ECE_LaserBlast>& laserBlasts, 
    std::list<ECE_LaserBlast>& enemyLasers
);

/**
 * Update enemy formation position 
 */
void updateEnemyFormation(
    std::vector<ECE_Enemy>& enemies,
    float deltaTime,
    bool& movingRight,
    float& formationY
);

/**
 * Get bounds of enemy formation
 */
void getFormationBounds(
    const std::vector<ECE_Enemy>& enemies,
    float& minX, 
    float& maxX, 
    float& minY, 
    float& maxY
);

/**
 * Handle enemies shooting
 */
void handleEnemyShooting(
    const std::vector<ECE_Enemy>& enemies,
    std::list<ECE_LaserBlast>& enemyLasers,
    std::mt19937& rng,
    sf::Clock& enemyFireClock
);

/**
 * Check Win / Loss Conditions
 */
bool checkWinCond(const std::vector<ECE_Enemy>& enemies);
bool checkLossCond(const ECE_Buzzy& buzzy, const std::vector<ECE_Enemy>& enemies);

// Main Function
int main()
{
    // Create Window
    // Use windowed mode by default for easier testing; switch to Fullscreen when ready
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "ECE Buzzy Defender", sf::Style::Fullscreen);

    // Game State
    GameState currentState = GameState::Start;
    
    // Create Rendering Window
    sf::Texture backgroundTexture;
    if (!backgroundTexture.loadFromFile("graphics/background.png")) // Background Texture from Space Invaders
    {
        std::cerr << "Error loading background texture" << std::endl;
        return -1;
    }
    sf::Sprite backgroundSprite(backgroundTexture);

    // Load Start Screen Texture
    sf::Texture startScreenTexture;
    if(!startScreenTexture.loadFromFile("graphics/Start_Screen.png"))
    {
        std::cerr << "Error loading start screen texture" << std::endl;
        return -1;
    }
    sf::Sprite startScreenSprite(startScreenTexture);
    startScreenSprite.setPosition(0, 0);    

    // Create Game Objects
    sf::Texture buzzyTexture;
    ECE_Buzzy buzzy(
        "graphics/Buzzy_blue.png", 
        WINDOW_WIDTH / 2.0f - 50.0f, 
        WINDOW_HEIGHT - 200.0f, 
        static_cast<float>(WINDOW_WIDTH),
        static_cast<float>(WINDOW_HEIGHT)
    );

    std::vector<ECE_Enemy> enemies;
    enemies.reserve(ENEMY_ROWS * ENEMY_COLS);

    // Initialize Enemy Formation
    for (int row = 0; row < ENEMY_ROWS; ++row)
    {
        std::string textureFile = (row % 2 == 0) ? "graphics/bulldog.png" : "graphics/clemson_tigers.png";

        for (int col = 0; col < ENEMY_COLS; ++col)
        {
            float x = FORMATION_START_X + col * ENEMY_SPACING_X;
            float y = FORMATION_START_Y + row * ENEMY_SPACING_Y;
            enemies.emplace_back(textureFile, x, y);
        }
    }

    // Lists - Enemy / Player Lasers
    std::list<ECE_LaserBlast> playerLasers;
    std::list<ECE_LaserBlast> enemyLasers;

    // Variables - Enemy Formation Movement
    bool movingRight = true;
    float formationY = FORMATION_START_Y;

    // Random Number Generation - Enemy Shooting
    std::random_device rd;
    std::mt19937 rng(rd());
    sf::Clock enemyFireClock; // Clock - Enemy Shooting Rate Limiter

    // Game Timer
    sf::Clock gameClock;
    const float FIXED_TIME_STEP = 1.0f / 120.0f; // Fixed Time Step - Update 120 FPS
    float accumulator = 0.0f; // Keep track of game time

    // Main Game Loop
    while (window.isOpen())
    {
        float deltaTime = gameClock.restart().asSeconds();
        accumulator += deltaTime;

        // Handle Events
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed || sf::Keyboard::isKeyPressed(sf::Keyboard::Escape))
            {
                window.close();
            }

            if (event.type == sf::Event::KeyPressed)
            {
                if (event.key.code == sf::Keyboard::Enter)
                {
                    if (currentState == GameState::Start)
                    {
                        currentState = GameState::Playing;
                        resetGame(buzzy, enemies, playerLasers, enemyLasers);
                    }
                    else if (currentState == GameState::GameOver)
                    {
                        currentState = GameState::Start;
                    }
                }
            }
        }

        // Update Game State - Fixed Time Step
        while (accumulator >= FIXED_TIME_STEP)
        {
            if (currentState == GameState::Playing)
            {
                // Update Buzzy
                buzzy.update(FIXED_TIME_STEP);
                buzzy.handleInput(playerLasers);

                // Update Enemy Formation
                updateEnemyFormation(enemies, FIXED_TIME_STEP, movingRight, formationY);

                // Handle Enemies Shooting
                handleEnemyShooting(enemies, enemyLasers, rng, enemyFireClock);

                // Update Player Lasers
                auto playerIt = playerLasers.begin();
                while (playerIt != playerLasers.end())
                {
                    playerIt->update(FIXED_TIME_STEP);
                    
                    if (playerIt->isOffScreen())
                    {
                        playerIt = playerLasers.erase(playerIt);
                        continue;
                    }

                    // Check Blast Collision with Enemies
                    bool hitEnemy = false;
                    for (auto& enemy : enemies)
                    {
                        if (enemy.isAlive() && playerIt->checkCollision(enemy))
                        {
                            enemy.kill();
                            hitEnemy = true;
                            break;
                        }
                    }

                    if (hitEnemy)
                    {
                        playerIt = playerLasers.erase(playerIt);
                    }
                    else
                    {
                        ++playerIt;
                    }
                }

                // Update Enemy Lasers
                auto enemyIt = enemyLasers.begin();
                while (enemyIt != enemyLasers.end())
                {
                    enemyIt->update(FIXED_TIME_STEP);

                    if (enemyIt->isOffScreen())
                    {
                        enemyIt = enemyLasers.erase(enemyIt);
                        continue;
                    }

                    // Check collision with player
                    if (enemyIt->checkCollision(buzzy))
                    {
                        // Remove laser and flag loss
                        enemyIt = enemyLasers.erase(enemyIt);
                        currentState = GameState::GameOver;
                        // Early out of laser processing
                        break;
                    }

                    ++enemyIt;
                }

                // Check Win / Loss Conditions
                if (checkWinCond(enemies))
                {
                    currentState = GameState::GameOver;
                }
                else if (checkLossCond(buzzy, enemies))
                {
                    currentState = GameState::GameOver;
                }
            }

            accumulator -= FIXED_TIME_STEP;
        }

        // Rendering
        window.clear();

        // Draw Background
        window.draw(backgroundSprite);

        if (currentState == GameState::Start)
        {
            // Draw Start Screen
            window.draw(startScreenSprite);
        }
        else if (currentState == GameState::Playing)
        {
            // Draw Game Objects
            window.draw(buzzy);

            for (const auto& enemy : enemies)
            {
                if (enemy.isAlive())
                {
                    window.draw(enemy);
                }
            }

            for (const auto& laser : playerLasers)
            {
                window.draw(laser);
            }

            for (const auto& laser : enemyLasers)
            {
                window.draw(laser);
            }
        }
        else if (currentState == GameState::GameOver)
        {
            // Game Over -- Go back to displaying Start Screen
            window.draw(startScreenSprite);
        }

        window.display();
    }

    return 0;
}

void resetGame(ECE_Buzzy& buzzy, std::vector<ECE_Enemy>& enemies, std::list<ECE_LaserBlast>& playerLasers, std::list<ECE_LaserBlast>& enemyLasers)
{
    // Reset Player Position (near bottom)
    buzzy.reset(WINDOW_WIDTH / 2.0f - 25.0f, WINDOW_HEIGHT - 200.0f);

    // Reset Enemies
    int index = 0;
    for (int row = 0; row < ENEMY_ROWS; ++row)
    {
        for (int col = 0; col < ENEMY_COLS; ++col)
        {
            float x = FORMATION_START_X + col * ENEMY_SPACING_X;
            float y = FORMATION_START_Y + row * ENEMY_SPACING_Y;
            enemies[index].setPosition(x, y);
            enemies[index].reset();
            ++index;
        }
    }

    // Clear Player / Enemy Lasers
    playerLasers.clear();
    enemyLasers.clear();
}

void updateEnemyFormation(std::vector<ECE_Enemy>& enemies, float deltaTime, bool& movingRight, float& formationY)
{
    // Get Formation Bounds
    float minX, maxX, minY, maxY;
    getFormationBounds(enemies, minX, maxX, minY, maxY);

    // Check for Boundary Collision
    bool contAdvance = false;

    if (movingRight && maxX >= WINDOW_WIDTH - 50.0f)
    {
        movingRight = false;
        contAdvance = true;
    }
    else if (!movingRight && minX <= 50.0f)
    {
        movingRight = true;
        contAdvance = true;
    }

    // Moving Enemy Formation
    float moveDistance = FORMATION_SPEED * deltaTime;
    if (!movingRight) moveDistance = -moveDistance; // If moving left, invert distance

    for (auto& enemy : enemies)
    {
        if (enemy.isAlive())
        {
            sf::Vector2f pos = enemy.getPosition();
            enemy.setPosition(pos.x + moveDistance, pos.y);

            if (contAdvance)
            {
                // Advance formation upward (decrease y) rather than downward
                enemy.setPosition(enemy.getPosition().x, pos.y - FORMATION_ADVANCE_STEP);
            }
        }
    }

    if (contAdvance)
    {
        formationY -= FORMATION_ADVANCE_STEP;
    }
}

void getFormationBounds(const std::vector<ECE_Enemy>& enemies, float& minX, float& maxX, float& minY, float& maxY)
{
    minX = WINDOW_WIDTH;
    maxX = 0;
    minY = WINDOW_HEIGHT;
    maxY = 0;

    for (const auto& enemy : enemies)
    {
        if (enemy.isAlive())
        {
            sf::FloatRect bounds = enemy.getGlobalBounds();
            minX = std::min(minX, bounds.left);
            maxX = std::max(maxX, bounds.left + bounds.width);
            minY = std::min(minY, bounds.top);
            maxY = std::max(maxY, bounds.top + bounds.height);
        }
    }
}

void handleEnemyShooting(const std::vector<ECE_Enemy>& enemies, std::list<ECE_LaserBlast>& enemyLasers, std::mt19937& rng, sf::Clock& enemyFireClock)
{
    // Enemy Fire Rate (Random Interval)
    if (enemyFireClock.getElapsedTime().asSeconds() >= 1.5f)
    {
        // Check Enemies Status - If Alive, Can Shoot
        std::vector<int> aliveIndices;
        for (int i = 0; i < static_cast<int>(enemies.size()); ++i)
        {
            if (enemies[i].isAlive())
            {
                aliveIndices.push_back(i);
            }
        }

        // Pick Random Enemy to Shoot
        if (!aliveIndices.empty())
        {
            std::uniform_int_distribution<int> dist(0, aliveIndices.size() - 1);
            int selectedIndex = aliveIndices[dist(rng)];

            const ECE_Enemy& shooter = enemies[selectedIndex];
            float laserX = shooter.getPosition().x + shooter.getGlobalBounds().width / 2.0f;
            float laserY = shooter.getPosition().y + shooter.getGlobalBounds().height;

            // Enemy lasers move downwards toward the player (moveUp = false)
            enemyLasers.emplace_back("graphics/laser_blast.png", laserX, laserY, true, WINDOW_HEIGHT);
        }

        enemyFireClock.restart();
    }
}

bool checkWinCond(const std::vector<ECE_Enemy>& enemies)
{
    // Check Enemy Status - If All Dead, Player Wins
    for (const auto& enemy : enemies)
    {
        if (enemy.isAlive())
        {
            return false; // At least one enemy still alive
        }
    }
    return true; // All enemies dead
}

bool checkLossCond(const ECE_Buzzy& buzzy, const std::vector<ECE_Enemy>& enemies)
{
    // Check loss: if any alive enemy collides with the player OR has moved up to the player's level.
    // Use intersection of global bounds to detect direct collision, and a conservative
    // Y-position check (enemy top <= buzzy bottom + threshold) for near-misses.
    const float threshold = 10.0f;
    sf::FloatRect buzzyBounds = buzzy.getGlobalBounds();
    float buzzyTop = buzzyBounds.top;

    for (const auto& enemy : enemies)
    {
        if (!enemy.isAlive()) continue;

        sf::FloatRect enemyBounds = enemy.getGlobalBounds();

        // Direct collision
        if (enemyBounds.intersects(buzzyBounds))
        {
            return true;
        }

        // If enemy has moved down to the player's level (enemy bottom >= buzzy top - threshold)
        float enemyBottom = enemyBounds.top + enemyBounds.height;
        if (enemyBottom >= buzzyTop - threshold)
        {
            return true;
        }
    }

    return false;
}