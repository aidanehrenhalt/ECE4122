/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 09/26/2025

Description: ECE_Buzzy class implementation

*/

#include "ECE_Buzzy.h"
#include "ECE_LaserBlast.h"
#include <iostream>

// <Class::Constructor()> : <Initializer List>
ECE_Buzzy::ECE_Buzzy(const std::string& textureFile, float startX, float startY, float screenW, float screenH) : movementSpeed(400.0f), fireRate(0.2f), screenWidth(screenW), screenHeight(screenH)
{
    // Load Buzzy Texture
    if (!buzzyTexture.loadFromFile(textureFile))
    {
        std::cerr << "Error loading Buzzy texture: " << textureFile << std::endl;
    }

    // Set Buzzy Texture / Position
    setTexture(buzzyTexture);
    setPosition(startX, startY);
}

void ECE_Buzzy::update(float deltaTime)
{
    // Handle movement from player input (Ensure we handle continuous input [holding key press])
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Left))
    {
        // Move left, check boundary collision
        float newX = getPosition().x - movementSpeed * deltaTime;
        if (newX >= 0) // Leave position unchanged if left boundary reached
        {
            setPosition(newX, getPosition().y);
        }
    }

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Right))
    {
        // Move right, check boundary collision
        float newX = getPosition().x + movementSpeed * deltaTime;
        if (newX + getGlobalBounds().width <= screenWidth) // Leave position unchanged if right boundary reached
        {
            setPosition(newX, getPosition().y);
        }
    }
}

void ECE_Buzzy::handleInput(std::list<ECE_LaserBlast>& laserBlasts)
{
    // Handle shooting from player input (rate limited upon held key press)
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Space))
    {
        if(fireClock.getElapsedTime().asSeconds() >= fireRate)
        {
            // Create laser blast at player's current position
            float laserX = getPosition().x + getGlobalBounds().width / 2.0f; // Position blast horizontally centered
            float laserY = getPosition().y; // Position blast below player

                // Create player laser moving down (player at top shooting down): moveUp = false
                laserBlasts.emplace_back("graphics/laser_blast.png", laserX, laserY, false, screenHeight);

            fireClock.restart(); // Reset fire clock (rate limiter)
        }
    }
}

void ECE_Buzzy::reset(float startX, float startY)
{
    setPosition(startX, startY); // Reset to starting position
    fireClock.restart();
}

bool ECE_Buzzy::checkCollision(const sf::Sprite& other) const
{
    return getGlobalBounds().intersects(other.getGlobalBounds());
}