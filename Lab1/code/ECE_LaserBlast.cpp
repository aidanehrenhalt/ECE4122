/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 09/26/2025

Description: ECE_LaserBlast implementation file

*/

#include "ECE_LaserBlast.h"
#include <iostream>

ECE_LaserBlast::ECE_LaserBlast(const std::string& textureFile, float startX, float startY, bool moveUp, float screenH) : speed(600.0f), movingUp(moveUp), screenHeight(screenH)
{
    // Load Laser Texture
    if (!laserTexture.loadFromFile(textureFile))
    {
        std::cerr << "Error loading laser texture: " << textureFile << std::endl;
    }

    // Set Laser Texture / Position
    setTexture(laserTexture);
    // Center laser origin so bounding box aligns with visual sprite
    sf::Vector2u texSize = laserTexture.getSize();
    if (texSize.x > 0 && texSize.y > 0)
    {
        setOrigin(static_cast<float>(texSize.x) / 2.0f, static_cast<float>(texSize.y) / 2.0f);
    }
    setPosition(startX, startY);
}

void ECE_LaserBlast::update(float deltaTime)
{
    // Move laser in appropriate direction
    if (movingUp)
    {
        setPosition(getPosition().x, getPosition().y - speed * deltaTime);
    }
    else
    {
        setPosition(getPosition().x, getPosition().y + speed * deltaTime);
    }
}

bool ECE_LaserBlast::isOffScreen() const
{
    float y = getPosition().y;
    float h = getGlobalBounds().height;

    // Check laser position relative to screen bounds
    if (movingUp && y + h < 0.0f) // Check top boundary -- Blast moving up
    {
        return true;
    }
    else if (!movingUp && y > screenHeight) // Check bottom boundary -- Blast moving down
    {
        return true;
    }
    return false;
}

// Check blast interaction with target sprite
bool ECE_LaserBlast::checkCollision(const sf::Sprite& other) const
{
    return getGlobalBounds().intersects(other.getGlobalBounds());
}