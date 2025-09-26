/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 09/26/2025

Description: ECE_Enemy implementation file

*/

#include "ECE_Enemy.h"
#include <iostream>

// <Class::Constructor()> : <Initializer List>
ECE_Enemy::ECE_Enemy(const std::string& textureFile, float startX, float startY) : alive(true)
{
    // Load Enemy Texture
    if (!enemyTexture.loadFromFile(textureFile))
    {
        std::cerr << "Error loading enemy texture: " << textureFile << std::endl;
    }

    // Set Enemy Texture / Position
    setTexture(enemyTexture);
    setPosition(startX, startY);

    // Scale enemy sprite to reasonable size to avoid overlap on high-res displays
    const float desiredEnemyWidth = 64.0f; // pixels
    sf::Vector2u texSize = enemyTexture.getSize();
    if (texSize.x > 0 && texSize.y > 0)
    {
        float scale = desiredEnemyWidth / static_cast<float>(texSize.x);
        setScale(scale, scale);
    }
}

// Check enemy collision with other sprite(s)
bool ECE_Enemy::checkCollision(const sf::Sprite& other)
{
    if(!alive)
    {
        return false; // If enemy dead, ignore collision
    }
    return getGlobalBounds().intersects(other.getGlobalBounds());
}