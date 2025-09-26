/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 09/26/2025

Description: Header file for ECE_LaserBlast class
- Laser blasts (projectiles) fired by player / enemies
*/

#ifndef ECE_LASERBLAST_H
#define ECE_LASERBLAST_H

#include <SFML/Graphics.hpp>

/**
 * ECE_LaserBlast Class -- Laser projectile derived from sf::Sprite
 * Handles movement, collision detection, and blast lifetime (eliminate upon reaching screen limit / hitting target)
 */
class ECE_LaserBlast : public sf::Sprite
{
    private:
        sf::Texture laserTexture;
        float speed; // Movement speed of blast
        bool movingUp; // Movement direction - true = up, false = down
        float screenHeight; // Height for checking screen bounds

    public:
        /**
         * Constructor - Creates laser blast
         * @param textureFile - File path to laser texture
         * @param startX - Starting X position
         * @param startY - Starting Y position
         * @param moveUp - Movement Direction (true = up, false = down)
         * @param screenH - Screen height for bounds checking
         */
        ECE_LaserBlast(const std::string& textureFile, float startX, float startY, bool moveUp, float screenH);

        /**
         * Update laser position
         * @return true if off screen, otherwise false
         */
        bool isOffScreen() const;

        /**
         * Check if laser collides with target sprite
         * @param other - reference to other sprite
         * @return true if collision detected, otherwise false
         */
        bool checkCollision(const sf::Sprite& other) const;
};

#endif // ECE_LASERBLAST_H