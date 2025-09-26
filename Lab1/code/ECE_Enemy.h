/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 09/26/2025

Description: Header file for ECE_Enemy class
- Enemies are placed in a formation
- Enemies move left and right between screen bounds
- Enemies slowly move upwards as time progresses
- Enemies can shoot laser blasts upward towards the player

*/

#ifndef ECE_ENEMY_H
#define ECE_ENEMY_H

#include <SFML/Graphics.hpp>

/**
 * ECE_Enemy Class -- Enemy sprite from sf::Sprite
 * Represents individual enemies within the formation
 * Handles movement / shooting
 */
class ECE_Enemy : public sf::Sprite
{
    private:
        sf::Texture enemyTexture;
        bool alive;

    public:
        /**
         * Constructor - Initialize Enemy sprite
         * @param textureFile - File path to enemy texture
         * @param startX - Starting X position
         * @param startY - Starting Y position
         */
        ECE_Enemy(const std::string& textureFile, float startX, float startY);

        /**
         * Check status of enemy [Alive / Dead]
         * @return true if alive, otherwise false
         */
        bool isAlive() const 
        {
            return alive;
        }

        /**
         * Set enemy status as dead
         */
        void kill()
        {
            alive = false;
        }

        /**
         * Set enemy status as alive
         */
        void reset()
        {
            alive = true;
        }

        /**
         * Check if enemy collides with another sprite
         * @param other - reference to other sprite
         * @return true if collision detected, otherwise false
         */
        bool checkCollision(const sf::Sprite& other);
};

#endif // ECE_ENEMY_H