/*
Author: Aidan (Ace) Ehrenhalt
Class: ECE 4122-A
Last Date Modified: 09/26/2025

Description: Header file for ECE_Buzzy class
- Player controlled entity / sprite
- Player can move left and right within screen bounds
- Player can shoot laser blasts downward towards enemies

*/

#ifndef ECE_BUZZY_H
#define ECE_BUZZY_H

#include <SFML/Graphics.hpp>
#include <list>

// ECE_LaserBlast - Forward Declaration
class ECE_LaserBlast;

/**
 * ECE_Buzzy Class -- Player controlled sprite from sf::Sprite
 * Class handles player input for movement and shooting, and collision detection
 */
class ECE_Buzzy : public sf::Sprite
{
    private:
        sf::Texture buzzyTexture; // Texture - Buzzy Sprite
        float movementSpeed;
        float fireRate;
        sf::Clock fireClock;
        float screenWidth;
        float screenHeight;

    public:
        /**
         * Constructor - Initialize Buzzy sprite
         * @oaran textureFile - path to texture file
         * @param startX - starting X position
         * @param startY - starting Y position
         * @param screenW - screen width for maintaining boundaries
         */
    ECE_Buzzy(const std::string& textureFile, float startX, float startY, float screenW, float screenH);

        /**
         * Update Buzzy position based on player input
         * @param deltaTime - time since previous frame
         */
        void update(float deltaTime);

        /**
         * Handle player inputs for movement / shooting
         * @param laserBlasts - references list of laser blasts and updates upon shooting
         */
        void handleInput(std::list<ECE_LaserBlast>& laserBlasts);

        /**
         * Reset Buzzy to starting position
         */
        void reset(float startX, float startY);

        /**
         * Check Buzzy collision with other sprites
         * @param other - reference to other sprite
         * @return true if collision detected, otherwise false
         */
        bool checkCollision(const sf::Sprite& other) const;
};

#endif // ECE_BUZZY_H