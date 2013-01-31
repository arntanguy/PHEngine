/******************************************************************************
     Copyright (C) 2012-2013  TANGUY Arnaud arn.tanguy@gmail.com
*                                                                             *
* This program is free software; you can redistribute it and/or modify        *
* it under the terms of the GNU General Public License as published by        *
* the Free Software Foundation; either version 2 of the License, or           *
* (at your option) any later version.                                         *
*                                                                             *
* This program is distributed in the hope that it will be useful,             *
* but WITHOUT ANY WARRANTY; without even the implied warranty of              *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                *
* GNU General Public License for more details.                                *
*                                                                             *
* You should have received a copy of the GNU General Public License along     *
* with this program; if not, write to the Free Software Foundation, Inc.,     *
* 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.                 *
 ******************************************************************************/

#ifndef __particules__
#define __particules__

//#include "GL/glew.h"
#include <iostream>
#include <cstdlib>
#include "GL/gl.h"
#include <glm/glm.hpp>
#include <vector>

#define MAX_PARTICLES 1000
#define MAX_MODIFIERS 10

/**
 * Struct particle: represents a particle
 */
typedef struct
{   bool active; // Active
    double life; // Lifespan
    double fade; // Speed of fading
    glm::vec3 color; // RGB values for colour
    glm::vec3 position;
    glm::vec3 velocity;
    glm::vec3 force; // Force accumulator

    float mass; // Particle mass

    double size; // Size of the particle, usually very small (0.1)
} Particle;


class PlaneEntity;

class ParticleGenerator
{
    private:
        Particle *mParticles;
        int mMaxParticles;
        glm::vec3 mCameraLocation;
        glm::vec3 mEmitterPosition;

        std::vector<glm::vec3> mForces;

        std::vector<float> mModifiersDistance;
        std::vector<glm::vec3> mColorModifiers;

        std::vector<PlaneEntity*> mCollisionPlanes;

        void initParticles();
        void initParticle(int i);
        void applyForces(int i, float timestep);
        void handleCollisions(int i);


    public:
        float mG, mA; // Gravity and Archimede
        float mBounce; // Bounce factor
        ParticleGenerator(glm::vec3 emitterPos, int maxParticles);
        ~ParticleGenerator();
        void render(float ellapsedtime);
        void changeSize(double size);

        void addForce(glm::vec3 f);
        bool addColorModifier(float distance, const glm::vec3& color);

        void addCollisionPlane(PlaneEntity *plane);

        void setGravity(float G) {
            if(G>=0) mG = G;
            else mG=0;
        }

        void setBounce(float B) {
            if(B>=0) mBounce = B;
            else mBounce = 0;
        }

        void setVisquosity(float V) {
            if(V>=0) mA = V;
            else mA = 0;
        }
};


double myRand(double min, double max);
float dot(glm::vec3 v1, glm::vec3 v2);


#endif
