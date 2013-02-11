/******************************************************************************
     Copyright (C) 2013  TANGUY Arnaud arn.tanguy@gmail.com
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

#ifndef __PhysicsWorld__
#define __PhysicsWorld__

#include <vector>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <glm/glm.hpp>
#include "RigidBody.h"
#include "BroadPhaseCollision.h"


/**
 * @brief Manages the physic world pipeline
 * Register rigid bodies to this class to manage the physic simulation
 * Steps:
 * - Apply BroadPhase collision detection
 */
class PhysicsWorld
{
    protected:
        bool mDebugBroadPhase;
        bool mDebugNarrowPhase;

        void debugBroadPhase(RigidBody *rigidBody);
        void debugNarrowPhase(RigidBody *rigidBody);

    private:
        // Active rigid bodies
        std::vector<RigidBody *> mRigidBodies;
        std::vector<BroadPhaseCollision *> mBroadPhaseCollision;
        std::unordered_map<RigidBodyPair, RigidBody::CollidingType> mCollidingPairs;

    public:
        PhysicsWorld();
        ~PhysicsWorld();

        void addRigidBody(RigidBody *rigidBody);
        void addBroadPhaseCollisionHandler(BroadPhaseCollision *broadPhaseCollision);

        void detectBroadPhaseCollisions();
        void checkCollisions(float minDistance);
        void renderAllRigidBodies(float timestep);
        std::vector<RigidBody *> getRigidBodies() const;


        void setDebugAll(bool status = true);
        void setDebugBroadPhase(bool status = true);
        void setDebugNarrowPhase(bool status = true);
        bool getDebugNarrowPhase() const { return mDebugNarrowPhase; }
        bool getDebugBroadPhase() const { return mDebugBroadPhase; }
};


#endif
