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

#include "PhysicsWorld.h"
#include "RigidBody.h"
#include "AABoundingBox.h"
#include "AABBBroadPhaseCollision.h"
#include "BoundingSphereBroadPhaseCollision.h"
#include <algorithm>

PhysicsWorld::PhysicsWorld()
{
    addBroadPhaseCollisionHandler(new AABBBroadPhaseCollision(this));
    addBroadPhaseCollisionHandler(new BoundingSphereBroadPhaseCollision(this));
    setDebugAll(false);
}
PhysicsWorld::~PhysicsWorld()
{
}

void PhysicsWorld::addRigidBody(RigidBody *rigidBody)
{
    if(rigidBody != 0)
        mRigidBodies.push_back(rigidBody);
}

void PhysicsWorld::addBroadPhaseCollisionHandler(BroadPhaseCollision *broadPhaseCollision)
{
    mBroadPhaseCollision.push_back(broadPhaseCollision);
}

/**
 * @brief Goes through the list of BroadPhase collision handlers,
 * and have them check for collisions.
 */
void PhysicsWorld::detectBroadPhaseCollisions()
{
    mCollidingPairs.clear();
    if(mBroadPhaseCollision.size() != 0) {
        std::unordered_map<RigidBodyPair, AxisCollide> collidingPairs;
        std::vector<BroadPhaseCollision *>::iterator it;
        std::unordered_map<RigidBodyPair, AxisCollide>::iterator cit;
        for( it = mBroadPhaseCollision.begin() ; it != mBroadPhaseCollision.end(); it++ ) {
            (*it)->update();

            collidingPairs = (*it)->getCollidingPairs();
            for( cit = collidingPairs.begin() ; cit != collidingPairs.end(); cit++ ) {
                if(cit->second.collide()) {
                   mCollidingPairs[cit->first] = RigidBody::CollidingType::BROAD_PHASE;
                } else {
                   mCollidingPairs[cit->first] = RigidBody::CollidingType::NONE;
                }
            }
        }

    }
}

void PhysicsWorld::checkCollisions(float minDistance)
{
    detectBroadPhaseCollisions();
    std::unordered_map<RigidBodyPair, RigidBody::CollidingType>::iterator it;
    ContactModel *contactModel;
    for( it = mCollidingPairs.begin() ; it != mCollidingPairs.end(); it++ ) {
        RigidBodyPair pair = it->first;
        contactModel = pair.rigidBody1->distanceMeshToMesh(pair.rigidBody2);

        if(contactModel->distance <= minDistance) {
            std::cout << "Narrow phase collision with distance: " << contactModel->distance << std::endl;
            it->second = RigidBody::CollidingType::NARROW_PHASE;
        }
    }

}

void PhysicsWorld::debugBroadPhase(RigidBody *rigidBody)
{
    if(mDebugBroadPhase) {
        RigidBody::CollidingType ct = rigidBody->getCollidingType();
        if(ct == RigidBody::CollidingType::NONE)
            rigidBody->getBoundingBox()->render(false);
        else
            rigidBody->getBoundingBox()->render(true);
    }
}

void PhysicsWorld::debugNarrowPhase(RigidBody *rigidBody)
{
    if(mDebugNarrowPhase) {
        RigidBody::CollidingType ct = rigidBody->getCollidingType();
        if(ct == RigidBody::CollidingType::NARROW_PHASE) {
            debugBroadPhase(rigidBody);
            // XXX: do something with narrow phase
        }
    }
}

void PhysicsWorld::renderAllRigidBodies(float timestep)
{
    // Set debug info
    std::unordered_map<RigidBodyPair, RigidBody::CollidingType>::iterator cit = mCollidingPairs.begin();
    for(; cit != mCollidingPairs.end(); cit++) {
        cit->first.rigidBody1->setCollide(cit->second);
        cit->first.rigidBody2->setCollide(cit->second);
    }

    // Render
    std::vector<RigidBody *>::iterator it;
    for(it = mRigidBodies.begin(); it != mRigidBodies.end(); it++) {
        (*it)->render(timestep);
        debugBroadPhase(*it);
        debugNarrowPhase(*it);
    }

    // Restore debug info
    cit = mCollidingPairs.begin();
    for(; cit != mCollidingPairs.end(); cit++) {
        cit->first.rigidBody1->setCollide(RigidBody::CollidingType::NONE);
        cit->first.rigidBody2->setCollide(RigidBody::CollidingType::NONE);
    }
}

std::vector<RigidBody *> PhysicsWorld::getRigidBodies() const
{
    return mRigidBodies;
}

void PhysicsWorld::setDebugAll(bool status)
{
    mDebugBroadPhase = status;
    mDebugNarrowPhase = status;
}

void PhysicsWorld::setDebugBroadPhase(bool status)
{
    mDebugBroadPhase = status;
}

void PhysicsWorld::setDebugNarrowPhase(bool status)
{
    mDebugNarrowPhase = status;
}
