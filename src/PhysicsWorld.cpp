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
#include "mt.h"

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
    std::cout << "Check amongst " << mCollidingPairs.size() << " objects" << std::endl;
    std::unordered_map<RigidBodyPair, RigidBody::CollidingType>::iterator it;
    ContactModel *contactModel;
    for( it = mCollidingPairs.begin() ; it != mCollidingPairs.end(); it++ ) {
        RigidBodyPair pair = it->first;
        std::cout << "Rigid model for object " << pair.rigidBody1->getId() << " and " << pair.rigidBody2->getId() << std::endl;
        contactModel = pair.rigidBody1->distanceMeshToMesh(pair.rigidBody2);

        //if(contactModel->distance <= minDistance) {
            ////std::cout << "Narrow phase collision with distance: " << contactModel->distance << std::endl;
            //it->second = RigidBody::CollidingType::NARROW_PHASE;
        //}
    }
}

void PhysicsWorld::reactToCollision(ContactModel *contact)
{
    glm::vec3 n = contact->normal;
    std::cout << "n:" << " ( " << n.x << ", " << n.y << ", " << n.z << " )" << std::endl;


    //glm::vec3 n(0, 1, 0);

    float epsilon = 1.f;
    glm::vec3 va = contact->rigidBody1->getLinearVelocity();
    glm::vec3 vb = contact->rigidBody2->getLinearVelocity();
    float vrel = /*mt::norm(va-vb); */ glm::dot(n, va-vb);

    std::cout << "epsilon: " << epsilon << std::endl;
    std::cout << "va:" << " ( " << va.x << ", " << va.y << ", " << va.z << " )" << std::endl;
    std::cout << "vb:" << " ( " << vb.x << ", " << vb.y << ", " << vb.z << " )" << std::endl;
    std::cout << "vrel: " << vrel << std::endl;

    float N = -(1.f+epsilon) * vrel;
    std::cout << "N:" << " " << N << std::endl;

    float t1 = contact->rigidBody1->getInvMass();
    float t2 = contact->rigidBody2->getInvMass();
    std::cout << "t1 (= 1/m1): " << t1 << std::endl;
    std::cout << "t2 (= 1/m2): " << t2 << std::endl;

    glm::mat4 IbodyAInv = contact->rigidBody1->getInverseInertialTensor();
    glm::mat4 IbodyBInv = contact->rigidBody2->getInverseInertialTensor();


    glm::mat4 Ra = contact->rigidBody1->getRotation();
    glm::mat4 Rb = contact->rigidBody2->getRotation();
    glm::mat4 RaT = glm::transpose(Ra);
    glm::mat4 RbT = glm::transpose(Rb);

    glm::mat4 IaInv = Ra * IbodyAInv * RaT;
    glm::mat4 IbInv = Rb * IbodyBInv * RbT;

    std::cout << "IaInv : " << std::endl
	<< "\t" << IaInv[0][0] << ", " << IaInv[0][1] << ", " << IaInv[0][2] << std::endl
	<< "\t" << IaInv[1][0] << ", " << IaInv[1][1] << ", " << IaInv[1][2] << std::endl
	<< "\t" << IaInv[2][0] << ", " << IaInv[2][1] << ", " << IaInv[2][2] << std::endl;
    std::cout << "IbInv : " << std::endl
	<< "\t" << IbInv[0][0] << ", " << IbInv[0][1] << ", " << IbInv[0][2] << std::endl
	<< "\t" << IbInv[1][0] << ", " << IbInv[1][1] << ", " << IbInv[1][2] << std::endl
	<< "\t" << IbInv[2][0] << ", " << IbInv[2][1] << ", " << IbInv[2][2] << std::endl;

    // XXX: Use contact point 1 and 2 instead
    glm::vec3 ra = contact->contactPoint - (contact->rigidBody1->getPosition() + contact->rigidBody1->getCenterOfMass());
    glm::vec3 rb = contact->contactPoint - (contact->rigidBody2->getPosition() + contact->rigidBody2->getCenterOfMass());
    std::cout << "ra:" << " ( " << ra.x << ", " << ra.y << ", " << ra.z << " )" << std::endl;
    std::cout << "rb:" << " ( " << rb.x << ", " << rb.y << ", " << rb.z << " )" << std::endl;





    glm::vec3 cross1 = glm::cross(ra, n);
    glm::vec3 cross2 = glm::cross( glm::vec3(IaInv * glm::vec4(cross1, 0.)), ra);
    float t3 = glm::dot( n, glm::cross( cross2, ra ) );
    cross1 = glm::cross(rb, n);
    cross2 = glm::cross( glm::vec3(IbInv * glm::vec4(cross1, 0.)), rb);
    float t4 = glm::dot( n, glm::cross( cross2, rb ) );

    float j = N / (t1 + t2 + t3 + t4);
    std::cout << "j: " << j << std::endl;

    glm::vec3 J = j * n;
    std::cout << "J:" << " ( " << J.x << ", " << J.y << ", " << J.z << " )" << std::endl;

    glm::vec3 omegaA = glm::vec3( IaInv * glm::vec4(glm::cross(ra, J), 0.f) );
    glm::vec3 omegaB = glm::vec3( IbInv * glm::vec4(glm::cross(rb, J), 0.f) );
    std::cout << "omegaA:" << " ( " << omegaA.x << ", " << omegaA.y << ", " << omegaA.z << " )" << std::endl;
    std::cout << "omegaB:" << " ( " << omegaB.x << ", " << omegaB.y << ", " << omegaB.z << " )" << std::endl;


    contact->rigidBody1->updateFromImpulse(J, omegaA);
    contact->rigidBody2->updateFromImpulse(J, omegaB);

    std::cout << "\n\n";
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
