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
#include <algorithm>

PhysicsWorld::PhysicsWorld()
{
}
PhysicsWorld::~PhysicsWorld()
{
}

void PhysicsWorld::addRigidBody(RigidBody *rigidBody)
{
    if(rigidBody != 0)
        mRigidBodies.push_back(rigidBody);
}


bool remove_if_end(const AABBComp& c1, const AABBComp& c2) {
    return c2.min > c1.max;
}

void PhysicsWorld::AABBBroadPhase()
{
    std::cout << "AABBBroadPhase()" << std::endl;
    collidingPairs.clear();
    /**
     * Step 1: sort list for all 3 axis by minimum location of bounding box
     **/
    std::multiset<AABBComp, AABBComp> sortedList[3];
    std::vector<RigidBody *>::iterator it;
    AABoundingBox  *boundingBox = 0;
    glm::vec3 min, max;
    for(it = mRigidBodies.begin(); it != mRigidBodies.end(); it++) {
        boundingBox = 0;
        boundingBox = dynamic_cast<AABoundingBox *>((*it)->getBoundingBox());
        if(boundingBox != 0) {
            min = boundingBox->getMin();
            max = boundingBox->getMax();
            sortedList[0].insert(AABBComp(*it, min.x, max.x));
            sortedList[1].insert(AABBComp(*it, min.y, max.y));
            sortedList[2].insert(AABBComp(*it, min.z, max.z));
        }
    }

    std::cout << "Sorted list: " << std::endl;
    std::multiset<AABBComp, AABBComp>::iterator sss = sortedList[0].begin();
    for(; sss != sortedList[0].end(); sss++) {
        std::cout << sss->min << " " << sss->max << std::endl;
    }
    std::cout << "End sorted list\n\n";

    /**
     * Step 2:
     * Traverse each list
     * - each time a start point is reached insert into active list
     * - if endpoint is hit remove from active list
     * - if two or more object are active at the same time, they overlap in the specified dimension
     * - Potential colliding pairs must overlap in all 3 lists
     **/
    std::vector<AABBComp> activeList[3];
    //std::unordered_map<CollidingPair, AxisCollide> collidingPairs;
    std::multiset<AABBComp>::iterator lit;
    for(lit = sortedList[0].begin(); lit != sortedList[0].end(); lit++) {

        // Pedicate min
        MaxPredicate p(lit->min);
        std::vector<AABBComp>::iterator start = activeList[0].begin();
        std::vector<AABBComp>::iterator end = activeList[0].end();
        //std::vector<AABBComp>::iterator last = std::remove_if(start, end, p ) ;
        activeList[0].erase(std::remove_if(start, end, p ), activeList[0].end());
        activeList[0].push_back(*lit);
        //activeList[0].remove_if(p);

        std::vector<AABBComp>::iterator i;
        std::cout << "ActiveList: " << std::endl;
        for(i = activeList[0].begin(); i != activeList[0].end(); i++) {
            std::cout << i->min << " " << i->max << std::endl ;
        }
        std::cout << "End ActiveList\n\n";
        // If there is at least 2 elements in list, then there is collision on this axis
        if(activeList[0].size() > 1) {
            int i = 0;
            for(i = 0; i < activeList[0].size(); i++) {
                int j = 0;
                while (j<activeList[0].size()) {
                    if(j != i) {
                        std::cout << i << " " << j << std::endl;
                        CollidingPair currentPair = CollidingPair(activeList[0].at(i).rigidBody, activeList[0].at(j).rigidBody);
                        std::unordered_map<CollidingPair, AxisCollide>::iterator cit = collidingPairs.find(currentPair);
                        if(cit != collidingPairs.end()) {
                            AxisCollide a = cit->second;
                            a.setCollideOnAxis(0);
                            collidingPairs[cit->first] = a;
                        } else {
                            AxisCollide a;
                            a.setCollideOnAxis(0);
                            collidingPairs[currentPair] = a;
                        }
                    }
                    j++;
                }
            }
        }
    }

    std::unordered_map<CollidingPair, AxisCollide>::iterator cit = collidingPairs.begin();
    for(; cit != collidingPairs.end(); cit++) {
        std::cout << "Collide " << cit->second.axis[0] << " id1:  " << cit->first.rigidBody1->getId()
            << ", id2: " << cit->first.rigidBody2->getId() << std::endl;
    }
    std::cout << "\n\n\n";
}

void PhysicsWorld::renderAllRigidBodies(float timestep)
{
    std::unordered_map<CollidingPair, AxisCollide>::iterator cit = collidingPairs.begin();
    for(; cit != collidingPairs.end(); cit++) {
        cit->first.rigidBody1->setCollide(true);
        cit->first.rigidBody2->setCollide(true);
    }

    std::vector<RigidBody *>::iterator it;
    for(it = mRigidBodies.begin(); it != mRigidBodies.end(); it++) {
        (*it)->render(timestep);
    }

    cit = collidingPairs.begin();
    for(; cit != collidingPairs.end(); cit++) {
        cit->first.rigidBody1->setCollide(false);
        cit->first.rigidBody2->setCollide(false);
    }
}
