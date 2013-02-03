/******************************************************************************
*     Copyright (C) 2013 TANGUY Arnaud arn.tanguy@gmail.com                   *
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

#include "AABBBroadPhaseCollision.h"
#include "PhysicsWorld.h"
#include <unordered_map>
#include <set>
#include <vector>

AABBBroadPhaseCollision::AABBBroadPhaseCollision(PhysicsWorld *phWorld) : BroadPhaseCollision(phWorld)
{
}
AABBBroadPhaseCollision::~AABBBroadPhaseCollision()
{
}

void AABBBroadPhaseCollision::update()
{
    //dinf << "PhysicsWorld::AABBBroadPhase()" << std::endl;

    // Clear all colliding pairs
    mCollidingPairs.clear();
    std::vector<RigidBody *> rigidBodies = mPhysicsWorld->getRigidBodies();
    /**
     * Step 1: sort list for all 3 axis by minimum location of bounding box
     * XXX: take into account frame coherency to get a quicker sort
     **/
    std::multiset<AABBComp, AABBComp> sortedList[3];
    std::multiset<AABBComp>::iterator sortedListIt;
    std::vector<AABBComp> activeList[3];
    std::unordered_map<RigidBodyPair, AxisCollide> collidingPairs;
    AABoundingBox  *boundingBox = 0;
    glm::vec3 min, max;
    std::vector<RigidBody *>::iterator it;
    for(it = rigidBodies.begin(); it != rigidBodies.end(); it++) {
        boundingBox = 0;
        // Only handle AABB, do nothing with the rest
        boundingBox = dynamic_cast<AABoundingBox *>((*it)->getBoundingBox());
        if(boundingBox != 0) {
            min = boundingBox->getMin();
            max = boundingBox->getMax();
            sortedList[0].insert(AABBComp(*it, min.x, max.x));
            sortedList[1].insert(AABBComp(*it, min.y, max.y));
            sortedList[2].insert(AABBComp(*it, min.z, max.z));
        }
    }



    /**
     * Step 2:
     * Traverse each list
     * - each time a start point is reached insert into active list
     * - if endpoint is hit remove from active list
     * - if two or more object are active at the same time, they overlap in the specified dimension
     * - Potential colliding pairs must overlap in all 3 lists
     **/
    for(short axis=0; axis<3; axis++){
        //dinf << "Sorted list [Axis " << axis <<"]: " << std::endl;
        //std::multiset<AABBComp, AABBComp>::iterator sss = sortedList[axis].begin();
        //for(; sss != sortedList[axis].end(); sss++) {
            //std::cout << sss->min << " " << sss->max << std::endl;
        //}
        //dinf << "End sorted list\n\n";
        for(sortedListIt = sortedList[axis].begin(); sortedListIt != sortedList[axis].end(); sortedListIt++) {

            // Pedicate min
            RemoveFromActiveList p(sortedListIt->min);
            //std::vector<AABBComp>::iterator start = activeList[axis].begin();
            //std::vector<AABBComp>::iterator end = activeList[axis].end();
            //std::vector<AABBComp>::iterator last = std::remove_if(start, end, p ) ;
            activeList[axis].erase(std::remove_if(activeList[axis].begin(), activeList[axis].end(), p ), activeList[axis].end());
            activeList[axis].push_back(*sortedListIt);

            //dinf << "ActiveList: " << std::endl;
            //std::vector<AABBComp>::iterator i;
            //for(i = activeList[axis].begin(); i != activeList[axis].end(); i++) {
            //std::cout << i->min << " " << i->max << std::endl ;
            //}
            //dinf << "End ActiveList\n\n";
            // If there is at least 2 elements in list, then there is collision on this axis
            if(activeList[axis].size() > 1) {
                int i = 0;
                for(; i < activeList[axis].size(); i++) {
                    int j = 0;
                    while (j<activeList[axis].size()) {
                        if(j != i) {
                            RigidBodyPair currentPair = RigidBodyPair(activeList[axis].at(i).rigidBody, activeList[axis].at(j).rigidBody);
                            std::unordered_map<RigidBodyPair, AxisCollide>::iterator cit = collidingPairs.find(currentPair);
                            if(cit != collidingPairs.end()) {
                                AxisCollide a = cit->second;
                                a.setCollideOnAxis(axis);
                                collidingPairs[cit->first] = a;
                            } else {
                                AxisCollide a;
                                a.setCollideOnAxis(axis);
                                collidingPairs[currentPair] = a;
                            }
                        }
                        j++;
                    }
                }
            }
        }
    }

    /**
     * Store all colliding objects
     **/
    std::unordered_map<RigidBodyPair, AxisCollide>::iterator cit = collidingPairs.begin();
    for(; cit != collidingPairs.end(); cit++) {
        if(cit->second.collide()) {
            mCollidingPairs[cit->first] = cit->second;
        }
    }

    //std::unordered_map<RigidBodyPair, AxisCollide>::iterator cit = collidingPairs.begin();
    //for(; cit != collidingPairs.end(); cit++) {
        //dinf << "Collide on axis (" << cit->second.axis[0] << " " << cit->second.axis[1] << " " << cit->second.axis[2] << "), id1:  " << cit->first.rigidBody1->getId() << ", id2: " << cit->first.rigidBody2->getId() << std::endl;
    //}
    //dinf << "\n\n\n";
}


