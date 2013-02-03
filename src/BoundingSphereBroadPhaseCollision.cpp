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

#include "BoundingSphereBroadPhaseCollision.h"
#include "PhysicsWorld.h"
#include "BoundingSphere.h"

BoundingSphereBroadPhaseCollision::BoundingSphereBroadPhaseCollision(PhysicsWorld *phWorld) : BroadPhaseCollision(phWorld)
{
}
BoundingSphereBroadPhaseCollision::~BoundingSphereBroadPhaseCollision()
{
}

void BoundingSphereBroadPhaseCollision::update()
{
    mCollidingPairs.clear();

    std::vector<RigidBody *> rigidBodies = mPhysicsWorld->getRigidBodies();

    std::vector<RigidBody *>::iterator it;
    BoundingSphere *boundingSphere1, *boundingSphere2 = 0;
    for (int i = 0; i < rigidBodies.size(); i++ ) {
        boundingSphere1 = 0;
        boundingSphere1 = dynamic_cast<BoundingSphere *>(rigidBodies[i]->getBoundingBox());
        if(boundingSphere1 != 0) {
            int j=0;
            while(j<rigidBodies.size()) {
                if( i != j) {
                    boundingSphere2 = 0;
                    boundingSphere2 = dynamic_cast<BoundingSphere *>(rigidBodies[j]->getBoundingBox());
                    if(boundingSphere2 != 0) {
                        //ding << "Checking " << i << " " << j << std::endl;
                        if(boundingSphere1->collideWith(*boundingSphere2)) {
                            AxisCollide a ;
                            a.setCollideOnAxis(0);
                            a.setCollideOnAxis(1);
                            a.setCollideOnAxis(2);
                            mCollidingPairs[RigidBodyPair(rigidBodies[i], rigidBodies[j])] = a;
                            //dinf << "Colliding " << i << " " << j << std::endl;
                        }
                    }
                }
                j++;
            }
        }
    }

}
