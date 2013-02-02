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
#include <vector>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <glm/glm.hpp>
#include <iostream>
#include "RigidBody.h"


struct AABBComp
{
    float min, max;
    RigidBody *rigidBody;
    AABBComp() {
    }
    AABBComp(RigidBody *rigidBody, float min, float max) {
        this->min = min;
        this->max = max;
        this->rigidBody = rigidBody;
    }
    bool operator()(AABBComp v1, AABBComp v2)
    {
        return v1.min < v2.min;
    }
};

// a predicate implemented as a class:
struct MaxPredicate{
  float min;
  MaxPredicate(float min) { this->min = min; }
  bool operator() (const AABBComp& v) { return v.max < min; }
  //bool operator() (const AABBComp& v) { return true; }
};

struct AxisCollide
{
    // Number of axis on which the collision occurs
    bool axis[3];
    void setCollideOnAxis(short a) {
        if(a < 3)
            axis[a] = true;
    }
    // When collision is on all axis simultaneously, then the two object are in collision
    // (
    bool collide() {
        return axis[0] && axis[1] && axis[2];
    }
};

struct CollidingPair
{
    RigidBody *rigidBody1;
    RigidBody *rigidBody2;
    CollidingPair(RigidBody *r1, RigidBody *r2) {
        rigidBody1 = r1;
        rigidBody2 = r2;
    }

    bool operator==(const CollidingPair &c) const
    {
       // std::cout << "call == operator" << std::endl;
       // std::cout << "Checking " << rigidBody1->getId() <<"=="<<c.rigidBody1->getId()
       //     << " && " << rigidBody2->getId() << " == " << c.rigidBody2->getId() << std::endl;
        return ((rigidBody1 == c.rigidBody1) && (rigidBody2 == c.rigidBody2))  ||
            ((rigidBody2 == c.rigidBody1) && (rigidBody1 == c.rigidBody2)) ;
    }
};

/**
 * Provide hash function for CollidingPair type.
 * See http://marknelson.us/2011/09/03/hash-functions-for-c-unordered-containers/
 * for details
 **/
namespace std {
    template <>
        class hash<CollidingPair>{
        public :
            size_t operator()(const CollidingPair &c ) const
            {
                return std::hash<int>()(c.rigidBody1->getId()) + std::hash<int>()(c.rigidBody2->getId());
            }
    };
};

class PhysicsWorld
{
    private:
        // Active rigid bodies
        std::vector<RigidBody *> mRigidBodies;
        std::unordered_map<CollidingPair, AxisCollide> collidingPairs;
        //BroadPhaseCollision *mBroadphase;

    public:
        PhysicsWorld();
        ~PhysicsWorld();

        void addRigidBody(RigidBody *rigidBody);

        void AABBBroadPhase();

        void renderAllRigidBodies(float timestep);
};


#endif
