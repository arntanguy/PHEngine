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

#ifndef __BroadPhaseCollision__
#define __BroadPhaseCollision__

#include <unordered_map>
#include <vector>
#include <list>
#include "PhysicsBody.h"

class PhysicsWorld;

/**
 * @brief Holds information used to compare two AABB
 * In effect, this will be the min and max value of the AABB along the considered axis
 */
struct AABBComp
{
    float min, max;
    PhysicsBody *rigidBody;

    AABBComp() { }
    AABBComp(PhysicsBody *rigidBody, float min, float max) {
        this->min = min;
        this->max = max;
        this->rigidBody = rigidBody;
    }
    /**
     * @brief Operator used by std containers (multiset) to sort the items
     *
     * @param v1 AABB comparison information of rigid body 1
     * @param v2 AABB comparison information of rigid body 2
     *
     * @return
     *  true when min(object1) < min(object2)
     */
    bool operator()(AABBComp v1, AABBComp v2)
    {
        return v1.min < v2.min;
    }
};

/**
 * @brief A predicate for removing items from the active list of AABB
 * Used to perform BroadPhase
 */
struct RemoveFromActiveList{
  float min;
  /**
   * @brief Predicate returning true for every item that has to be removed from active list.
   * This happens when the maximum value of the AABB along considered axis is smaller than the minimal value of the current AABB.
   * -----A----
   *      -----B-----
   *               -----Current-----
   * In this example case, A will be removed because A.max < Current.min
   *
   * @param min
   *    The minimal value of the current bounding box
   */
  RemoveFromActiveList(float min) { this->min = min; }
  bool operator() (const AABBComp& v) { return v.max < min; }
};

/**
 * @brief Holds information about which axis the two AABB are
 * colliding.
 */
struct AxisCollide
{
    // Number of axis on which the collision occurs
    bool axis[3];
    AxisCollide() {
        axis[0] = false;
        axis[1] = false;
        axis[2] = false;
    }
    void setCollideOnAxis(short a) {
        if(a < 3)
            axis[a] = true;
    }
    /**
     * @brief Checks whether the AABB of the two objects are colliding
     * This happens when the objects are colliding along all 3 axis at the same time
     *
     * @return
     *  true in case of collision
     */
    bool collide() {
        return axis[0] && axis[1] && axis[2];
    }
};

/**
 * @brief Represent a pair of rigid bodies.
 * Operator == is overridden so that the order of the bodies doesn't matter, meaning (A,B)=(B,A)
 */
struct RigidBodyPair
{
    PhysicsBody *rigidBody1;
    PhysicsBody *rigidBody2;

    RigidBodyPair(PhysicsBody *r1, PhysicsBody *r2) {
        rigidBody1 = r1;
        rigidBody2 = r2;
    }

    bool operator==(const RigidBodyPair &c) const
    {
        return ((rigidBody1 == c.rigidBody1) && (rigidBody2 == c.rigidBody2))  ||
            ((rigidBody2 == c.rigidBody1) && (rigidBody1 == c.rigidBody2)) ;
    }
};

namespace std {
    /**
     * @brief
     * Provide hash function for RigidBodyPair type.
     * See http://marknelson.us/2011/09/03/hash-functions-for-c-unordered-containers/
     * for details
     **/
    template <>
        class hash<RigidBodyPair>{
            public :
                size_t operator()(const RigidBodyPair &c ) const
                {
                    return std::hash<int>()(c.rigidBody1->getId()) + std::hash<int>()(c.rigidBody2->getId());
                }
        };
};

class BroadPhaseCollision
{
    protected:
        PhysicsWorld *mPhysicsWorld;
        std::unordered_map<RigidBodyPair, AxisCollide> mCollidingPairs;

    public:
        BroadPhaseCollision(PhysicsWorld *phWorld);
        virtual ~BroadPhaseCollision();

        virtual void update() = 0;

        std::unordered_map<RigidBodyPair, AxisCollide> getCollidingPairs() const;
};

#endif
