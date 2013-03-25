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

#ifndef __BoundingVolume__
#define __BoundingVolume__

#include "Entity.h"
#include <glm/glm.hpp>

class MeshData;
class PhysicsBody;

/**
 * @brief Abstract class representing a bounding box
 * Defines essential virtual functions to all bounding boxes:
 * - Generation
 * - update (use data from parent rigid body to determine update parameters)
 * - render (debug)
 */
class BoundingVolume
{
    protected:
        bool mCollide;

        PhysicsBody *mParent;
        MeshData *mMeshData;

    public:
        BoundingVolume();
        BoundingVolume(PhysicsBody *parent);
        virtual ~BoundingVolume();

        virtual bool computeFromMeshData() = 0;
        virtual void update() = 0;
        virtual bool render(bool collide) = 0;
};

#endif
