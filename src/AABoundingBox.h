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

#ifndef __AABoundingBox__
#define __AABoundingBox__

#include "BoundingBox.h"
#include "ParallelogramEntity.h"
#include <glm/glm.hpp>

class AABoundingBox : public BoundingBox
{
    private:
        glm::vec3 mCenter;
        glm::vec3 mSize; // Size along x, y, and z axis
        glm::vec3 min, max;
        ParallelogramEntity *mEntity;

        void init(const glm::vec3& min, const glm::vec3& max);

    public:
        AABoundingBox();
        AABoundingBox(const glm::vec3& min, const glm::vec3& max);
        ~AABoundingBox();

        void update(const glm::vec3& center, const glm::vec3& min, const glm::vec3& max);

        bool render();
};

#endif
