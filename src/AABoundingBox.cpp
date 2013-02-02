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

#include "AABoundingBox.h"

AABoundingBox::AABoundingBox() : BoundingBox()
{
    init(glm::vec3(), glm::vec3());
}

AABoundingBox::AABoundingBox(const glm::vec3& min, const glm::vec3& max)
{
    init(min, max);
}

void AABoundingBox::init(const glm::vec3& min, const glm::vec3& max)
{
    update(glm::vec3(0,0,0), min, max);
}

AABoundingBox::~AABoundingBox()
{
}

void AABoundingBox::update(const glm::vec3 &center, const glm::vec3& min, const glm::vec3& max)
{
    mMin = min;
    mMax = max;
    mCenter = center + 0.5f * (mMin + mMax);
    mSize = mMax - mMin;
    mEntity = new ParallelogramEntity(mCenter, mSize);
    mEntity->generate();
}

bool AABoundingBox::render(bool collide)
{
    // Save all the states, so that it can be restored later.
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    // Render in wireframe
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

    if(collide) {
        glColor3f(1.f, 0.f, 0.f);
    } else {
        glColor3f(0.f, 0.f, 1.f);
    }
    mEntity->render();

    glPopAttrib();
    return true;
}
