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
    mCenter = 0.5f * (min + max);
    mSize = max - min;
    mEntity = new ParallelogramEntity(mCenter, mSize);
    mEntity->generate();
}

AABoundingBox::~AABoundingBox()
{
}

bool AABoundingBox::render()
{

    dinf << "Rendering bounding box " << std::endl;

    // Save all the states, so that it can be restored later.
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    // Render in wireframe
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

    mEntity->render();

    glPopAttrib();
    return true;
}
