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

#include "mt.h"
#include "BoundingSphere.h"
#include <vector>
#include "MeshData.h"
#include "RigidBody.h"
#include <GL/glut.h>

BoundingSphere::BoundingSphere(RigidBody *parent) : BoundingVolume(parent)
{
    init();
}
BoundingSphere::~BoundingSphere()
{
}

void BoundingSphere::init()
{
    computeFromMeshData();
}


/**
 * Virtual functions
 **/
bool BoundingSphere::computeFromMeshData()
{
    std::vector<glm::vec3>::const_iterator it;
    for(it = mMeshData->mVertices.begin(); it != mMeshData->mVertices.end(); it++)
    {
        mRadius = glm::max(mt::norm(glm::vec3(*it)), mRadius);
    }
    return true;
}

void BoundingSphere::update()
{
}

bool BoundingSphere::render(bool collide)
{
    // Save all the states, so that it can be restored later.
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    // Render in wireframe
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    glDisable(GL_LIGHTING);

    if(collide) {
        glColor3f(1.f, 0.f, 0.f);
    } else {
        glColor3f(0.f, 0.f, 1.f);
    }
    glm::vec3 pos = mParent->getPosition();
    glPushMatrix();
        glTranslatef(pos.x, pos.y, pos.z);
        glutWireSphere(mRadius, 10, 10);
    glPopMatrix();

    glPopAttrib();
    return true;
}


bool BoundingSphere::collideWith(const BoundingSphere &boundingSphere)
{
    return mt::norm(mParent->getPosition() - boundingSphere.getCenter()) < mRadius + boundingSphere.getRadius();
}

