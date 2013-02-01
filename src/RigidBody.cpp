/******************************************************************************
     Copyright (C) 2012  TANGUY Arnaud arn.tanguy@gmail.com
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

#include "RigidBody.h"
#include "Entity.h"
#include <GL/gl.h>
#include "Debug.h"
#include "AABoundingBox.h"

RigidBody::RigidBody()
{
    init();
}

RigidBody::RigidBody(Entity *mEntity, const float mass) : mMass(mass)
{
    init();
    setEntity(mEntity);
}

void RigidBody::init()
{
    mRotation = glm::mat4(1.f);
    mMass = 1.;
    //mLinearMomentum = glm::vec3(0.01, 0.01, 0.);
    mPosition = glm::vec3(0.f,0.f,0.f);
    mLinearMomentum = glm::vec3(0.f);
    setAngularVelocity(glm::vec3(0.f, 0.f, 0.f), 0.f);
    mLinearMomentum = glm::vec3(0.f, 0.f, 0.f);
    setAngularVelocity(glm::vec3(0.f, 0.f, 0.f), 0.f);
}

void RigidBody::AABB(const std::string& nodeName)
{
    AssimpMeshEntity *e = 0;
    e = dynamic_cast<AssimpMeshEntity*>(mEntity);
    if(e != 0) {
        dinf << "Entity type is supported by the physics engine." << std::endl;
        AABB(e, nodeName);
    } else {
        derr << "Entity type is unsupported by the physics engine." << std::endl;
    }
}

void RigidBody::AABB(AssimpMeshEntity *entity, const std::string &nodeName)
{
    MeshData *data = entity->toMeshData(nodeName);
    dinf << "Printing data..." << std::endl;
    glm::vec3 min, max;
    std::vector<glm::vec3>::iterator it = data->mVertices.begin();
    for(; it != data->mVertices.end(); it++)
    {
        min.x = glm::min(min.x, it->x);
        min.y = glm::min(min.y, it->y);
        min.z = glm::min(min.z, it->z);
        max.x = glm::max(max.x, it->x);
        max.y = glm::max(max.y, it->y);
        max.z = glm::max(max.z, it->z);
        //dinf << "V: " << it->x << " " << it->y << " " << it->z << std::endl;
    }
    dinf << "Min: " << min.x << " " << min.y << " " << min.z << std::endl;
    dinf << "Max: " << max.x << " " << max.y << " " << max.z << std::endl;
    AABoundingBox *aabb = new AABoundingBox(min, max);
    mBoundingBox = aabb;
}

RigidBody::~RigidBody()
{
}

/**
 * Virutal functions
 **/
void RigidBody::update(float ellapsedTime)
{
    mPosition = mPosition + mLinearMomentum/mMass * ellapsedTime;
}

glm::mat4 RigidBody::rotationMatrix(glm::vec3 axis, float angle)
{
    glm::mat4 rotationMatrix(1.f);
    rotationMatrix *= glm::rotate(rotationMatrix, angle, axis);
    return rotationMatrix;
}

/**
 * @brief Renders next step of physics simulation
 *
 * @param ellapsedTime
 */
void RigidBody::render(float ellapsedTime)
{
    update(ellapsedTime);

    glm::mat4 transformation;
    if(mAngularVelocity  != glm::vec3(0., 0., 0.) ) {
        mRotation *= rotationMatrix(mAngularVelocity, glm::length2(mAngularVelocity));
    } else {
        mRotation = glm::mat4(1.0f);
    }
    glm::mat4 translation = glm::translate(mPosition.x, mPosition.y, mPosition.z);

    transformation *= translation * mRotation ;

    // Build GL Matrix from glm mat4
    GLfloat *mat;
    mat = glm::value_ptr(transformation);
    // Multiply current stack by matrix
    glMultMatrixf(mat);

    mEntity->render();
    mBoundingBox->render();
}


void RigidBody::setAngularVelocity(const glm::vec3& direction, float magnitude)
{
    mAngularVelocity = glm::normalize(direction) * magnitude;
}

void RigidBody::setLinearMomentum(const glm::vec3 &linearMomentum)
{
    mLinearMomentum = linearMomentum;
}

void RigidBody::translate(const glm::vec3& translation)
{
    mPosition += translation;
}

void RigidBody::rotate(const glm::vec3& angularVelocity)
{
    mRotation *= rotationMatrix(angularVelocity, glm::length2(angularVelocity));
}

void RigidBody::setEntity(Entity *entity) {
    mEntity = entity;
    // XXX: don't do it automatically
    AABB("bear");
}
