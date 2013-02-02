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
#include <glm/ext.hpp>

int RigidBody::id_counter = 0;

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
    id = id_counter++;
    mRotation = glm::mat4(1.f);
    mMass = 1.;
    //mLinearMomentum = glm::vec3(0.01, 0.01, 0.);
    mPosition = glm::vec3(0.f,0.f,0.f);
    mLinearMomentum = glm::vec3(0.f);
    setAngularVelocity(glm::vec3(0.f, 0.f, 0.f), 0.f);
    mAngularVelocityNorm = 0;
    mLinearMomentum = glm::vec3(0.f, 0.f, 0.f);
    setAngularVelocity(glm::vec3(0.f, 0.f, 0.f), 0.f);

    setCollide(false);

    AABoundingBox *aabb = new AABoundingBox();
    mBoundingBox = new AABoundingBox();
}

void RigidBody::AABB()
{
    if(mMeshData != 0) {
    glm::vec3 min, max;
    std::vector<glm::vec3>::iterator it = mMeshData->mVertices.begin();
    for(; it != mMeshData->mVertices.end(); it++)
    {
        glm::vec4 transformed = mRotation * glm::vec4(*it, 1);
        min.x = glm::min(min.x, transformed.x);
        min.y = glm::min(min.y, transformed.y);
        min.z = glm::min(min.z, transformed.z);
        max.x = glm::max(max.x, transformed.x);
        max.y = glm::max(max.y, transformed.y);
        max.z = glm::max(max.z, transformed.z);
    }
    //dinf << "Min: " << min.x << " " << min.y << " " << min.z << std::endl;
    //dinf << "Max: " << max.x << " " << max.y << " " << max.z << std::endl;
    AABoundingBox *aabb = dynamic_cast<AABoundingBox *>(mBoundingBox);
    if(aabb != 0)
        aabb->update(mPosition, min, max);
    } else {
        derr << "Error setting up AABB: No mesh data! Please call setEntity() first!" << std::endl;
    }
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
    if(mAngularVelocityNorm != 0) {
        mRotation *= rotationMatrix(mAngularVelocity, glm::length2(mAngularVelocity));
    } else {
        mRotation = glm::mat4(1.0f);
    }
    mTransformation = glm::mat4(1.f);
    glm::mat4 translation = glm::translate(mPosition.x, mPosition.y, mPosition.z);

    // Transformation = rotation followed by translation
    mTransformation *= glm::translate(mPosition.x, mPosition.y, mPosition.z) * mRotation ;

    // XXX: don't recompute full bounding box everytime, use an approximation
    AABB();
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

    // Build GL Matrix from glm mat4
    GLfloat *mat;
    mat = glm::value_ptr(mTransformation);
    glPushMatrix();
        // Multiply current stack by matrix
        glMultMatrixf(mat);
        mEntity->render();
    glPopMatrix();

    mBoundingBox->render(mCollide);
}


void RigidBody::setAngularVelocity(const glm::vec3& direction, float magnitude)
{
    mAngularVelocity = glm::normalize(direction) * magnitude;
    mAngularVelocityNorm = magnitude;
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
    mAngularVelocityNorm = glm::length2(angularVelocity);
    mRotation *= rotationMatrix(angularVelocity, mAngularVelocityNorm);
}

void RigidBody::setEntity(Entity *entity) {
    mEntity = entity;
    AssimpMeshEntity *e = 0;
    e = dynamic_cast<AssimpMeshEntity *>(entity);
    if(e != 0)
        mMeshData = e->toMeshData("bear");
    // XXX: don't call aabb automatically like that
    AABB();
}

BoundingBox* RigidBody::getBoundingBox()
{
    return mBoundingBox;
}

void RigidBody::setPosition(const glm::vec3 & position)
{
    mPosition = position;
    // XXX: Find better way to update AABB
    AABB();
}
void RigidBody::setCollide(bool collide)
{
    mCollide = collide;
}
