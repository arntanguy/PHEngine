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
#include "mt.h"
#include "DrawingTools.h"

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

    mBoundingBox = 0;

    setCollide(false);
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
        //mRotation = glm::mat4(1.0f);
    }
    mTransformation = glm::mat4(1.f);

    // Transformation = rotation followed by translation
    mTransformation *= glm::translate(mPosition.x, mPosition.y, mPosition.z) * mRotation ;

    if(mBoundingBox != 0)
    {
        mBoundingBox->update();
    }
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

    if(mBoundingBox != 0) {
        mBoundingBox->render(mCollide);
    }
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

void RigidBody::rotate(const glm::vec3& direction, float angle)
{
    mRotation *= rotationMatrix(direction, angle);
}

void RigidBody::setEntity(Entity *entity) {
    mEntity = entity;
    AssimpMeshEntity *e = 0;
    e = dynamic_cast<AssimpMeshEntity *>(entity);
    if(e != 0)
        mMeshData = e->toMeshData("Monkey");
}

void RigidBody::setPosition(const glm::vec3 & position)
{
    mPosition = position;
}

void RigidBody::setCollide(bool collide)
{
    mCollide = collide;
}

MeshData* RigidBody::getMeshData() const
{
    return mMeshData;
}

MeshData* RigidBody::getTransformedMeshData() const
{
    MeshData *tMesh = new MeshData();
    for (int i = 0; i < mMeshData->mVertices.size(); i++) {
        glm::vec4 transformedV = mTransformation * glm::vec4(mMeshData->mVertices.at(i),1.f);
        tMesh->mVertices.push_back(glm::vec3(transformedV));
        if(i < mMeshData->mNormals.size()) {
            glm::vec4 transformedN = mTransformation * glm::vec4(mMeshData->mNormals.at(i),1.f);
            tMesh->mNormals.push_back(glm::vec3(transformedN));
        }
    }
    return tMesh;
}

void RigidBody::setBoundingBox(BoundingVolume *boundingBox)
{
    mBoundingBox = boundingBox;
}

BoundingVolume* RigidBody::getBoundingBox()
{
    return mBoundingBox;
}


float RigidBody::distanceToPlane(RigidBody *planeRigidBody)
{
    MeshData *data = getTransformedMeshData();
    std::vector<glm::vec3>::iterator it = data->mVertices.begin();
    MeshData *planeMeshData = planeRigidBody->getTransformedMeshData();
    std::vector<glm::vec3>::iterator planeIt = planeMeshData->mVertices.begin();

    float distance = 0;
    float norm = 0;
    glm::vec3 pointPlane, pointObject;
    if(it != data->mVertices.end()) distance = mt::norm((*it++)-(*planeIt++));
    for(it=data->mVertices.begin() ; it != data->mVertices.end(); it++ ) {
        for(planeIt = planeMeshData->mVertices.begin(); planeIt != planeMeshData->mVertices.end(); planeIt++ ) {
            norm = mt::norm((*it)-(*planeIt));
            if(norm <= distance) {
                distance = norm;
                pointPlane = (*planeIt);
                pointObject = (*it);
            }
        }
    }
    dt::drawPoint(pointPlane);
    dt::drawPoint(pointObject);
    dt::drawLine(pointPlane, pointObject);

    return distance;
}
