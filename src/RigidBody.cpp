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


ContactModel* RigidBody::distanceVerticesToVerticesMesh(RigidBody *otherRigidBody)
{
    MeshData *data = getTransformedMeshData();
    std::vector<glm::vec3>::iterator it = data->mVertices.begin();
    MeshData *otherMeshData = otherRigidBody->getTransformedMeshData();
    std::vector<glm::vec3>::iterator otherIt = otherMeshData->mVertices.begin();

    float distance = 0;
    float norm = 0;
    glm::vec3 pointPlane, pointObject;
    glm::vec3 projectionOnPlane;
    glm::vec3 minProjectionOnEdge;
    glm::vec3 edge1, edge2, pointEdge;
    float minProjectionOnEdgeDistance = 100000;
    // Edges
    glm::vec3 e1, e2;
    if(otherMeshData->mVertices.size() > 1) {
        e1 = otherMeshData->mVertices[0];
        e2 = otherMeshData->mVertices[1];
    }
    if(it != data->mVertices.end()) distance = mt::norm((*it++)-(*otherIt++));
    for(it=data->mVertices.begin() ; it != data->mVertices.end(); it++ ) {
        for(otherIt = otherMeshData->mVertices.begin(); otherIt != otherMeshData->mVertices.end(); otherIt++ ) {
            // Vertex-Vertex
            norm = mt::norm((*it)-(*otherIt));
            if(norm <= distance) {
                distance = norm;
                pointPlane = (*otherIt);
                pointObject = (*it);
            }


            // Vertex-Edge
            glm::vec3 edgeVector = glm::normalize(e2-e1);
            glm::vec3 projection = e1 + glm::dot((*it) - e1, edgeVector) * edgeVector;
            // Check if on edge segment
            if(glm::dot(e2-e1, projection-e1)  > 0 && glm::dot(e2-projection, e2-e1) > 0) {
                norm = mt::norm((*it)-projection);
                //std::cout << "project on edge " << e1.x << " " << e1.y << " " << e1.z << ", " << e2.x << " " << e2.y << " " << e2.z << std::endl;
                //std::cout << "projection distance" << norm << std::endl;
                if(norm < minProjectionOnEdgeDistance)
                {
                    //std::cout << "dot: " << glm::dot(e2-e1, projection-e1) << " " << glm::dot(e2-projection, e2-e1) << std::endl;
                    minProjectionOnEdgeDistance = norm;
                    minProjectionOnEdge = projection;
                    edge1 = e1;
                    edge2 = e2;
                    pointEdge= (*it);
                }
            }
            // next edge
            e1 = e2;
            e2 = (*otherIt);
        }
    }
    ContactModel *contactModel = new ContactModel();
    // V-E collision
    if(minProjectionOnEdgeDistance < distance) {
        dt::drawPoint(minProjectionOnEdge, 0.2, glm::vec3(1., 1., 0.));
        dt::drawPoint(pointEdge, 0.2, glm::vec3(1., 1., 0.));
        dt::drawLine(edge1, minProjectionOnEdge);
        dt::drawLine(edge2, minProjectionOnEdge);
        dt::drawLine(edge2, edge1,glm::vec3(0,0,1), 0.2f, 0.2f, glm::vec3(1.,0., 0.));
        dt::drawLine(pointEdge, minProjectionOnEdge);

        contactModel->contactPoint = 0.5f * (pointObject + minProjectionOnEdge);
        contactModel->normal = glm::normalize(pointObject-minProjectionOnEdge);
        contactModel->distance = minProjectionOnEdgeDistance;
        contactModel->type = ContactModel::Type::VE;
    } else {
        dt::drawPoint(pointPlane);
        glm::vec3 contactPoint = 0.5f*(pointPlane + pointObject);
        dt::drawPoint(contactPoint, 0.2, glm::vec3(1., 0., 0.));
        dt::drawPoint(pointObject);
        dt::drawLine(pointPlane, pointObject);
        contactModel->distance = distance;
        contactModel->normal = glm::normalize(pointPlane-pointObject);
        contactModel->contactPoint = contactPoint;
        contactModel->type = ContactModel::Type::VV;
    }

    return contactModel;
}
float RigidBody::distanceToPlane(RigidBody *planeRigidBody)
{
    MeshData *data = getTransformedMeshData();
    std::vector<glm::vec3>::iterator it = data->mVertices.begin();
    MeshData *planeMeshData = planeRigidBody->getTransformedMeshData();
    std::vector<glm::vec3>::iterator planeIt = planeMeshData->mVertices.begin();

    float distance = 0;
    float norm = 0;
    std::cout << "size normals: " << planeMeshData->mNormals.size() << std::endl;
    glm::vec3 planeNormal;
    if(planeMeshData->mNormals.size() > 0)
        planeNormal = planeMeshData->mNormals[0];
    else {
        dinf<< "RigidBody::distanceToPlane: No normal defined on plane, aborting!";
        return 0;
    }
    glm::vec3 pointPlane, pointObject;
    glm::vec3 projectionOnPlane;
    if(it != data->mVertices.end()) distance = mt::norm((*it++)-(*planeIt++));
    for(it=data->mVertices.begin() ; it != data->mVertices.end(); it++ ) {
        for(planeIt = planeMeshData->mVertices.begin(); planeIt != planeMeshData->mVertices.end(); planeIt++ ) {
            norm = mt::norm((*it)-(*planeIt));
            if(norm <= distance) {
                distance = norm;
                pointPlane = (*planeIt);
                pointObject = (*it);
            }
            float distanceToPlane = glm::dot((*it), planeNormal);
            projectionOnPlane = (*it)-distanceToPlane*planeNormal;
            if(distanceToPlane <= distance) {
                distance = distanceToPlane;
                pointPlane = projectionOnPlane;
                pointObject = (*it);
            }
        }
    }
    dt::drawPoint(pointPlane);
    dt::drawPoint(pointObject);
    dt::drawLine(pointPlane, pointObject);

    return distance;
}
