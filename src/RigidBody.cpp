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
#include <limits.h> // For max size of numbers
#include "Triangle.h"
#include <omp.h>


int RigidBody::id_counter = 0;

RigidBody::RigidBody()
{
    init();
}

RigidBody::RigidBody(Entity *mEntity, const float mass)
{
    init();
    setMass(mass);
    setEntity(mEntity);
}

void RigidBody::init()
{
    id = id_counter++;
    mRotation = glm::mat4(1.f);
    mInverseInertialTensor = glm::mat4(1.f);
    setMass(0.0);
    //mLinearMomentum = glm::vec3(0.01, 0.01, 0.);
    mPosition = glm::vec3(0.f,0.f,0.f);
    mLinearMomentum = glm::vec3(0.f);
    setAngularVelocity(glm::vec3(0.f, 0.f, 0.f), 0.f);
    mAngularVelocityNorm = 0;
    mLinearMomentum = glm::vec3(0.f, 0.f, 0.f);
    setAngularVelocity(glm::vec3(0.f, 0.f, 0.f), 0.f);
    scale(1.f, 1.f, 1.f);

    mBoundingBox = 0;

    setCollide(CollidingType::NONE);
}

RigidBody::~RigidBody()
{
}

/**
 * Virutal functions
 **/
void RigidBody::update(float ellapsedTime)
{
    if(mInvMass > 0)
        mPosition = mPosition + mLinearMomentum / mInvMass * ellapsedTime;
    if(mAngularVelocityNorm != 0) {
        mRotation *= rotationMatrix(mAngularVelocity, glm::length2(mAngularVelocity));
    } else {
        //mRotation = glm::mat4(1.0f);
    }
    mTransformation = glm::mat4(1.f);

    // Transformation = rotation followed by translation
    mTransformation *= glm::translate(mPosition.x, mPosition.y, mPosition.z) * mRotation ;

    //if(mInvMass != 0) {
    //mLinearMomentum.y -= ellapsedTime * .01f/mInvMass;
    //}

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
 *  Time ellapsed since last simulation step
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
    glPushMatrix();
    glScalef(mScaleFactorX, mScaleFactorY, mScaleFactorZ);
    mEntity->render();
    glPopMatrix();
    glPopMatrix();

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
    if(e != 0) {
        mMeshData = e->toMeshData();
        approximateIntertialTensor();
    }
}

/**
 * @brief If MeshData is correctly set, approximate intertial tensor,
 * assuming a uniformally distributed mass.
 */
void RigidBody::approximateIntertialTensor()
{
    if(mMeshData != 0) {
        if(mInvMass == 0) {
            mInverseInertialTensor = glm::mat4(0.f);
        } else {
            float mi = (1.f/mInvMass) / mMeshData->mVertices.size();
            glm::vec3 ri;
            std::vector<glm::vec3>::iterator it;
            glm::mat4 inertialTensor;
            for( it = mMeshData->mVertices.begin() ; it != mMeshData->mVertices.end(); it++ ) {
                ri = *it;

                // row 1
                inertialTensor[0][0] +=  mi * (ri.y*ri.y + ri.z*ri.z);
                inertialTensor[0][1] += -mi * ri.x * ri.y;
                inertialTensor[0][2] += -mi * ri.x * ri.z;

                // row 2
                inertialTensor[1][0] += -mi * ri.y * ri.z;
                inertialTensor[1][1] +=  mi * (ri.x*ri.x + ri.z*ri.z);
                inertialTensor[1][2] += -mi * ri.y * ri.z;

                // row 3
                inertialTensor[2][0] += -mi * ri.z * ri.x;
                inertialTensor[2][1] += -mi * ri.z * ri.y;
                inertialTensor[2][1] +=  mi * (ri.x*ri.x + ri.y*ri.y);

            }
            mInverseInertialTensor = glm::inverse(inertialTensor);
        }
    }
}


void RigidBody::setPosition(const glm::vec3 & position)
{
    mPosition = position;
}

void RigidBody::setCollide(CollidingType collide)
{
    mCollide = collide;
}

RigidBody::CollidingType RigidBody::getCollidingType() const
{
    return mCollide;
}

MeshData* RigidBody::getMeshData() const
{
    return mMeshData;
}

MeshData* RigidBody::getTransformedMeshData() const
{
    MeshData *tMesh = new MeshData();
    for (int i = 0; i < mMeshData->mVertices.size(); i++) {
        glm::vec4 transformedV = mTransformation * glm::scale(mScaleFactorX, mScaleFactorY, mScaleFactorZ) * glm::vec4(mMeshData->mVertices.at(i),1.f);
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


/**
 * @brief Compute a contact model from the minimal distance from mesh to mesh
 *
 * @param otherRigidBody
 *  The other rigig body to check contact against
 *
 * @return
 *  A contact model for the area of potential collision
 */
ContactModel* RigidBody::distanceMeshToMesh(RigidBody *otherRigidBody)
{
    glDisable(GL_LIGHTING);
    MeshData *data = getTransformedMeshData();
    std::vector<glm::vec3>::iterator it = data->mVertices.begin();
    MeshData *otherMeshData = otherRigidBody->getTransformedMeshData();


    // For minimum
    float normVV = FLT_MAX;
    float normEE = FLT_MAX;
    float normVE = FLT_MAX;
    float normVF = FLT_MAX;

    float norm = 0;
    glm::vec3 pointPlane, pointObject;
    glm::vec3 projectionOnPlane;
    glm::vec3 minProjectionOnEdge;
    glm::vec3 edge1, edge2, pointEdge;
    // Edges
    glm::vec3 e1, e2; // Of other object
    glm::vec3 ev; // normalized edge vector of object
    glm::vec3 eo1, eo2; // Of object
    glm::vec3 eov; // normalized edge vector of other object
    glm::vec3 minPE1, minPE2;
    glm::vec3 EEedge11, EEedge12, EEedge21, EEedge22;

    //V-F
    glm::vec3 minPF1, minPF2;
    glm::vec3 vfEdge1[2], vfEdge2[2], vfEdge3[2];

    // Init first edges
    if(otherMeshData->mVertices.size() > 1) {
        e1 = otherMeshData->mVertices[0];
        e2 = otherMeshData->mVertices[1];
        ev = glm::normalize(e2 - e1);
    }
    if(data->mVertices.size() > 1) {
        eo1 = data->mVertices[0];
        eo2 = data->mVertices[1];
        eov = glm::normalize(eo2 - eo1);
    }

    int triangle = 0;
    for(it=data->mVertices.begin() ; it != data->mVertices.end(); it++ ) {
        triangle = 0;
        std::vector<glm::vec3>::iterator otherIt = otherMeshData->mVertices.begin();
        for(otherIt = otherMeshData->mVertices.begin(); otherIt != otherMeshData->mVertices.end(); otherIt++ ) {
            // Vertex-Vertex
            // Straight computation of norm between vertices
            norm = mt::norm((*it)-(*otherIt));
            if(norm <= normVV) {
                normVV = norm;
                pointPlane = (*otherIt);
                pointObject = (*it);
            }


            // Vertex-Edge
            // Project vertex onto edge
            ev = glm::normalize(e2-e1);
            glm::vec3 projection;
            // Project and check if edge is on segment
            if(mt::projectPointOnEdge(e1, e2, (*it), projection)) {
                norm = mt::norm((*it)-projection);
                if(norm <= normVE)
                {
                    normVE = norm;
                    minProjectionOnEdge = projection;
                    pointEdge= (*it);

                    // Just for debug
                    edge1 = e1;
                    edge2 = e2;
                }
            }


            // Edge-Edge
            //- Minimal projection of edge onto another edge (pE1)
            //- Then project this point onto other edge (pE2)
            float k = glm::dot(ev, eov);
            glm::vec3 pE1 = e1 + (glm::dot(eo1-e1, ev-k*eov) * ev);
            if(mt::onEdge(pE1, e1, e2)) {

                glm::vec3 pE2;
                if(mt::projectPointOnEdge(eo1, eo2, pE1, pE2)) {
                    norm = mt::norm(pE2-pE1);
                    if(norm < normEE) {
                        normEE = norm;
                        minPE1 = pE1;
                        minPE2 = pE2;

                        // Just for debug
                        EEedge11 = e1;
                        EEedge12 = e2;
                        EEedge21 = eo1;
                        EEedge22 = eo2;
                    }
                }
            }

            //Vertex-Face
            // 3 vertices of the triangle
            glm::vec3 f1, f2, f3;
            glm::vec3 pF;
            // XXX: don't compute normal if correctly specified
            glm::vec3 n;
            if(3*triangle+2 <= otherMeshData->mVertices.size()) {
                f1 = otherMeshData->mVertices[3*triangle];
                f2 = otherMeshData->mVertices[3*triangle+1];
                f3 = otherMeshData->mVertices[3*triangle+2];
                // XXX: use model normals instead
                n = glm::normalize(glm::cross(f2-f1, f3-f1));
                // Show normal
                //dt::drawLine(f1, f1+n);

                if(mt::projectOnTriangle(f1, f2, f3, n, (*it), pF)) {
                    norm = mt::norm(pF-(*it));
                    if(norm < normVF) {
                        normVF = norm;
                        minPF1 = (*it);
                        minPF2 = pF;

                        vfEdge1[0] = f1; vfEdge1[1] = f2;
                        vfEdge2[0] = f2; vfEdge2[1] = f3;
                        vfEdge3[0] = f3; vfEdge3[1] = f1;
                    }
                }
                triangle++;
            }
            // next edge of other object
            e1 = e2;
            e2 = (*otherIt);
            ev = glm::normalize(e2-e1);
        }
        // Next edge of object
        eo1 = eo2;
        eo2 = (*it);
        eov = glm::normalize(eo2 - eo1);
    }
    ContactModel *contactModel = new ContactModel();
    contactModel->contactHappened = false;
    contactModel->rigidBody1 = this;
    contactModel->rigidBody2 = otherRigidBody;

    //std::cout << "dist V-V: " << normVV << std::endl;
    //std::cout << "dist V-E: " << normVE << std::endl;
    //std::cout << "dist E-E: " << normEE << std::endl;
    //std::cout << "dist V-F: " << normVF << std::endl;

    // V-V
    if( (normVV <= normVE) && (normVV <= normEE) && (normVV <= normVF)) {
        //std::cout << "VV" <<std::endl;
        dt::drawPoint(pointPlane);
        glm::vec3 contactPoint = 0.5f*(pointPlane + pointObject);
        dt::drawPoint(contactPoint, 0.2, glm::vec3(1., 0., 0.));
        dt::drawPoint(pointObject);
        dt::drawLine(pointPlane, pointObject);
        contactModel->distance = normVV;
        contactModel->normal = glm::normalize(pointPlane-pointObject);
        contactModel->contactPoint = contactPoint;
        contactModel->type = ContactModel::Type::VV;
    }
    // V-E collision
    if((normVE <= normVV) && (normVE <= normEE) && (normVE <= normVF)) {
        //std::cout << "VE" <<std::endl;
        dt::drawPoint(minProjectionOnEdge, 0.2, glm::vec3(1., 1., 0.));
        dt::drawPoint(pointEdge);
        dt::drawLine(edge1, minProjectionOnEdge);
        dt::drawLine(edge2, minProjectionOnEdge);
        dt::drawLine(edge2, edge1,glm::vec3(0,0,1), 0.2f, 0.2f, glm::vec3(1.,0., 0.));
        dt::drawLine(pointEdge, minProjectionOnEdge);

        contactModel->contactPoint = 0.5f * (pointObject + minProjectionOnEdge);
        contactModel->normal = glm::normalize(pointObject-minProjectionOnEdge);
        contactModel->distance = normVE;
        contactModel->type = ContactModel::Type::VE;
    }
    // E-E
    if((normEE <= normVV) & (normEE <= normVE) && (normEE <= normVF)) {
        //std::cout << "EE" <<std::endl;
        dt::drawPoint(minPE1, 0.2, glm::vec3(1.,1., 0.));
        dt::drawPoint(minPE2, 0.2, glm::vec3(1.,1., 0.));
        dt::drawLine(minPE1, minPE2);
        dt::drawLine(EEedge11, EEedge12,glm::vec3(0,0,1), 0.2f, 0.2f, glm::vec3(1.,0., 0.));
        dt::drawLine(EEedge21, EEedge22,glm::vec3(0,0,1), 0.2f, 0.2f, glm::vec3(1.,0., 0.));


        contactModel->type = ContactModel::Type::EE;
        contactModel->contactPoint = 0.5f * (minPE1+minPE2);
        contactModel->edge1[0] = EEedge11;
        contactModel->edge1[1] = EEedge12;
        contactModel->edge2[0] = EEedge21;
        contactModel->edge2[1] = EEedge22;
        contactModel->distance = normEE;
        // XXX: Check if this is the right normal
        contactModel->normal = glm::normalize(pointEdge-minPE1);
        dt::drawPoint(contactModel->contactPoint);
    }
    // V-F
    if((normVF <= normEE) && (normVF <= normVE) && (normVF <= normVV)) {
        //std::cout << "VF" <<std::endl;
        dt::drawPoint(minPF1);
        dt::drawPoint(minPF2, 0.2, glm::vec3(0.,1.,0.));
        dt::drawLine(minPF1, minPF2);
        dt::drawLine(vfEdge1[0], vfEdge1[1],glm::vec3(0,0,1), 0.2f, 0.2f, glm::vec3(1.,0., 0.));
        dt::drawLine(vfEdge2[0], vfEdge2[1],glm::vec3(0,0,1), 0.2f, 0.2f, glm::vec3(1.,0., 0.));
        dt::drawLine(vfEdge3[0], vfEdge3[1],glm::vec3(0,0,1), 0.2f, 0.2f, glm::vec3(1.,0., 0.));

        contactModel->type = ContactModel::Type::VF;
        contactModel->edge1[0] = vfEdge1[0];
        contactModel->edge1[1] = vfEdge1[1];
        contactModel->edge2[0] = vfEdge2[0];
        contactModel->edge2[1] = vfEdge2[1];
        contactModel->edge3[0] = vfEdge3[0];
        contactModel->edge3[1] = vfEdge3[1];
        contactModel->contactPoint1 = minPF1;
        contactModel->contactPoint2 = minPF2;
        contactModel->contactPoint = 0.5f*(minPF1+minPF2);
        contactModel->distance = normVF;
        contactModel->normal = glm::normalize(minPF2-minPF1);
    }

    return contactModel;
    glEnable(GL_LIGHTING);
    }

    void RigidBody::scale(float scaleFactor)
    {
        scale(scaleFactor, scaleFactor, scaleFactor);
    }

    void RigidBody::scale(float scaleFactorX, float scaleFactorY, float scaleFactorZ)
    {
        mScaleFactorX = scaleFactorX;
        mScaleFactorY = scaleFactorY;
        mScaleFactorZ = scaleFactorZ;
    }

    void RigidBody::setMass(float mass)
    {
        if( mass != 0 )
            mInvMass = 1.f/mass;
        else
            mInvMass = 0.f;
    }

    void RigidBody::updateFromImpulse(glm::vec3 J, glm::vec3 omega)
    {
        //static bool truc = true;
        //if(truc) {
        //mLinearMomentum.y = -mLinearMomentum.y;
        //truc = false;
        //}
        mLinearMomentum += J * mInvMass;
        std::cout << "mLinearMomentum:" << " ( " << mLinearMomentum.x << ", " << mLinearMomentum.y << ", " << mLinearMomentum.z << " )" << std::endl;

        glm::vec3 angularVelocity = mAngularVelocityNorm * mAngularVelocity;
        angularVelocity += omega;
        mAngularVelocityNorm = mt::norm(angularVelocity);
        mAngularVelocity = glm::normalize(angularVelocity);
    }

