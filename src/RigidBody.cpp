#include "RigidBody.h"
#include "Entity.h"
#include <GL/gl.h>

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
