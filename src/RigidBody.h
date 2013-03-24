#ifndef __RigidBody__
#define __RigidBody__

#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "AssimpMeshEntity.h"
#include "BoundingVolume.h"
#include "AABoundingBox.h"
#include "PhysicsBody.h"

class Entity;

struct ContactModel
{
    enum Type {VV, VE, EE, VF};
    Type type;
    glm::vec3 contactPoint;
    glm::vec3 contactPoint1;
    glm::vec3 contactPoint2;
    glm::vec3 normal;
    glm::vec3 edge1[2];
    glm::vec3 edge2[2];
    glm::vec3 edge3[2];
    float distance;

    RigidBody *rigidBody1;
    RigidBody *rigidBody2;
    bool contactHappened;
};


/**
 * @brief Represents a rigid body
 * This class provides all the necessary functions to work with rigid bodies:
 * - Applying transformations
 * - Applying forces (velocity, angular momentum
 *
 *
 * Example:
 *
 * Entity *entity = new Entity(); // creates an entity (can be whatever derived type you want (AssimpMeshEntity, FlagEntity....)
 *
 * RigidBody *rigidBody = new RigidBody(entity, 10); // creates a rigid body with a mass of 10 units
 *
 * AABoundingBox *boundingBox = new AABoundingBox(rigidBody); // creates a bounding box from the rigid body vertex data. An entity must be attached to the rigid body first for this to work properly
 *
 * rigidBody->setBoundingBox(boundingBox);
 *
 * physicsWorld->addRigidBody(rigidBody); // Adds it to the simulation (PhysicsWorld class)
 ***/
class RigidBody : public PhysicsBody
{
    public:
        enum CollidingType {BROAD_PHASE, NARROW_PHASE, NONE};

    protected:
        glm::vec3 mPosition;
        glm::mat4 mRotation;
        glm::mat4 mInverseInertialTensor;
        glm::vec3 mLinearMomentum; // P(t) = v(t)*m
        glm::vec3 mAngularVelocity; // w(t) = {wx, wy, wz}. Magnitude encodes speed of the spin
        float mAngularVelocityNorm;
        float mScaleFactorX, mScaleFactorY, mScaleFactorZ;


    private:
        float mInvMass;
        CollidingType mCollide;
        bool mDebug;
        glm::mat4 mTransformation; // Final transformation


        //glm::vec3 mInverseInertialTensor; // I(t)
        //glm::vec3 mAngularMomentum; // L(t) = I(t)*w(t)

        Entity *mEntity; // attached graphic entity to which physics apply
        MeshData *mMeshData;

        void init();
        void update(float ellapsedTime);
        glm::mat4 rotationMatrix(glm::vec3 axis, float angle);

        void approximateIntertialTensor();

    public:
        RigidBody();
        RigidBody(Entity *mEntity, const float mass);
        ~RigidBody();

        virtual void render(float ellapsedTime);

        void setPosition(const glm::vec3& position);
        void setEntity(Entity *entity);
        void setAngularVelocity(const glm::vec3& direction, float magnitude);
        void setLinearMomentum(const glm::vec3 &linearMomentum);
        void setMass(float mass);
        void rotate(const glm::vec3& direction, float angle);
        void scale(float scaleFactor);
        void scale(float scaleFactorX, float scaleFactorY, float scaleFactorZ);
        void translate(const glm::vec3& translation);

        ContactModel* distanceMeshToMesh(RigidBody *planeRigidBody);

        void setCollide(CollidingType state);
        CollidingType getCollidingType() const;

        glm::mat4 getRotation() const
        {
            return mRotation;
        }
        glm::vec3 getPosition() const
        {
            return mPosition;
        }
        glm::vec3 getLinearVelocity() const
        {
            return mLinearMomentum;
        }
        glm::vec3 getAngularVelocity() const
        {
            return mAngularVelocityNorm * mAngularVelocity;
        }
        glm::mat4 getInverseInertialTensor() const
        {
            return mInverseInertialTensor;
        }
        float getInvMass() const
        {
            return mInvMass;
        }
        glm::vec3 getCenterOfMass() const
        {
            if(mMeshData != 0)
                return mMeshData->center;
            else
                return glm::vec3();
        }
        MeshData *getMeshData() const;
        MeshData* getTransformedMeshData() const;
        void updateFromImpulse(glm::vec3 J, glm::vec3 omega);
};

#endif


