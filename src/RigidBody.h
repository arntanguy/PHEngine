#ifndef __RigidBody__
#define __RigidBody__

#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "AssimpMeshEntity.h"
#include "BoundingVolume.h"
#include "AABoundingBox.h"

class Entity;

/**
 * @brief Represents a rigid body
 * This class provides all the necessary functions to work with rigid bodies:
 * - Applying transformations
 * - Applying forces (velocity, angular momentum
 */
class RigidBody
{
    private:
        static int id_counter;
        int id;
        float mMass;
        glm::vec3 mPosition;
        glm::mat4 mRotation;
        glm::vec3 mLinearMomentum; // P(t) = v(t)*m
        glm::vec3 mAngularVelocity; // w(t) = {wx, wy, wz}. Magnitude encodes speed of the spin
        float mAngularVelocityNorm;

        bool mCollide;

        glm::mat4 mTransformation; // Final transformation


        //glm::vec3 mIntertialTensor; // I(t)
        //glm::vec3 mAngularMomentum; // L(t) = I(t)*w(t)

        Entity *mEntity; // attached graphic entity to which physics apply
        BoundingVolume* mBoundingBox;
        MeshData *mMeshData;

        void init();
        void update(float ellapsedTime);
        glm::mat4 rotationMatrix(glm::vec3 axis, float angle);

    public:
        RigidBody();
        RigidBody(Entity *mEntity, const float mass);
        ~RigidBody();

        virtual void render(float ellapsedTime);

        void setBoundingBox(BoundingVolume *boundingBox);

        void setPosition(const glm::vec3& position);
        void setEntity(Entity *entity);
        void setAngularVelocity(const glm::vec3& direction, float magnitude);
        void setLinearMomentum(const glm::vec3 &linearMomentum);
        void rotate(const glm::vec3& direction, float angle);
        void translate(const glm::vec3& translation);

        BoundingVolume *getBoundingBox() ;
        void setCollide(bool state);
        int getId() const {
            return id;
        }

        glm::mat4 getRotation() const
        {
            return mRotation;
        }
        glm::vec3 getPosition() const
        {
            return mPosition;
        }
        MeshData *getMeshData() const;
};

#endif


/********************************************
 * Example:
 * Entity *entity = new Entity(); // creates an entity (can be whatever derived type you want (AssimpMeshEntity, FlagEntity....)
 * RigidBody *rigidBody = new RigidBody(entity, 10); // creates a rigid body with a mass of 10 units
 * AABoundingBox *boundingBox = new AABoundingBox(rigidBody); // creates a bounding box from the rigid body vertex data. An entity must be attached to the rigid body first for this to work properly
 * rigidBody->setBoundingBox(boundingBox);
 * physicsWorld->addRigidBody(rigidBody); // Adds it to the simulation (PhysicsWorld class)
 ***/
