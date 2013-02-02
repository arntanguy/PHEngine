#ifndef __RigidBody__
#define __RigidBody__

#include <glm/glm.hpp>
#include <glm/ext.hpp>
#include "AssimpMeshEntity.h"
#include "BoundingBox.h"

class Entity;

class RigidBody {
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
        BoundingBox* mBoundingBox;
        MeshData *mMeshData;

        void init();
        void update(float ellapsedTime);
        glm::mat4 rotationMatrix(glm::vec3 axis, float angle);

        // Rotation matrix must be set correctly
        void AABB();

    public:
        RigidBody();
        RigidBody(Entity *mEntity, const float mass);
        ~RigidBody();

        virtual void render(float ellapsedTime);

        void setPosition(const glm::vec3& position);
        void setEntity(Entity *entity);
        void setAngularVelocity(const glm::vec3& direction, float magnitude);
        void setLinearMomentum(const glm::vec3 &linearMomentum);
        void rotate(const glm::vec3& mAngularVelocityNorm);
        void translate(const glm::vec3& translation);

        BoundingBox *getBoundingBox() ;
        void setCollide(bool state);
        int getId() const {
            return id;
        }
};

#endif
