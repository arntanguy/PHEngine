#ifndef __BoundingSphere__
#define __BoundingSphere__

#include "BoundingVolume.h"
#include "RigidBody.h"

class BoundingSphere : public BoundingVolume
{
    private:
        float mRadius;

        void init();

    public:
        BoundingSphere (RigidBody *parent);
        virtual ~BoundingSphere ();

        virtual bool computeFromMeshData();
        virtual void update();
        virtual bool render(bool collide);

        glm::vec3 getCenter() const
        {
            return mParent->getPosition();
        }
        float getRadius() const
        {
            return mRadius;
        }

        bool collideWith(const BoundingSphere &boundingSphere);

};



#endif
