/******************************************************************************
 Copyright (C) 2013  TANGUY Arnaud arn.tanguy@gmail.com
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

#ifndef __AABoundingBox__
#define __AABoundingBox__

#include "BoundingVolume.h"
#include "ParallelogramEntity.h"
#include <glm/glm.hpp>

/**
 * @brief Provides an Axis Aligned Bounding Box
 * This type of bounding box is always oriented by 3 main axis (x,y,z).
 * It is thus rotation dependant. This implementation support 2 main modes:
 * - AABB_EXACT: At each update, the most precise bounding box is recalculated. Thus, there won't ever be space between the object and the bounding box. *   This is the optimal AABB
 *   WARNING: The algorithm calculating the best fitting bounding box has to go through all vertices, and this everytime a change of orientation is made to the object! This can be a huge computation if you have a lot of vertices.
 * - AABB_APPROXIMATE: Computes an AABB that will fit every possible orientation of the object.
 *   The update step is trivial, only the center position of the AABB is updated.
 */
class AABoundingBox: public BoundingVolume {
public:
	enum Type {
		AABB_EXACT, AABB_APPROXIMATE
	};

private:
	Type mType;
	glm::vec3 mCenter;
	glm::vec3 mSize; // Size along x, y, and z axis
	glm::vec3 mMin, mMax;
	ParallelogramEntity *mEntity;

	void init(Type type, const glm::vec3& mMin, const glm::vec3& mMax);

	void computeExactAABB();
	void computeApproximateAABB();

	void update(const glm::vec3&center);
	void update(const glm::vec3& center, const glm::vec3& mMin,
			const glm::vec3& mMax);

public:
	AABoundingBox();
	AABoundingBox(RigidBody *parent, Type type = Type::AABB_EXACT);
	AABoundingBox(RigidBody *parent, const glm::vec3& mMin,
			const glm::vec3& mMax, Type type);
	virtual ~AABoundingBox();

	virtual bool computeFromMeshData();
	virtual void update();
	virtual bool render(bool collide);

	// XXX: fix it
	void manualUpdate(glm::vec3 min, glm::vec3 max) {
		this->update(glm::vec3(0,0,0), min, max);
	}
	glm::vec3 getMin() const {
		return mCenter + mMin;
	}
	glm::vec3 getMax() const {
		return mCenter + mMax;
	}
};

#endif
