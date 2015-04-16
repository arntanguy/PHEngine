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

#include "RigidBody.h"
#include "PhysicsBody.h"
#include "AABoundingBox.h"
#include "CGEngine/MeshData.h"
#include <vector>
#include "mt.h"
#include <iostream>

using namespace std;

AABoundingBox::AABoundingBox() :
		BoundingVolume() {
	mParent = 0;
}

AABoundingBox::AABoundingBox(PhysicsBody *parent, Type type) :
		BoundingVolume(parent) {
	init(type, glm::vec3(), glm::vec3());
}

AABoundingBox::AABoundingBox(PhysicsBody *parent, const glm::vec3& min,
		const glm::vec3& max, Type type = AABB_EXACT) :
		BoundingVolume(parent) {
	init(type, min, max);
}

void AABoundingBox::init(Type type, const glm::vec3& min,
		const glm::vec3& max) {
	mType = type;
	//update(glm::vec3(0,0,0), min, max);
	computeFromMeshData();
}

AABoundingBox::~AABoundingBox() {
}

/**
 * @brief Computes the exact AABB by going through all vertices and looking for min and max in all directions
 */
void AABoundingBox::computeExactAABB() {
	if (mParent != 0 && mMeshData != 0) {
		std::vector<glm::vec3>::const_iterator it =
				mMeshData->mVertices.begin();
		RigidBody *rb = 0;
		rb = dynamic_cast<RigidBody *>(mParent);
		if (rb != 0) {
			mMin = glm::vec3(0);
			mMax = glm::vec3(0);
			for (; it != mMeshData->mVertices.end(); it++) {
				glm::vec4 transformed = rb->getRotation()
						* glm::vec4(*it, 1);
				mMin.x = glm::min(mMin.x, transformed.x);
				mMin.y = glm::min(mMin.y, transformed.y);
				mMin.z = glm::min(mMin.z, transformed.z);
				mMax.x = glm::max(mMax.x, transformed.x);
				mMax.y = glm::max(mMax.y, transformed.y);
				mMax.z = glm::max(mMax.z, transformed.z);
			}
			//dinf << "Min: " << min.x << " " << min.y << " " << min.z << std::endl;
			//dinf << "Max: " << max.x << " " << max.y << " " << max.z << std::endl;
			this->update(mParent->getPosition(), mMin, mMax);
		}
	}

}

/**
 * @brief Computes an approximate AABB fitting all possible orientations of the object
 */
void AABoundingBox::computeApproximateAABB() {
	if (mMeshData != 0) {
		float max = 0;
		std::vector<glm::vec3>::const_iterator it;
		for (it = mMeshData->mVertices.begin();
				it != mMeshData->mVertices.end(); it++) {
			max = glm::max(mt::norm(glm::vec3(*it)), max);
		}
		mMin = glm::vec3(-max, -max, -max);
		mMax = glm::vec3(max, max, max);

		this->update(mParent->getPosition(), mMin, mMax);
	}
}

/**
 * @brief Uses the mesh data to compute the AABB
 * The type of AABB used is defined by the member mType
 *
 * @return
 *  True on success
 */
bool AABoundingBox::computeFromMeshData() {
	if (mMeshData != 0) {
		if (mType == Type::AABB_EXACT) {
			computeExactAABB();
		} else {
			computeApproximateAABB();
		}
		return true;
	}
	return false;
}

/**
 * @brief Update the bounding box w.r.t the new parent rigid body location and orientation
 */
void AABoundingBox::update() {
	if (mParent != 0 && mMeshData != 0) {
		if (mType == AABB_EXACT) {
			computeExactAABB();
		} else {
			if (mParent != 0) {
				update(mParent->getPosition());
			}
		}
	}
}

void AABoundingBox::update(const glm::vec3 &center, const glm::vec3& min,
		const glm::vec3& max) {
	mMin = min;
	mMax = max;
	mSize = mMax - mMin;
	update(center);
	mEntity = new ParallelogramEntity(mCenter, mSize);
	mEntity->generate();
}

void AABoundingBox::update(const glm::vec3 &center) {
	mCenter = center + 0.5f * (mMin + mMax);
	mEntity = new ParallelogramEntity(mCenter, mSize);
	mEntity->generate();
}

/**
 * @brief This is a debug function. It will display a wireframe of the bounding box.
 *
 * @param collide
 *      Sets whether the object is colliding.
 *      This is used to display in a different color in case of collision.
 *
 * @return
 */
bool AABoundingBox::render(bool collide) {
	// Save all the states, so that it can be restored later.
	glPushAttrib (GL_ALL_ATTRIB_BITS);

	// Render in wireframe
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	if (collide) {
		glColor3f(1.f, 0.f, 0.f);
	} else {
		glColor3f(0.f, 0.f, 1.f);
	}
	mEntity->render();

	glPopAttrib();
	return true;
}

/**
 * Min, max: absolute values relative to the world
 * XXX: change that?
 * @param min
 * @param max
 */
void AABoundingBox::manualUpdate(glm::vec3 min, glm::vec3 max) {
	glm::vec3 center = 0.5f * (min + max);
	min = min - center;
	max = max - center;

	this->update(center, min, max);
}
