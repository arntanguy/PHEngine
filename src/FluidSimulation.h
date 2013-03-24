/********************************************************************************
 * FluidSimulation.h
 *
 *  Created on: 23 Mar 2013
 *      Author: Arnaud TANGUY
 *
 *    Copyright (C) 2013  TANGUY Arnaud arn.tanguy@gmail.com
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
 ********************************************************************************/

#ifndef CELLGRID_H_
#define CELLGRID_H_

#include "Entity.h"
#include "Cell.h"
#include "Array2D.h"
#include "Pipe.h"
#include <unordered_map>
#include <glm/glm.hpp>
#include <array>
#include "Grid.h"
#include "PhysicsBody.h"

class AABoundingBox;

/**
 * Key from one cell to the next cell of the grid (i,j)->(k,l).
 * Ordered using the lexicographical comparaison of (i,j) and (k,l)
 */
struct MultiKey {
	std::array<int, 4> indices;

	MultiKey(int i, int j, int k, int l) {
		indices[0] = i;
		indices[1] = j;
		indices[2] = k;
		indices[3] = l;
	}
	bool operator<(const MultiKey &r) const {
		return std::lexicographical_compare(indices.begin(), indices.end(),
				r.indices.begin(), r.indices.end());
	}

	int i() {
		return indices[0];
	}
	int j() {
		return indices[1];
	}
	int k() {
		return indices[2];
	}
	int l() {
		return indices[3];
	}

};

/**
 * Fluid simulation based on the paper
 *	"Dynamic Simulation of Splashing Fluids" - James F. O'Brien and Jessica K. Hodgins
 */
class FluidSimulation: public PhysicsBody {
private:
	void init();

protected:
	int mGridSize;
	float mCellLength;
	float mGridLength;
	float mParticleEmittingThreshold;
	float mMaxX, mMinX;

	Grid *mGrid;

	// Fluid density
	float p;
	// Atmospheric pressure in the system
	float p0;
	// Gravity
	float g;
	float c;
	// Length of pipe
	float pipeLength;

	Array2D<Cell*> *mCells;
	std::map<MultiKey, Pipe*> mPipes;

	AABoundingBox *mBoundingBox;

	void generateCellGrid();
	void generatePipes();

	void generateGrid();
	float getInterpolatedHeight(int x, int y, int x1, int y1, int x2, int y2,
			int x3, int y3) const;
	void emitParticles(int i, int j, float timeEllapsed);
	float upwardsVelocity(int i, int j);
public:
	FluidSimulation();
	FluidSimulation(int gridSize, float gridLength,
			float particleEmittingThreshold);
	virtual ~FluidSimulation();

	bool generate();
	virtual void update(float timeEllapsed);
	virtual void render();
	void debugRenderCell(int i, int j);
	void debugRenderPipes();
	void applyForceToControlPoint(int ci, int cj, float force);
	virtual void setBoundingBox(BoundingVolume *boundingBox);

};

#endif /* CELLGRID_H_ */
