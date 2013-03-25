/********************************************************************************
 * FluidSimulation.cpp
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

#include "FluidSimulation.h"
#include <glm/glm.hpp>
#include <GL/gl.h>
#include <iostream>
#include "DrawingTools.h"
#include "AABoundingBox.h"
#include <omp.h>
#include <time.h>

using namespace std;

FluidSimulation::FluidSimulation() : PhysicsBody() {
	init();
}

void FluidSimulation::init() {
	mBoundingBox = 0;
    setCollide(CollidingType::NONE);
}

FluidSimulation::FluidSimulation(int gridSize, float gridLength,
		float particleEmittingThreshold) : PhysicsBody() {
	init();
	mGridSize = gridSize;
	mCellLength = gridLength / gridSize;
	mParticleEmittingThreshold = particleEmittingThreshold;
	mGridLength = gridLength;
	mMaxX = gridLength;
	mMinX = 0;

	mCells = new Array2D<Cell*>(gridSize, gridSize);
	for (int i = 0; i < gridSize; i++) {
		for (int j = 0; j < gridSize; j++) {
			(*mCells)(i, j) = new Cell();
			(*mCells)(i, j)->setVolume(5000);
		}
	}
	//(*mCells)(50, 50)->setVolume(10000);
	(*mCells)(0, 0)->setVolume(10000);
	(*mCells)(99, 99)->setVolume(10000);
	(*mCells)(99, 0)->setVolume(10000);
	(*mCells)(0, 99)->setVolume(10000);
	(*mCells)(50, 50)->setExternalForce(50000);

	// Fluid density
	p = 1; // water
	// Atmospheric pressure in the system (in Pa)
	p0 = 101325;
	// Gravity
	g = 9.8;
	c = 15; //gridLength*gridLength;
	pipeLength = 1;

	generatePipes();

	mGrid = new Grid(gridSize, gridLength);
	mGrid->generate();
}

FluidSimulation::~FluidSimulation() {
	delete mCells;
}

bool FluidSimulation::generate() {

	return true;
}

/**
 * Generate a system of pipes linking every column with each other.
 * Fluid will flow through the pipes according to physics law of fluid mechanics.
 */
void FluidSimulation::generatePipes() {
	for (int i = -1; i < mGridSize; i++) {
		for (int j = -1; j < mGridSize + 1; j++) {
			/**
			 * Creates pipes to the right
			 */
			mPipes[MultiKey(i, j, i + 1, j)] = new Pipe();
		}
	}

	for (int i = -1; i < mGridSize + 1; i++) {
		for (int j = -1; j < mGridSize; j++) {
			/**
			 * Creates pipes down
			 */
			mPipes[MultiKey(i, j, i, j + 1)] = new Pipe();
		}
	}

	// XXX: pipe in the corners are useless.
	for (int i = -1; i < mGridSize; i++) {
		for (int j = -1; j < mGridSize; j++) {
			/**
			 * Creates pipes to the diagonal
			 */
			mPipes[MultiKey(i, j, i + 1, j + 1)] = new Pipe();
			mPipes[MultiKey(i, j + 1, i + 1, j)] = new Pipe();
		}
	}
}

/**
 * Update the state of the fluid simulation:
 * - Recompute all the flows based on the previous flow,
 * 		and acceleration due to gravity, air pressure and other external forces
 * - Recompute all the cells volume according to flow
 * @param timeEllapsed
 * 	Timestep for the update
 */
void FluidSimulation::update(float timeEllapsed) {
	timeEllapsed = 0.1;

	clock_t t = clock();

	map<MultiKey, Pipe*>::iterator it = mPipes.begin();
	for (auto it = mPipes.begin(); it != mPipes.end(); it++) {
		MultiKey indices = it->first;
		Pipe *pipe = it->second;

		int i = indices.indices[0];
		int j = indices.indices[1];
		int k = indices.indices[2];
		int l = indices.indices[3];

		/*std::cout << "(" << indices.i() << "," << indices.j() << " -> "
		 << indices.k() << "," << indices.l() << ")" << std::endl;*/
		Cell *cell1 = 0;
		Cell *cell2 = 0;
		if (i >= 0 && j >= 0 && k >= 0 && l >= 0 && i < mGridSize
				&& j < mGridSize && k < mGridSize && l < mGridSize) {
			cell1 = (*mCells)(i, j);
			cell2 = (*mCells)(k, l);
			if (cell1 != 0 && cell2 != 0) {
				/*cout << "(" << i << ", " << j << "): ";
				 cell1->printDebug();
				 cout << "(" << k << ", " << l << "): ";
				 cell2->printDebug();*/

				float hij = cell1->getHeight(mCellLength);
				float hkl = cell2->getHeight(mCellLength);

				float Hij = hij * p * g + p0;
				float Hkl = hkl * p * g + p0;
				float Eij = cell1->getExternalForce();
				float Ekl = cell2->getExternalForce();
				float aij_kl = (p * g * (hij - hkl) + (Eij - Ekl))   / (p * pipeLength);
				float Qij_kl = timeEllapsed * (c * aij_kl);
				float newFlow = Qij_kl + pipe->getPreviousFlow();
				pipe->setFlow(newFlow);

//				if (i == 1 && j == 0) {
					/*cout << "hij(" << hij << "), hkl(" << hkl
					 << "), Hij(" << Hij << "), Hkl(" << Hkl
					 << "), Pij_kl(" << Pij_kl << "), a_ijkl("
					 << aij_kl << "), Qij_kl(" << Qij_kl << endl;
					 cout << "Previous Flow: " << pipe->getPreviousFlow()
					 << ", current flow: " << pipe->getFlow()
					 << endl;
					 cout << "Diff of volume (" << cell1->getVolume()
					 << " - " << cell2->getVolume() << ") : "
					 << cell1->getVolume() - cell2->getVolume()
					 << endl;
					 cout << "hij " << hij << ", hkl " << hkl << endl;*/
//				}
			} else {
				cout << "Outer cell" << endl;
			}
		}
	}

	/**
	 * Update volume
	 */
	for (auto it = mPipes.begin(); it != mPipes.end(); it++) {
		MultiKey indices = it->first;
		Pipe *pipe = it->second;

		int i = indices.indices[0];
		int j = indices.indices[1];
		int k = indices.indices[2];
		int l = indices.indices[3];

		Cell *cell1 = 0;
		Cell *cell2 = 0;
		if (i >= 0 && j >= 0 && k >= 0 && l >= 0 && i < mGridSize
				&& j < mGridSize && k < mGridSize && l < mGridSize) {
			cell1 = (*mCells)(i, j);
			cell2 = (*mCells)(k, l);
			if (cell1 != 0 && cell2 != 0) {
				float netVolumeChange = timeEllapsed
						* (pipe->getPreviousFlow() + pipe->getFlow()) / 2;

				cell1->updateVolume(-netVolumeChange);
				cell2->updateVolume(netVolumeChange);
				if (cell1->getVolume() < 0 || cell2->getVolume() < 0)
					cerr << "Negative Volume!!" << endl;
//				if (i == 1 && j == 0) {
//					cout << "Net volume change(" << netVolumeChange << ")"
//							<< endl;
//				}
			}
		}
	}

	/**
	 * Update Grid
	 */
	//cout << "Cell size " << mCells->getWidth() << endl;
	float max = 0; float min=__INT_MAX__;
	Cell *cell = 0;
	for (int i = 0; i < mCells->getWidth(); i++) {
		for (int j = 0; j < mCells->getHeight(); j++) {
			/**
			 * Update particles
			 */
			//emitParticles(i, j, timeEllapsed);

			cell = (*mCells)(i, j);
			max = glm::max(cell->getHeight(mCellLength), max);
			min = glm::min(cell->getHeight(mCellLength), min);

			/**
			 * Update Grid
			 */
			float topleft = getInterpolatedHeight(i, j, i - 1, j - 1, i, j - 1,
					i - 1, j);
			float topright = getInterpolatedHeight(i, j, i, j - 1, i + 1, j - 1,
					i + 1, j);
			float bottomleft = getInterpolatedHeight(i, j, i - 1, j, i - 1,
					j + 1, i, j + 1);
			float bottomright = getInterpolatedHeight(i, j, i + 1, j, i + 1,
					j + 1, i, j + 1);

			Vertex topleftV = mGrid->getVertexForCell(i, j,
					Grid::VertexPos::TOP_LEFT);
			topleftV.position.y = topleft;
			Vertex toprightV = mGrid->getVertexForCell(i, j,
					Grid::VertexPos::TOP_RIGHT);
			toprightV.position.y = topright;
			Vertex bottomLeftV = mGrid->getVertexForCell(i, j,
					Grid::VertexPos::BOTTOM_LEFT);
			bottomLeftV.position.y = bottomleft;
			Vertex bottomRightV = mGrid->getVertexForCell(i, j,
					Grid::VertexPos::BOTTOM_RIGHT);
			bottomRightV.position.y = bottomright;

			mGrid->setVertexForCell(i, j, Grid::VertexPos::TOP_LEFT, topleftV);
			mGrid->setVertexForCell(i, j, Grid::VertexPos::TOP_RIGHT,
					toprightV);
			mGrid->setVertexForCell(i, j, Grid::VertexPos::BOTTOM_LEFT,
					bottomLeftV);
			mGrid->setVertexForCell(i, j, Grid::VertexPos::BOTTOM_RIGHT,
					bottomRightV);
		}
	}

	if(mBoundingBox != 0) {
		mBoundingBox->manualUpdate(glm::vec3(mMinX, min, mMinX), glm::vec3(mMaxX, max, mMaxX));
	}

	t = clock() - t;
	cout << "Time update " << t / 1000 << " ms." << endl;
}

float FluidSimulation::upwardsVelocity(int i, int j) {
//	cout << "(" << i - 1 << ", " << j << ") -> (" << i << ", " << j << ")"
//			<< endl;
	Pipe * q1 = mPipes[MultiKey(i - 1, j, i, j)];
	Pipe * q2 = mPipes[MultiKey(i, j, i + 1, j)];
	Pipe * q3 = mPipes[MultiKey(i, j - 1, i, j)];
	Pipe * q4 = mPipes[MultiKey(i, j, i, j + 1)];
	Pipe * q5 = mPipes[MultiKey(i - 1, j - 1, i, j)];
	Pipe * q6 = mPipes[MultiKey(i, j, i + 1, j + 1)];
	Pipe * q7 = mPipes[MultiKey(i, j, i + 1, j - 1)];
	Pipe * q8 = mPipes[MultiKey(i - 1, j + 1, i, j)];
	float f1 = (q1 != 0) ? q1->getFlow() : 0;
	float f2 = (q2 != 0) ? q2->getFlow() : 0;
	float f3 = (q3 != 0) ? q3->getFlow() : 0;
	float f4 = (q4 != 0) ? q4->getFlow() : 0;
	float f5 = (q5 != 0) ? q5->getFlow() : 0;
	float f6 = (q6 != 0) ? q6->getFlow() : 0;
	float f7 = (q7 != 0) ? q7->getFlow() : 0;
	float f8 = (q8 != 0) ? q8->getFlow() : 0;

	return (f1 - f2 + f3 - f4 + f5 - f6 - f7 + f8) / (mCellLength * mCellLength);
}
void FluidSimulation::emitParticles(int i, int j, float timeEllapsed) {
	Cell *cell = (*mCells)(i, j);

	float up = cell->upwardsVelocity(mCellLength);
	if (up > mParticleEmittingThreshold) {
		//float percent = timeEllapsed/1000;
		//up = (1-percent)*up + up;
		/*float yv = (upwardsVelocity(i, j) + upwardsVelocity(i, j + 1)
		 + upwardsVelocity(i + 1, j) + upwardsVelocity(i + 1, j + 1)) / 4.f;

		 float xv = (mPipes[MultiKey(i, j, i + 1, j)]->getFlow()
		 + mPipes[MultiKey(i, j + 1, i + 1, j)]->getFlow())
		 / (2 * mCellLength * mCellLength);
		 float zv = (mPipes[MultiKey(i, j, i, j + 1)]->getFlow()
		 + mPipes[MultiKey(i, j + 1, i + 1, j + 1)]->getFlow())
		 / (2 * mCellLength * mCellLength);*/
		float xv;
		if (i >= 0 && i < mGridSize - 1)
			xv = (*mCells)(i + 1, j) - (*mCells)(i, j);
		else
			xv = 0 - (*mCells)(i, j)->getVolume();
		float zv;
		if (j >= 0 && j < mGridSize - 1)
			zv = (*mCells)(i, j + 1) - (*mCells)(i, j);
		else
			zv = 0 - (*mCells)(i, j)->getVolume();
		float yv = up;
		xv = xv/(mCellLength*mCellLength);
		zv = zv/(mCellLength*mCellLength);
		cout << "Cell(" << i << ", " << j
				<< ") emmiting particle with velocity: (" << xv << ", " << yv
				<< "(" << up << ") , " << zv << ")" << endl;
	}
}

/**
 * Interpolate height of control point between adjacent columns
 * (x, y) : current column
 * (x1, y1), (x2, y2), (x3, y3), (x4, y4) : adjacent columns
 */
float FluidSimulation::getInterpolatedHeight(int x, int y, int x1, int y1,
		int x2, int y2, int x3, int y3) const {
	float h, h1, h2, h3;
	if (x >= 0 && y >= 0 && x < mGridSize && y < mGridSize)
		h = ((*mCells)(x, y))->getHeight(mCellLength);
	else
		h = 0;
	if (x1 >= 0 && y1 >= 0 && x1 < mGridSize && y1 < mGridSize)
		h1 = ((*mCells)(x1, y1))->getHeight(mCellLength);
	else
		h1 = h;
	if (x2 >= 0 && y2 >= 0 && x2 < mGridSize && y2 < mGridSize)
		h2 = ((*mCells)(x2, y2))->getHeight(mCellLength);
	else
		h2 = h;
	if (x3 >= 0 && y3 >= 0 && x3 < mGridSize && y3 < mGridSize)
		h3 = ((*mCells)(x3, y3))->getHeight(mCellLength);
	else
		h3 = h;
	return (h + h1 + h2 + h3) / 4.f;
}

void FluidSimulation::render() {
	/*for (int i = 0; i < mGridSize; i++) {
	 for (int j = 0; j < mGridSize; j++) {
	 debugRenderCell(i, j);
	 }
	 }*/

	mGrid->render();
	//debugRenderPipes();
}

/**
 * Renders the cell volume, as coloured squares representing the height.
 * This is merely used for debug purposes.
 * @param i
 * @param j
 */
void FluidSimulation::debugRenderCell(int i, int j) {
	Cell *cell = (*mCells)(i, j);
//std::cout << "cell(" << i << ", " << j << "), volume: " << cell.getVolume() << std::endl;
	float x1 = mCellLength * i;
	float y1 = mCellLength * j;
	float x2 = x1 + mCellLength;
	float y2 = y1 + mCellLength;
	float z = cell->getHeight(mCellLength);
	glPushMatrix();
	glColor3f(0.1 + (100 * i % 255) / 255.f, 0.1 + (100 * j % 255) / 255.f,
			0.1 + (100 * (i + j) % 255) / 255.f);
	glBegin(GL_TRIANGLES);
	glVertex3f(x1, z, y1);
	glVertex3f(x2, z, y1);
	glVertex3f(x2, z, y2);

	glVertex3f(x2, z, y2);
	glVertex3f(x1, z, y2);
	glVertex3f(x1, z, y1);
	glEnd();
	glPopMatrix();
}

/**
 * Renders all the pipes.
 * This is used for debug purposes
 */
void FluidSimulation::debugRenderPipes() {
	int i = 0;
	for (auto it = mPipes.begin(); it != mPipes.end(); it++) {
		MultiKey indices = it->first;
		//Pipe *pipe = it->second;

		//std::cout << i++ << " Pipe("<<indices.i() << "," << indices.j() << " -> " << indices.k() << "," << indices.l() << ")"<< std::endl;

		float x1 = mCellLength * indices.i() + mCellLength / 2;
		float z1 = mCellLength * indices.j() + mCellLength / 2;
		float x2 = mCellLength * indices.k() + mCellLength / 2;
		float z2 = mCellLength * indices.l() + mCellLength / 2;

		glm::vec3 p1(x1, 0, z1);
		glm::vec3 p2(x2, 0, z2);
		dt::drawLine(p1, p2);
	}
}

void FluidSimulation::applyForceToControlPoint(int ci, int cj, float force) {
	float Eij = force/c;
	if(ci > 0 && cj > 0) {
		Cell *c1 = (*mCells)(ci-1, cj-1);
		Cell *c2 = (*mCells)(ci-1, cj);
		Cell *c3 = (*mCells)(ci, cj-1);
		Cell *c4 = (*mCells)(ci, cj);
		c1->setExternalForce(Eij);
		c2->setExternalForce(Eij);
		c3->setExternalForce(Eij);
		c4->setExternalForce(Eij);
	} else {
		cerr << "WARNING: applyForceToControlPoint: Border not handled for now!" << endl;
	}
}


void FluidSimulation::setBoundingBox(BoundingVolume *boundingBox)
{
	AABoundingBox *bb=0;
	bb = dynamic_cast<AABoundingBox *>(boundingBox);
	if(bb != 0) {
		cout << "Setting bounding box" << endl;
		mBoundingBox = bb;
	} else {
		mBoundingBox = 0;
	}
}

BoundingVolume* FluidSimulation::getBoundingBox() {
	return mBoundingBox;
}

ContactModel* FluidSimulation::distanceToPhysicsBody(PhysicsBody* physicsBody) {
	cerr << "Distance from fluid to physics body not implemented" << endl;
	return 0;
}

