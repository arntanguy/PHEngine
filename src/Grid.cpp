/********************************************************************************
 * Grid.cpp
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

#include "Grid.h"
#include <GL/glut.h>
#include "OutOfBoundException.h"
#include <iostream>
#include <sstream>

using namespace std;
using namespace glm;

/**
 * Creates a grid of gridSize*gridSize cells covering a total area of gridLength*gridLength
 * @param gridSize
 * 	Number of cells in the grid.
 * 	For instance 20 will give you a grid of 20 columns by 20 rows
 * @param gridLength
 * 	Total side length of the grid.
 * 	Each cell in the grid will have a size of gridLength/gridSize
 */
Grid::Grid(int gridSize, float gridLength) {
	mGridSize = gridSize + 1;
	mCellLength = gridLength / (float) gridSize;

	mNumberOfVertices = (mGridSize * mGridSize)
			+ (mGridSize - 1) * (mGridSize - 1);
	mVertices = new Vertex[(mGridSize + 1) * (mGridSize + 1)];
	mIndices = new int[mNumberOfVertices];
}

Grid::~Grid() {
	delete[] mIndices;
	delete[] mVertices;
}

/**
 * Returns texture coordinates, flipped from one cell to the next to get a repeating texture
 * @param x
 * @param z
 * @return
 */
glm::vec2 Grid::getTextureCoordinates(float x, float z) const {
	return glm::vec2(int(x) % 2, int(z) % 2);
}

/**
 * @brief
 * Generates an optimal grid, with no vertices repeated in memory.
 * Display is also optimised by creating an array of indices that can be used to display the grid
 * using a minimal number of vertices (only one is repeated at each row to link it to the next one).
 */
void Grid::generateGrid() {
	/**
	 * Variables used for the generate grid algorithm
	 */
	bool add = true;
	int i = 0;
	int j = 0;
	int loop = 0;
	int stepByRow = 2 * mGridSize;
	int substractVal = mGridSize - 1;
	int row = 0;
	Vertex v;
	int x, z;

	/**
	 * Loop to generateVertices all the vertices of the grid,
	 * with a minimum of vertex!
	 **/

	while (j < (stepByRow) * (mGridSize - 1)) {
		x = i % mGridSize;
		z = /*mGridSize -*/i / mGridSize;
		v.position = glm::vec3(x * mCellLength, 0, z * mCellLength);
		v.texcoords = getTextureCoordinates(x, z);

		// Add current index to indices table
		mIndices[j] = i;
		// Add corresponding vertex for index i
		mVertices[i] = v;

		// Little glue to change row:
		// Loop current indice (put it twice in the indices array.
		// Update substract value to get the right indices on next row
		if (loop == stepByRow - 1) {
			//cout << "loop back on i " << i << endl;
			// Add index twice so to link one row with the next one
			mIndices[++j] = i;

			loop = 0;
			if (substractVal == mGridSize - 1)
				substractVal = mGridSize + 1;
			else
				substractVal = mGridSize - 1;
			add = !add;
			row++;
		}

		if (add)
			i += mGridSize;
		else
			i -= substractVal;
		add = !add;
		loop++;
		j++;
	}

	//generateVerticesDisplayList();
}

/**
 * Compile the openGL commands needed to render the scene
 */
void Grid::generateVerticesDisplayList() {
	// create one display list
	mDisplayListIndex = glGenLists(1);

	// compile the display list, store a triangle in it
	glNewList(mDisplayListIndex, GL_COMPILE);
	glFrontFace(GL_CW); //   Vertex are added clockwise. Used to calculate normals

	Vertex v;
	int j = 0;

	glBegin(GL_TRIANGLE_STRIP);
	for (int i = 0; i < (2 * mGridSize) * (mGridSize - 1); i++) {
		v = mVertices[i];
		//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, v.texcoords.x, v.texcoords.y);
		glVertex3f(v.position.x, v.position.y, v.position.z);
	}
	glEnd();

	glEndList();
}

bool Grid::generate() {
	generateGrid();
	return true;
}

bool Grid::render() {
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	// draw the display list
	//glCallList(mDisplayListIndex);

	Vertex v;
	glBegin(GL_TRIANGLE_STRIP);
	for (int i = 0; i < mNumberOfVertices; ++i) {
		v = mVertices[mIndices[i]];
		//glMultiTexCoord2fARB(GL_TEXTURE0_ARB, v.texcoords.x, v.texcoords.y);
		glVertex3f(v.position.x, v.position.y, v.position.z);
	}
	glEnd();

	glPopAttrib();

	return true;
}
/**
 * Gets the index in the 1D vertex array
 * params:
 * 	- i : column number
 * 	- j : row number
 * 	- VertexPos : corner of the cell concerned
 */
int Grid::getIndex(int i, int j, VertexPos pos) const {
	int index = j * (mGridSize) + i;
	if (pos == TOP_RIGHT) {
		index += 1;
	} else if (pos == BOTTOM_LEFT) {
		index += mGridSize;
	} else if (pos == BOTTOM_RIGHT) {
		index += mGridSize + 1;
	}
	return index;
}

/**
 * @brief
 * Return the vertex for the corner "pos" of cell (i, j)
 */
Vertex Grid::getVertexForCell(int i, int j, VertexPos pos) const throw() {

	int index = getIndex(i, j, pos);

	if (index < mNumberOfVertices) {
		return mVertices[index];
	} else {
		stringstream ss;
		ss << "Out of bound for (" << i << ", " << j << ")" << endl;
		throw OutOfBoundException(ss.str());
	}
}

/**
 * @brief
 * Sets the vertex for corner "pos" of cell (i, j)
 */
void Grid::setVertexForCell(int i, int j, VertexPos pos, Vertex vertex) {
	int index = getIndex(i, j, pos);
	mVertices[index] = vertex;
}
