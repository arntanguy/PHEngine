/********************************************************************************
 * Grid.h
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

#ifndef GRID_H_
#define GRID_H_

#include <glm/glm.hpp>
#include <GL/gl.h>
#include "CGEngine/Entity.h"
#include "Vertex.h"

/**
 * Creates a grid that can be used to render terrains, water, or whatever you might need a grid for...
 * This grid is optimised so that it has a minimal number of vertices both in memory and on display.
 * See http://dan.lecocq.us/wordpress/2009/12/25/triangle-strip-for-grids-a-construction/ for details about the method used
 */
class Grid: public Entity {
public:
	enum VertexPos {
		TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT
	};

protected:
	int mGridSize;
	float mCellLength;

	GLuint mDisplayListIndex;

	int mNumberOfVertices;
	Vertex *mVertices;
	int *mIndices;

	glm::vec2 getTextureCoordinates(float, float) const;
	int getIndex(int i, int j, VertexPos pos) const;
	glm::vec3 computeNormal(int i, int j, VertexPos pos);

public:
	Grid(int gridSize, float gridLength);
	virtual ~Grid();

	void generateGrid();
	void generateVerticesDisplayList();

	bool generate();
	void render();

	Vertex getVertexForCell(int i, int j, VertexPos pos) const throw();
	void setVertexForCell(int i, int j, VertexPos pos, Vertex vertex);

	glm::vec2 getGridCoordinates(const glm::vec3 &coordinates) const;
};

#endif /* GRID_H_ */
