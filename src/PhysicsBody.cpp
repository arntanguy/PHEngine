/********************************************************************************
 * PhysicsBody.cpp
 *
 *  Created on: 24 Mar 2013
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

#include "PhysicsBody.h"

int PhysicsBody::id_counter = 0;

PhysicsBody::PhysicsBody() {
	mBoundingVolume = 0;
}

PhysicsBody::PhysicsBody(BoundingVolume *boundingVolume) {
	setBoundingBox(boundingVolume);
}

void PhysicsBody::init() {
	 id = id_counter++;
}

PhysicsBody::~PhysicsBody() {
	// TODO Auto-generated destructor stub
}


void PhysicsBody::setBoundingBox(BoundingVolume *boundingBox)
{
    mBoundingVolume = boundingBox;
}

BoundingVolume* PhysicsBody::getBoundingBox()
{
    return mBoundingVolume;
}



