/********************************************************************************
 * PhysicsBody.h
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

#ifndef PHYSICSBODY_H_
#define PHYSICSBODY_H_

#include "BoundingVolume.h"

class PhysicsBody {
private:
	static int id_counter;
	int id;
	void init();

protected:
	BoundingVolume *mBoundingVolume;

public:
	PhysicsBody();
	PhysicsBody(BoundingVolume *boundingVolume);
	virtual ~PhysicsBody();

	BoundingVolume *getBoundingBox();
	virtual void setBoundingBox(BoundingVolume *boundingBox);

	virtual void update(float) = 0;
	virtual void render() = 0;

	int getId() const {
		return id;
	}

};

#endif /* PHYSICSBODY_H_ */
