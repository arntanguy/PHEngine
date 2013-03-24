/********************************************************************************
 * Cell.h
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
#ifndef CELL_H_
#define CELL_H_

class Cell {
public:
	float mVolume;
	float mPreviousVolume;
	float mExternalForce;

public:
	Cell();
	virtual ~Cell();

	void updateVolume(float amount);
	void setVolume(float volume);
	float getVolume() const;
	float upwardsVelocity(float dx) const;

	float getHeight(float dx) const;

	void printDebug() const;

	void setExternalForce(float force);
	float getExternalForce();
};

#endif /* CELL_H_ */
