/********************************************************************************
 * Cell.cpp
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
#include "Cell.h"
#include <iostream>

using namespace std;

Cell::Cell() {
	mVolume = 0;
	mPreviousVolume = 0;
	mExternalForce = 0;
	done = false;
}

Cell::~Cell() {
}

void Cell::updateVolume(float amount) {
	setVolume(mVolume + amount);
}

void Cell::setVolume(float volume) {
	if (volume >= 0) {
		mPreviousVolume = mVolume;
		mVolume = volume;
	}
}

float Cell::getVolume() const {
	return mVolume;
}

float Cell::getHeight(float dx) const {
	return mVolume / (dx * dx);
}

float Cell::upwardsVelocity(float dx) const {
	return (mVolume - mPreviousVolume) / (dx * dx);
}

void Cell::printDebug() const {
	cout << "Cell: volume(" << mVolume << ")" << endl;
}

void Cell::setExternalForce(float force) {
	//cout << "Set external force... "<<force << endl;
//	if (!done) {
//		cout << "Set external force " << force << endl;
		mExternalForce = force;
//		done = true;
//	}
}

/**
 * Get the external force defined,
 * and then sets the force back to 0
 * @return
 * 	The external force value
 */
float Cell::getExternalForce() {
	float e = mExternalForce;
	mExternalForce = 0;
	return e;
}

