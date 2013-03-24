/********************************************************************************
 * Pipe.cpp
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

#include "Pipe.h"

Pipe::Pipe() {
	init();
}

void Pipe::init() {
	mFlow = 0;
	mPreviousFlow = 0;
}

Pipe::~Pipe() {
	// TODO Auto-generated destructor stub
}


float Pipe::getFlow() const {
	return mFlow;
}

float Pipe::getPreviousFlow() const {
	return mPreviousFlow;
}

void Pipe::setFlow(float newFlow) {
	mPreviousFlow = mFlow;
	mFlow = newFlow;
}

