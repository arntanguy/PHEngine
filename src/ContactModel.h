/********************************************************************************
 * ContactModels.h
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

#ifndef CONTACTMODELS_H_
#define CONTACTMODELS_H_

#include <glm/glm.hpp>
#include "RigidBody.h"

struct ContactModel
{
    enum Type {VV, VE, EE, VF};
    Type type;
    glm::vec3 contactPoint;
    glm::vec3 contactPoint1;
    glm::vec3 contactPoint2;
    glm::vec3 normal;
    glm::vec3 edge1[2];
    glm::vec3 edge2[2];
    glm::vec3 edge3[2];
    float distance;

    RigidBody *rigidBody1;
    RigidBody *rigidBody2;
    bool contactHappened;
};


#endif /* CONTACTMODELS_H_ */
