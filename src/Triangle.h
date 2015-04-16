/******************************************************************************
*     Copyright (C) 2013 TANGUY Arnaud arn.tanguy@gmail.com                   *
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
 ******************************************************************************/

#ifndef __Triangle__
#define __Triangle__

#include "CGEngine/Entity.h"
#include <glm/glm.hpp>
#include <GL/gl.h>

class Triangle : public Entity
{
    private:
        glm::vec3 mPoints[3];
        glm::vec3 mNormal;

    public:
        Triangle (const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3);
        virtual ~Triangle ();

        void setPoints(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3);

        virtual void render();
        virtual bool generate();

        float closestPointOnTriangleToPointVoronoi(const glm::vec3 &point, bool showDebug = true);
};



#endif
