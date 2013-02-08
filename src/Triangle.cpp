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


#include "Triangle.h"
#include "Debug.h"
#include <GL/glut.h>
#include "mt.h"
#include "DrawingTools.h"

Triangle::Triangle (const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3)
{
    setPoints(p1, p2, p3);
}

Triangle::~Triangle()
{
}

void Triangle::setPoints(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3)
{
    mPoints[0] = p1;
    mPoints[1] = p2;
    mPoints[2] = p3;

    mNormal = glm::normalize(glm::cross(p2-p1, p3-p1));
}

bool Triangle::render()
{
    glBegin(GL_TRIANGLES);
        glVertex3f(mPoints[0].x, mPoints[0].y, mPoints[0].z);
        glVertex3f(mPoints[1].x, mPoints[1].y, mPoints[1].z);
        glVertex3f(mPoints[2].x, mPoints[2].y, mPoints[2].z);
    glEnd();
}

bool Triangle::generate()
{
    return true;
}

/**
 * @brief Find the closest distance from a point to a triangle based on voronoi regions
 *
 * @param point
 *  The point to check against the triangle
 * @param showDebug
 *  A debug boolean. If true, debug objects will be drawn on the scene
 *
 * @return
 *  The minimal distance between point and triangle
 */
float Triangle::closestPointOnTriangleToPointVoronoi(const glm::vec3 &point, bool showDebug)
{
    /**
     * Check closest point on triangle, using voronoi regions
     **/
    bool inEdgeRegion = false;
    glm::vec3 bestPoint;

    glm::vec3 edge[2];
    int i=0;
    glm::vec3 AX, AB, AC, BC, BX, BA;
    while(i < 3) {
        std::cout << "point: " << mPoints[i].x << " " << mPoints[i].y << " " << mPoints[i].z << std::endl;
        if(i==0) {
            AX = point-mPoints[0];
            BX = point-mPoints[1];
            AB = mPoints[1]-mPoints[0];
            AC = mPoints[2]-mPoints[0];
            BC = mPoints[2]-mPoints[1];
            edge[0] = mPoints[1];
            edge[1] = mPoints[0];
        } else if(i == 1) {
            AX = point-mPoints[1];
            BX = point-mPoints[2];
            AB = mPoints[2]-mPoints[1];
            BA = mPoints[1]-mPoints[2];
            AC = mPoints[0]-mPoints[1];
            BC = mPoints[0]-mPoints[2];
            edge[0] = mPoints[2];
            edge[1] = mPoints[1];
        } else if(i==2) {
            AX = point-mPoints[2];
            BX = point-mPoints[0];
            AB = mPoints[0]-mPoints[2];
            AC = mPoints[1]-mPoints[2];
            BC = mPoints[1]-mPoints[0];
            edge[0] = mPoints[0];
            edge[1] = mPoints[2];
            ////edge[0] = mPoints[0];
            ////edge[1] = mPoints[2];
        }

        std::cout << "i: " << i << std::endl;
        /**
         * Check vertex region
         **/
        if(glm::dot(AX, AB) <= 0 && glm::dot(AX, AC) <= 0) {
                std::cout << "vertex region for i: " << i << std::endl;
                bestPoint = mPoints[i];
                inEdgeRegion = false;
                break;
        } else if(glm::dot(glm::cross(glm::cross(BC, -AB), -AB), BX) >= 0
                && glm::dot(AX,AB) >= 0
                && glm::dot(BX, -AB) >= 0) {
                std::cout << "edge for i: " << i << std::endl;
            /**
             * Check edge region
             **/
            inEdgeRegion = true;

            /**
             * Project point onto line to get position of minimal distance
             **/
            glm::vec3 e = glm::normalize(glm::vec3(edge[1]-edge[0]));
            bestPoint = edge[0] + glm::dot(point-edge[0], e)*e;

            break;
        } else {
            std::cout << "nothing for i: " << i << std::endl;
        }
        i++;
    }
    if(showDebug) {
        dt::drawPoint(point, 0.3, glm::vec3(0., 1., 0.));
        glBegin(GL_LINES);
        glColor3f(1., 1., 1.);
        glVertex3f(point.x, point.y, point.z);
        glVertex3f(bestPoint.x, bestPoint.y, bestPoint.z);
        glEnd();
        if(inEdgeRegion) {
                dt::drawLine(edge[0], edge[1], mNormal, 0.2, 0.2, glm::vec3(0., 0., 1.));
                dt::drawPoint(bestPoint, 0.3);
        } else {
                dt::drawPoint(bestPoint, 0.3);
        }
    }

    std::cout << "distance: " << mt::norm(point-bestPoint) << std::endl;
    return mt::norm(point-bestPoint);
}
