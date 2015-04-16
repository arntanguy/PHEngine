/******************************************************************************
     Copyright (C) 2008  TANGUY Arnaud arn.tanguy@gmail.com
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

#include "mt.h"
#include <ctime>
#include <cstdlib>

namespace mt {
    /**
     * @brief Euclidian norm of vector
     *
     * @param v
     *    The vector
     *
     * @return
     *    The euclidian norm
     */
    float norm(const glm::vec3& v)
    {
        return glm::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    }

    int rand(int min, int max)
    {
        return (int) (((double) std::rand() / ((double)RAND_MAX+1)) * (max-min+1) + min);
    }
    /**
     * @brief Project point onto a plane, and then check whether the point is within triangles boundaries.
     *
     * Given point P and triangle A, B, C, compute:

     - the unit normal of triange (A, B, P)  - call it N1
     - the unit normal of triangle (B, C, P) - call it N2 (get the order right!)

     Now think about the dot product N1*N2. if P is in the plane of the triangle, and inside the three sides, those normals should be parallel, so this dot product will be 1.0000 (or 0.999...). If P is kept in the plane but moved beyond side BC, those two normals will be opposite: N1*N2==-1. If P isn't in the plane, the dot product will be some intermediate value. Whoops, we still have a loophole - if P goes out past side CA. We need to compute one more:

     - the unit normal (C,A,P) called N3
     Make these two tests (in an ideal world):

     N1*N2 == 1.0 ?
     N2*N3 == 1.0 ?
     (testing N3*N1 is redundant) Of course, the test will have to allow some slop for the imperfections of computer arithmetic. Look for (N1*N2 > 1-epsilon) where epsilon is some small value, depending on accuracy needed and floating point types.
     *
     *
     * @param t1
     *  Point 1 of the triangle
     * @param t2
     *  Point 2 of the triangle
     * @param t3
     *  Point 3 of the triangle
     * @param normal
     *  Normal of the triangle
     * @param point
     * Point to project.
     * @param projection
     *  Reference to the vector in which the result coordinates of the projection are set
     *
     * @return
     *  Boolean:
     *  - True when projection is in the triangle
     */
    bool projectOnTriangle(const glm::vec3 &t1, const glm::vec3& t2, const glm::vec3 &t3, const glm::vec3 &normal, const glm::vec3& point, glm::vec3 &projection)
    {
        glm::vec3 a, b, c, p;
        a=t1; b=t2; c=t3;
        projection = point-glm::dot(point-t1, normal)*normal;
        p = projection;

        // Check if inside the triangle
        glm::vec3 n1 = glm::normalize(glm::cross(b-a, p-a));
        glm::vec3 n2 = glm::normalize(glm::cross(c-b, p-b));
        glm::vec3 n3 = glm::normalize(glm::cross(a-c, p-c));

        return ((glm::dot(n1, n2) > 0.99)  && (glm::dot(n2,n3) > 0.99));
    }

    bool pointInTriangle(const glm::vec2& p, const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& p2) {
    	float Area = 1.f/2.f*(-p1.y*p2.x + p0.y*(-p1.x + p2.x) + p0.x*(p1.y - p2.y) + p1.x*p2.y);
    	float s = 1/(2*Area)*(p0.y*p2.x - p0.x*p2.y + (p2.y - p0.y)*p.x + (p0.x - p2.x)*p.y);
    	float t = 1/(2*Area)*(p0.x*p1.y - p0.y*p1.x + (p0.y - p1.y)*p.x + (p1.x - p0.x)*p.y);
    	return s>0 && t > 0 && 1-s-t > 0;
    }

    /**
     * @brief Gives the coordinates of the projected point onto an edge
     *
     * @param e1
     *  Edge point 1
     * @param e2
     *  Edge point 2
     * @param point
     *  Point to project
     * @param projection
     *  Return parameter: result of the projection
     *
     * @return
     *      A boolean:
     *      - True if point is in edge segment
     *      - False if point is outside of edge segment
     */
    bool projectPointOnEdge(glm::vec3 e1, glm::vec3 e2, glm::vec3 point, glm::vec3 &projection)
    {
        glm::vec3 ev = glm::normalize(e2 - e1);
        projection = e1 + glm::dot(point - e1, ev) * ev;
        // Check if on edge segment
        return (glm::dot(ev, projection-e1)  > 0 && glm::dot(e2-projection, ev) > 0);
    }

    bool onEdge(const glm::vec3 &point, const glm::vec3& e1, const glm::vec3& e2)
    {
        glm::vec3 ev = glm::normalize(e2-e1);
        return (glm::dot(ev, glm::normalize(point-e1))  > 0.9 && glm::dot(glm::normalize(e2-point), ev) > 0.9);
    }
} /* mt */

