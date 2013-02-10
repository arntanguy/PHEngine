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

#ifndef __mt__
#define __mt__

#include <glm/glm.hpp>

/**
 * @brief Math Tools
 */
namespace mt {
    float norm(const glm::vec3&);
    int rand(int min, int max);

    bool projectPointOnEdge(glm::vec3 e1, glm::vec3 e2, glm::vec3 point, glm::vec3 &projection);
    bool projectOnTriangle(const glm::vec3 &t1, const glm::vec3& t2, const glm::vec3 &t3, const glm::vec3 &normal, const glm::vec3& point, glm::vec3& projection);
    bool onEdge(const glm::vec3 &point, const glm::vec3& e1, const glm::vec3& e2);
} /* mt */

#endif
