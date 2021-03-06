/********************************************************************************
 * OutOfBoundException.h
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

#ifndef OUTOFBOUNDEXCEPTION_H_
#define OUTOFBOUNDEXCEPTION_H_
// standard exceptions
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

class OutOfBoundException : public std::runtime_error {
public:
    OutOfBoundException(const std::string& message)
        : std::runtime_error(message) { };
};

#endif /* OUTOFBOUNDEXCEPTION_H_ */
