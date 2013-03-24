/********************************************************************************
 * Array2D.h
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

template <typename T>
class Array2D
{
public:
  // constructor
  Array2D(unsigned wd,unsigned ht)
    : nWd(wd), nHt(ht), pAr(0)
  {
    if(wd > 0 && ht > 0)
      pAr = new T[wd * ht];
  }

  // destructor
  ~Array2D()
  {
    delete[] pAr;
  }

  // indexing (parenthesis operator)
  //  two of them (for const correctness)

  const T& operator() (unsigned x) const
  {
	  return pAr [ x ];
  }
  T& operator() (unsigned x)
  {
	  return pAr [ x ];
  }

  const T& operator () (unsigned x,unsigned y) const
  {  return pAr[ y*nWd + x ];   }

  T& operator () (unsigned x,unsigned y)
  {  return pAr[ y*nWd + x ];   }

  // get dims
  unsigned getWidth() const { return nWd; }
  unsigned getHeight() const { return nHt; }


  // private data members
private:
  unsigned nWd;
  unsigned nHt;
  T*       pAr;

  // to prevent unwanted copying:
  Array2D(const Array2D<T>&);
  Array2D& operator = (const Array2D<T>&);
};
