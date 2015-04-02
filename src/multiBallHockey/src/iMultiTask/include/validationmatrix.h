/***************************************************************************
 *   Copyright (C) 2007 by Zenon Mathews   *
 *   zmathews@iua.upf.edu   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef VALIDATIONMATRIX_H
#define VALIDATIONMATRIX_H



#include "cv.h"

/**
	@author Zenon Mathews <zmathews@iua.upf.edu>
*/
class ValidationMatrix{
public:
    ValidationMatrix(int nData, int nTargets);

    ~ValidationMatrix();

	const int nofTargets;
	const int nofData;
	
	CvMat* matrix;
	
	bool isTrue(int i, int j);

	void setTrue(int i, int j);

	void setFalse(int i, int j);
	
	// to set the validation gate value instead of just 0/1 (for testing)
	void setValue(int i, int j, float value);

	// returns value of the element (testing)
	float getValue(int i, int j);

	
	
};

#endif
