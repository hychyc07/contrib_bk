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
#include "validationmatrix.h"

ValidationMatrix::ValidationMatrix(int nData, int nTargets):nofData(nData),nofTargets(nTargets)
{
		
		matrix = cvCreateMat(nofData,nofTargets,CV_32FC1);
		/// initialize all elements to zero
		 for (int i =0; i < nofData; i++){
			for (int j =0; j < nofTargets; j++){
				cvmSet(matrix,i,j,0);	
			}
		}
}


ValidationMatrix::~ValidationMatrix()
{
	cvReleaseMat(&matrix);
}






bool ValidationMatrix::isTrue(int i, int j){
	// return true if element (i,j) is 1
	if (cvmGet(matrix, i,j) > 0){
		return true;
	}
	else{
		return false;
	}
}

void ValidationMatrix::setTrue(int i, int j){
	cvmSet( matrix, i, j, 1);
}


void ValidationMatrix::setFalse(int i, int j){
	cvmSet( matrix, i, j, 0);
}


void ValidationMatrix::setValue(int i, int j, float val){
	cvmSet( matrix, i, j, val);
}

float ValidationMatrix::getValue(int i, int j){
	return cvmGet( matrix, i, j);
}