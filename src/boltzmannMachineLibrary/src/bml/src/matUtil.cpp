// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */
  
/**
 * @file matUtil.h
 * @brief definition of the module that creates a structure from scratch
 */

#include <iCub/matUtil.h>
#include <yarp/sig/all.h>


// --------- methods of randomMatrix ------------
randomMatrix::randomMatrix():Matrix(){
    double *p=this->data();
    for(int row=0;row< this->rows();row++){
        for(int col=0;col<this->cols();col++){
            *p=yarp::math::Rand().scalar();
            p++;
        }
    }
};

randomMatrix::randomMatrix(int r, int c):Matrix(r,c){
    double *p=this->data();
    for(int row=0;row< this->rows();row++){
        for(int col=0;col<this->cols();col++){
            *p=yarp::math::Rand().scalar();
            p++;
        }
    }
}


//--------- methods of bmlMatrix ---------

/*static bmlMatrix bmlMatrix::operator+(const yarp::sig::Matrix &m, double c){
    printf("the matrix is multiplied by %f ", c);
    Matrix res(m.rows(),m.cols());
    for (int row=0;row<m.rows();row++)
        for(int col=0;col<m.cols();col++)
            res(row,col)=m(row,col)+c;
    return res;
}

static bmlMatrix bmlMatrix::operator*(yarp::sig::Matrix &m,double c){
    Matrix res(m.rows(),m.cols());
    for (int row=0;row<m.rows();row++)
        for(int col=0;col<m.cols();col++)
            res(row,col)=m(row,col)*c;
    return res;
}

static bmlMatrix bmlMatrix::operator/(double c,yarp::sig::Matrix &m){
    Matrix res(m.rows(),m.cols());
    for (int row=0;row<m.rows();row++)
        for(int col=0;col<m.cols();col++)
            res(row,col)=c/m(row,col);
    return res;
}

static bool bmlMatrix::operator^(const yarp::sig::Matrix &m, double b){
    printf("the matrix is multiplied by %f ", b);
    return true;
}*/