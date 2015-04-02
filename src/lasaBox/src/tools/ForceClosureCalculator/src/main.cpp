#include "forceClosure.h"

#include "Matrix.h"

#include <iostream>
using namespace std;
using namespace MathLib;

/**
 *
\defgroup icub_ForceClosureCalculator ForceClosureCalculator
@ingroup icub_lasaBox_tools

This software helps computing force closure and grasp quality measure from an input file.

\section intro_sec Description

This executable computes force closure and grasp quality measure from an input file
and write the results in an output file.

The input file has to ba a matrix tyoe of file.
Each row is an entry that contains:
- N 3d points (N>=3), followed by
- N 3d normals

The output file is a 2 column matrix where each column corresponds to:
- A boolean indicating if the entry is force closure
- The grasp quality measure (a real number)


\section dependencies_sec Dependencies

This software requires the qhull library that can be found here:  http://www.qhull.org.
For convienience, the library also is distributed here.

To compile libqhull locally:
\verbatim
> cd $ICUB_ROOT/contrib/src/lasaBox/src/tools/ForceClosureCalculator/lib
> ./build_libqhull.sh 
\endverbatim

And the most part of the code has been written by Juergen Dietel, modified my Sahar El Khoury and an unknown great coder.


\section parameters_sec Parameters

This software requires minimum two parameters: an input and an output filenames
A third parameter can be used to specify the friction coefficient (between 0 and 1)


\section tested_os_sec Tested OS

Linux

\section example_sec Example Instantiation of the Module

\verbatim
ForceClosureCalculator inputFileName outptFileName [friction_coef]
ForceClosureCalculator a.txt b.txt 0.1
\endverbatim

\author Eric Sauser (the packaging), but part of the code has been written by Juergen Dietel, modified my Sahar El Khoury and an unknown great coder.

Copyright (C) 2011 RobotCub Consortium
 
CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at $ICUB_ROOT/contrib/src/lasaBox/src/tools/ForceClosureCalculator
**/


int main(int argc, char* argv[]){
    if(!((argc==3)||(argc==4))){
        cout << "Usage: ForceClosureCalculator inputFilename outputFilename [mu_coefficient]"<<endl;
        return 0;
    }

    Matrix data;
    if(data.Load(argv[1])){
       
        Matrix mres(data.RowSize(),2);

        int nContact = data.ColumnSize()/(3*2);
        if(nContact*(3*2) != data.ColumnSize()){
            cout << "Error: Wrong number of columns("<<mres.ColumnSize()<<"). Must be a multiple of 6."<<endl;
            return 0;            
        }
        if(nContact<3){
            cout << "Error: Minimun number of contact is 3"<<endl;
            return 0;            
        }

        Vecteur pos[3];
        Vecteur nrm[3];
        
        float mu = 0.7;
        if(argc==4){
            mu = atof(argv[3]);
            if(mu<0.0) mu = 0.0;
            if(mu>1.0) mu = 1.0;
        }
        cout << "Using a friction coefficient of: " <<mu<<endl;

        
        float radius;

        cout << "Starting..."<<endl;
        for(int j=0;j<data.RowSize();j++){
            if(j%10==0)
                cout << "Currently computing row: "<<j+1<<"/"<<data.RowSize()<<"..."<<endl;

            for(int i=0;i<nContact;i++){
                pos[i].initVecteur3D(data(j,i*3+0),data(j,i*3+1),data(j,i*3+2));
                nrm[i].initVecteur3D(data(j,nContact*3+i*3+0),data(j,nContact*3+i*3+1),data(j,nContact*3+i*3+2));
                nrm[i].normalisation();
            }

            int res = testeForceClosure(pos,nrm,mu,nContact,10,&radius,1.0);
            
            if(res==0) radius = 0.0;
            
            mres(j,0) = res;
            mres(j,1) = radius;
            //cout << j<<"/"<<data.RowSize()<<" FC: "<<res<<" "<<radius<<endl;
        }
        cout << "Done..."<<endl;

        if(!mres.Save(argv[2])){
            cout << "Error: File "<< argv[2]<<" could not be saved."<<endl;
        }
        
    }else{
        cout << "Error: File "<< argv[1]<<" could not be opened."<<endl;
    }
    return 0;
}
