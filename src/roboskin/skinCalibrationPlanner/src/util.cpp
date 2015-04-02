/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Andrea DelPrete
  * email: andrea.delprete@iit.it
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

#include "iCub/skinCalibrationPlanner/util.h"
#include <string>
#include <stdarg.h>     // va_list va_arg va_end va_start
#include <yarp/os/Time.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <yarp/dev/PolyDriver.h>

using namespace iCub::skinCalibrationPlanner;
using namespace iCub::skinDynLib;
using namespace iCub::iDyn;
using namespace iCub::ctrl;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::math;
using namespace std;


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
void iCub::skinCalibrationPlanner::computeNeighbors(const vector<Vector> &taxelPos, double maxNeighDist, vector< list<unsigned int> > &neighborsXtaxel)
{
    neighborsXtaxel.clear();
    neighborsXtaxel.resize(taxelPos.size(), list<unsigned int>(0));
    Vector v;
    double d2 = maxNeighDist*maxNeighDist;
    for(unsigned int i=0; i<taxelPos.size(); i++)
    {
        for(unsigned int j=i+1; j<taxelPos.size(); j++)
        {
            // if the taxel exists
            if(taxelPos[i][0]!=0.0 || taxelPos[i][1]!=0.0 || taxelPos[i][2]!=0.0)
            {  
                if(taxelPos[i].size() != taxelPos[j].size()){
                    printf("Taxel pos %d: %s\n", i, taxelPos[i].toString().c_str());
                    printf("Taxel pos %d: %s\n", j, taxelPos[j].toString().c_str());
                }
                v = taxelPos[i]-taxelPos[j];
                if( dot(v,v) <= d2)
                {
                    neighborsXtaxel[i].push_back(j);
                    neighborsXtaxel[j].push_back(i);
                }
            }
        }
    }
}
