/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Andrea Del Prete
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

#ifndef SCP_UTIL_H
#define SCP_UTIL_H

#include <vector>
#include <list>
#include <sstream>
#include <yarp/sig/Matrix.h>
#include <yarp/os/Semaphore.h>
#include <ace/Recursive_Thread_Mutex.h>
#include <ace/Thread.h>
#include <iCub/ctrl/math.h>
#include <iCub/iDyn/iDyn.h>
#include "iCub/skinDynLib/common.h"

using namespace yarp::math;

namespace iCub
{

namespace skinCalibrationPlanner
{

    struct CtrlRef
    {
        yarp::sig::Vector q;            // desired joint angles (deg)
        yarp::sig::Vector x;            // desired cartesian position of control point (m)
		yarp::sig::Vector xEnv;			// cartesian position of desired contact w.r.t. link reference frame
        unsigned int xLink;             // link number of the desired contact point
        yarp::sig::Vector f;            // desired cartesian force of contact point (N)
        int taxelId;                    // index of the taxel chosen as destination
    };

    class SignalStabilityAnalyzer
    {
        unsigned int N;
        unsigned int listSize;
        std::deque<yarp::sig::Vector> xList;
        yarp::sig::Vector xMean;
        double timeWindow;
        double period;
		double stabilityMargin;
		double maxDeviation;
        bool isStable;

    public:
        SignalStabilityAnalyzer(double _timeWindow, double _period, double _stabilityMargin, int _signalSize)
            :timeWindow(_timeWindow), period(_period), stabilityMargin(_stabilityMargin)
        {
			N = _signalSize;
            maxDeviation = 0.0;
			xMean = yarp::math::zeros(N);
            listSize = std::max<unsigned int>((unsigned int)(_timeWindow/_period), 1);
            isStable = false;
        }

        bool addSample(const yarp::sig::Vector &x)
        {
            // update mean
            if(x.size()!=N)
            {
                printf("[SignalStabilityAnalyzer::addSample] Unexpected dimension of x: %d!=%d\n", x.size(), N);
                return false;
            }

			if(xList.size() < listSize){
				xMean += (x-xMean)/(xList.size()+1);
				xList.push_back(x);
				isStable = false;
				return isStable;
			}
			else{
				xMean += (x-xList[0])/(listSize);
				xList.push_back(x);
				xList.pop_front();
			}
            
            // update max deviation
            maxDeviation = 0.0;
            yarp::sig::Vector d;
            for(std::deque<yarp::sig::Vector>::const_iterator it=xList.begin(); it!=xList.end(); it++)
            {
                d = (*it) - xMean;
				if(norm(d)> maxDeviation)
					maxDeviation = norm(d);
            }
			
			isStable = stabilityMargin > maxDeviation;
            return isStable;
        }

		void reset(){
			maxDeviation = 0.0;
			xList.clear();
			xMean = yarp::math::zeros(N);
            isStable = false;
		}

        bool isSignalStable(){ return isStable; }
    };

    class ControllerImprovementAnalyzer
    {
        double timeWindow;
        double period;
		double improvementMargin;
        double minError;
        unsigned int timer;

    public:
        ControllerImprovementAnalyzer(double _timeWindow, double _period, double _improvementMargin)
            :timeWindow(_timeWindow), period(_period), improvementMargin(_improvementMargin)
        {
            reset();
        }

        bool addSample(const yarp::sig::Vector &err)
        {
            double errNorm = yarp::math::norm(err);
            if(errNorm < minError-improvementMargin)
            {
                minError = errNorm;
                timer = (unsigned int)(timeWindow/period);
            }
            else if(timer>0)
                timer--;
			
            return (timer!=0);
        }

		void reset(){
			timer = (unsigned int)(timeWindow/period);
            minError = DBL_MAX;
		}

        bool setTimeWindow(double t){
            if(t<=0.0)
                return false;
            timeWindow = t;
            return true;
        }

        bool isImproving(){ return (timer!=0); }

        double getRemainingTime(){ return timer*period; }
    };

    void computeNeighbors(const std::vector<yarp::sig::Vector> &taxelPos, double maxNeighDist, std::vector< std::list<unsigned int> > &neighborsXtaxel);

}

} // end namespace

#endif

