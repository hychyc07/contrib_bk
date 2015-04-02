/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Bjoern Browatzki
 * email:   bjoern.browatzki@tuebingen.mpg.de
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

#ifndef __RECOGVTM_THREAD_H__
#define __RECOGVTM_THREAD_H__

#include "iCub/Vtm.h"
#include "iCub/VtmThread.h"

class LearnModule;

class RecogVtmThread : public VtmThread
{
public:
    enum RecogState
    {
        REC_EXPLORE,
        REC_CHECKCANDIDATE
    };

    RecogVtmThread( 
            ObjRecModule *module_,
            int period,
            PolyDriver &robotArm_,
            ResourceFinder *rf_) :
        VtmThread(module_, period, robotArm_, rf_)
    {
    }

    virtual bool threadInit();     
    virtual void run(); 

    void setResultDir(const std::string &dir)
    {
        vtmResultDir = dir;
    }

private:
    
    double REC_CANDIDATE_THRESHOLD;
    double REC_CONFIRM_THRESHOLD;

    std::vector<Vtm> loadVtms(const std::string &vtmDir);

    void printDissimilarities() const; 
    bool findCandidateKeyframe(const cv::Mat &currentKeyframe, int candVtmNo = -1); // -1 -> search through all vtms
    void runVtmComparisions();
    double compareVtms(const Vtm &vtm1, const Vtm &vtm2) const;
    void saveResults();
    void restart();

    void showImages() const;

    std::string vtmResultDir;
    int maxIterations;
    int maxTrials;
    int recogIteration;
    int trialNo;
    RecogState recogState;
    Vector targetState;
    int targetVtmNo;
    int targetViewNo;
    std::vector<Vtm> vtmList;
    std::vector<std::vector<double> > objectDissimilarities;
    // images to display
    cv::Mat imgCurrentView;
    cv::Mat imgTargetView;
    cv::Mat imgMatchedView;
    ////////////////////////////////////////
};

#endif

