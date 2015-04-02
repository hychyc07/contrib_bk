/*
 * Copyright (C) 2012 Istituto Italiano di Tecnologia
 * Author: Elena Ceseracciu
 * email:  elena.ceseracciu@iit.it
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
#include <sstream>
#include <stdio.h>
#include <yarp/math/Rand.h>
#include <yarp/math/Math.h>
#include "iCub/module.h"
#include "iCub/DMP/DMPPointToPoint.h"
#include "iCub/DMP/DMPPeriodic.h"
#include <gsl/gsl_math.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;

#define RET_INVALID     -1

#define CMD_TRAIN               VOCAB4('t','r','a','i')
#define CMD_EXECUTE             VOCAB4('e','x','e','c')
#define CMD_TOOLEXTEND          VOCAB4('e','x','t','d')

/************************************a**********************/
bool DmpLearner::attach(yarp::os::Port &source)
{
    return this->yarp().attachAsServer(source);
}
/**********************************************************/
void DmpLearner::resetCurrent()
{
    currentMutex.wait();
    if(currentTrajectory)
        delete currentTrajectory;

    currentTrajectory=NULL;

    actionName="";
    currentTarget.clear();
    currentMutex.post();
}
/**********************************************************/
void DmpLearner::resetMap()
{
    for (std::map<std::string, DmpGPR*>::iterator mapIt=learnersMap.begin(); mapIt!=learnersMap.end(); ++mapIt)
    {
        if(mapIt->second)
            delete mapIt->second;
    }
    learnersMap.clear();
}
/**********************************************************/
bool DmpLearner::sync_opc()
{
    std::vector<int32_t> actionIds=opcPort.get_all_actions_with_traj_endeff();
    bool ok= true;
    for (size_t actCt=0; actCt<actionIds.size(); ++actCt)
    {
        ok= estimate_DMP(actionIds.at(actCt)) && ok;
    }
    return ok;
}
/**********************************************************/
bool DmpLearner::estimate_DMP(const int32_t id)
{
   
    if (!opcPort.get_information_for(id))
        return false;
  //  std::cout << "got info from opc" <<std::endl;
    if (currentTrajectory && currentTrajectory->get_number_of_samples()==0) // TODO check what other checks are needed
        return false;
  // std::cout << "checked traj" <<std::endl;
    if (currentDMP)
    {
        delete currentDMP;
   // std::cout << "deleted current dmp" <<std::endl;
    }
    if(dmpType=="DMPPointToPoint")
        currentDMP=new DMPPointToPoint(dof, N, alpha_x, alpha_z, beta_z);
    else if (dmpType=="DMPPeriodic")
        currentDMP=new DMPPeriodic(dof, N, alpha_z, beta_z);
    else return false;
   // std::cout << "about to estimate" <<std::endl; fflush(stdout);
    if (!currentDMP->estimate_from_trajectory(*currentTrajectory))
        return false;
   // std::cout << "estimated" <<std::endl; fflush(stdout);
    if (learnersMap.find(actionName)==learnersMap.end())
        learnersMap[actionName]=new DmpGPR(N, dof, currentTarget.size());
   // std::cout << "about to feed sample" <<std::endl;
    learnersMap.at(actionName)->feedSample(currentTarget, currentDMP);
    std::cout << *currentDMP <<std::endl;
    return opcPort.addActionDMP(id, currentDMP);
    //newDMP.print(); //can be commented out...
}
/**********************************************************/
bool DmpLearner::train_ids(const std::vector<int32_t> & trainInputIds)
{
    //resetMap();
    if (learnersMap.find("CUSTOM")!=learnersMap.end())
        delete learnersMap.at("CUSTOM");
    opcPort.get_information_for(trainInputIds.at(0));
  //  std::cout << "current target size " << currentTarget.size() <<std::endl;
    learnersMap["CUSTOM"]=new DmpGPR(N, dof, currentTarget.size());
    bool ok=true;
    for (size_t actCt=0; actCt<trainInputIds.size(); ++actCt)
    {
        ok= opcPort.get_information_for(trainInputIds.at(actCt)) && ok;
        currentMutex.wait();
        if (currentDMP)
            std::cout << *currentDMP;
        else 
        {
            currentMutex.post();
            continue;
        }
     //   if(ok && currentDMP->isValid() )
        {
    //        std::cout << "dmp valid, proceeding" <<std::endl; fflush(stdout);
            learnersMap.at("CUSTOM")->feedSample(currentTarget, currentDMP);
        }
        currentMutex.post();
    }
    ok= ok && learnersMap.at("CUSTOM")->inference();
    return ok;
}
/**********************************************************/
bool DmpLearner::train_action(const std::string& actionName)
{
    if (learnersMap.find(actionName)==learnersMap.end())
        return false;
    else 
    {
        return learnersMap.at(actionName)->inference();
    }
}
/**********************************************************/
bool DmpLearner::generalize_DMP(const int32_t id)
{
    if (!opcPort.get_information_for(id))
        return false;
    if ( actionName== "" || learnersMap.find(actionName)==learnersMap.end())
        return false;
    std::cout << "map for " << actionName << " has " << learnersMap.at(actionName)->getNumberOfSamples() << "samples" << std::endl;
   // TEMP TODO check if inference can be removed, or done only some times
    
    if (!learnersMap.at(actionName)->inference())
        return false;
    dmp::DMP* newDMP= learnersMap.at(actionName)->generalize(currentTarget);
     return opcPort.addActionDMP(id, newDMP);
}
/**********************************************************/
void DmpLearner::set_num_basis_functions(const int32_t N)
{
    this->N=N;
    resetCurrent();
    resetMap();  
}
/**********************************************************/
void DmpLearner::set_alphax(const double alphax)
{
    this->alpha_x=alphax;
    resetCurrent();
    resetMap();     
}
/**********************************************************/
void DmpLearner::set_alphaz(const double alphaz)
{
    this->alpha_z=alphaz;
    resetCurrent();
    resetMap(); 
}
/**********************************************************/
void DmpLearner::set_betaz(const double betaz)
{
    this->beta_z=betaz;
    resetCurrent();
    resetMap();
}

/**********************************************************/
DmpLearner::DmpLearner()
{
    alpha_x=0.0;
    alpha_z=0.0;
    beta_z=0.0;
    N=0;
    dof=0;
    currentTrajectory=NULL;
    currentDMP=NULL;
}

/**********************************************************/
DmpLearner::~DmpLearner()
{
    resetCurrent();
    resetMap();
}

/**********************************************************/
bool DmpLearner::configure(ResourceFinder &rf)
{
    name=rf.find("name").asString().c_str();
    //incoming
    opcPort.open(("/"+name+"/opc:rpc").c_str());
    opcPort.setLearner(this);

    //outgoing
    N=rf.check("N", Value(27)).asInt();
    alpha_x= rf.check("alphax", Value(2.0)).asDouble();
    alpha_z=rf.check("alphaz", Value(12.0)).asDouble();
    beta_z=rf.check("betaz", Value(3.0)).asDouble();
    dmpType=rf.check("dmpType", Value("DMPPointToPoint")).asString().c_str();

    if(dmpType=="DMPPointToPoint")
        currentDMP=new DMPPointToPoint(dof, N, alpha_x, alpha_z, beta_z);
    else if (dmpType=="DMPPeriodic")
        currentDMP=new DMPPeriodic(dof, N, alpha_z, beta_z);
    
    
//std::cout << *currentDMP;
//std::cout << "number of dofs" 
    //rpc 
    thriftPort.open(("/"+name+"/rpc").c_str());   
    attach(thriftPort);

    return true;
}

/**********************************************************/
bool DmpLearner::interruptModule()
{
    thriftPort.interrupt();
    opcPort.interrupt();

    return true;
}
/**********************************************************/
bool DmpLearner::close()
{
    thriftPort.close();
    opcPort.close();

    return true;
}

/**********************************************************/
bool DmpLearner::updateModule()
{
    if (isStopping())
        return false;

    return true;
}

/**********************************************************/
double DmpLearner::getPeriod()
{
    return 0.1;
}

/**********************************************************/
void DmpLearner::quit()
{
    stopModule();
}
