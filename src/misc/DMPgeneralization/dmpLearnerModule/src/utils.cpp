/*
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include <yarp/os/Time.h>
#include <list>

#include "iCub/utils.h"
#include "iCub/module.h"

using namespace std;
using namespace yarp::os;

/**********************************************************/
bool ObjectPropertiesCollectorPort::checkConnection()
{
    return (this->getOutputCount()>0 ? true : false);
}
/**********************************************************/
std::vector<int32_t> ObjectPropertiesCollectorPort::get_all_actions_with_traj_endeff()
{
    std::vector<int32_t> result(0);
    Bottle cmdOpc;
    cmdOpc.addVocab(VOCAB3('a','s','k'));
    Bottle &conditions=cmdOpc.addList();
    Bottle &entity= conditions.addList();
    entity.addString("entity");
    entity.addString("==");
    entity.addString("action");
    conditions.addString("&&");
    Bottle &trajectory= conditions.addList();
    trajectory.addString("traj_endeff");
    Bottle bReply;
    if (!checkConnection())
         return result;
    this->write(cmdOpc, bReply);
    // [ack] ("id" (<num0> <num1> ...)) 
    if (bReply.size()<1 || bReply.get(0).asVocab()==Vocab::encode("nack") || !bReply.get(1).check("id"))
        return result;
    Bottle* listIds=bReply.get(1).asList()->find("id").asList();
    result.reserve(listIds->size());
    for (int idCt=0; idCt<listIds->size(); ++idCt)
    {
       result.push_back(listIds->get(idCt).asInt());   
    }
    return result;
}

/**********************************************************/
bool ObjectPropertiesCollectorPort::addActionDMP(int32_t id, dmp::DMP *dmpToAdd)
{
    Bottle cmdOpc;
    cmdOpc.addVocab(VOCAB3('s','e','t'));
    Bottle& newProp=cmdOpc.addList();
    Bottle& dmpId=newProp.addList();
    dmpId.addString("id");
    dmpId.addInt(id);
    Bottle& bDmpElement=newProp.addList();
    bDmpElement.addString("DMP");
    bDmpElement.addList()=dmpToAdd->toBottle(); //TODO check deallocate!!
        
    Bottle bReply;
    if (!checkConnection())
        return false;
    this->write(cmdOpc, bReply);
    return bReply.size()!=0 && bReply.get(0).asVocab()==Vocab::encode("ack");
}
/**********************************************************/
bool ObjectPropertiesCollectorPort::get_information_for(int32_t id)
{
    if (!learnerModule)
        return false;
    learnerModule-> resetCurrent();
    
    Bottle cmdOpc;
    cmdOpc.addVocab(VOCAB3('g','e','t'));
    Bottle &trajId=cmdOpc.addList();
    trajId.addString("id");
    trajId.addInt(id);
    Bottle bReply;
    if (!checkConnection())
        return false;
    this->write(cmdOpc, bReply);
    if (bReply.size()<2 || bReply.get(0).asVocab()==Vocab::encode("nack") || !bReply.get(1).check("entity") || bReply.get(1).find("entity").asString()!="action")
         return false;
    learnerModule->currentMutex.wait();
    bool foundSmt=false;
    Bottle* trajectory=bReply.get(1).find("traj_endeff").asList();
    if (trajectory!=NULL)
    {
        int noSamples=trajectory->size();
        if (noSamples>0)
        {
            foundSmt=true;
            Bottle* sample = trajectory->get(0).asList();
            int dof=sample->size()-1;    //last element is the timestamp
            learnerModule->dof=dof;            
            if (learnerModule->currentTrajectory)
                delete learnerModule->currentTrajectory;
            learnerModule->currentTrajectory= new dmp::Trajectory(dof, noSamples);
            
            for (int ct=0; ct<noSamples; ++ct)     
            {
                sample= trajectory->get(ct).asList();
                yarp::sig::Vector values(dof);
                for(int ctDof=0; ctDof<dof; ++ctDof)
                       values[ctDof]=sample->get(ctDof).asDouble(); // fist column is for time
                
                learnerModule->currentTrajectory->add_trajectory_point(sample->get(dof).asDouble(), values);
            }
        }
     }
     
     Bottle *targetPosition=bReply.get(1).find("targetPosition").asList();
     if(targetPosition)
     {
        foundSmt=true;
        yarp::sig::Vector& targetVector=learnerModule->currentTarget;
        targetVector.resize(targetPosition->size());
        for (int i=0; i<targetPosition->size(); ++i)
        {
          targetVector(i)=targetPosition->get(i).asDouble();
        }
     }
     if (bReply.get(1).check("name"))
         learnerModule->actionName=bReply.get(1).find("name").asString().c_str();
     
     
     Bottle* dmpBot =bReply.get(1).find("DMP").asList();
     if(dmpBot)
     {
         foundSmt=true;
         if (learnerModule->currentDMP)
            learnerModule->currentDMP->fromBottle(*dmpBot);

     }
     learnerModule->currentMutex.post();
     return true;     
}