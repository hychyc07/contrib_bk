/*
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Ugo Pattacini
 * email:  vadim.tikhanoff@iit.it
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

#include "iCub/utils.h"
#include "iCub/module.h"

#include <yarp/sig/Matrix.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

/**********************************************************/
ParticleFilter::ParticleFilter() 
{
    useCallback();
}
/**********************************************************/
void ParticleFilter::onRead(Bottle &target)
{
    //CvPoint trackLoc;
    if (target.size()>1)
    {
        loc.x=(int)target.get(0).asDouble();
        loc.y=(int)target.get(1).asDouble();
        //fprintf( stdout, "particle something: %s \n\n",target.toString().c_str() );
    }
}
/**********************************************************/
bool ParticleFilter::getTraker(CvPoint &loc)
{
    loc=this->loc;
    return true;
}
/**********************************************************/
void SegmentationPoint::segment(Bottle &b)
{
    if (getOutputCount()>0)
    {
        //send 2D x y coordinates to segmentator
        Bottle request;
        request.addDouble(b.get(0).asDouble());
        request.addDouble(b.get(1).asDouble());
        write(request);
    }
}
/**********************************************************/
void PointedLocation::onRead(Bottle &b)
{
    fprintf(stdout, "got read from points size %d \n",b.size());
    if (b.size()>1)
    {
        loc.x=(int)b.get(0).asDouble();
        loc.y=(int)b.get(1).asDouble();
        rxTime=Time::now();
    }
}
/**********************************************************/
PointedLocation::PointedLocation()
{
    useCallback();
    rxTime=-1.0;
    timeout=2.0;
}
/**********************************************************/
bool PointedLocation::getLoc(CvPoint &loc)
{
    double t0=Time::now();

    if ((rxTime>0.0) && (t0-rxTime<timeout))
    {
        loc=this->loc;
        return true;
    }

    while (Time::now()-t0<timeout)
    {
        if ((rxTime>0.0) && (Time::now()-rxTime<timeout))
        {
            loc=this->loc;
            return true;
        }
        Time::delay(0.1);
    }
    return false;
}

/**********************************************************/
bool ObjectPropertiesCollectorPort::checkConnection()
{
    return (this->getOutputCount()>0 ? true : false);
}
/**********************************************************/
 bool ObjectPropertiesCollectorPort::addActionTarget(int id, yarp::sig::Vector
 targetPosition)
{
     Bottle cmdOpc;    
//     bool getStereoPosition(const std::string &obj_name, yarp::sig::Vector &stereo);
     cmdOpc.addVocab(VOCAB3('s','e','t'));
     Bottle &target=cmdOpc.addList();
     Bottle &identifier=target.addList();
     identifier.addString("id");
     identifier.addInt(id);
     Bottle &newProperty=target.addList();
     newProperty.addString("targetPosition");
     Bottle &newPosition=newProperty.addList();
     for (size_t i=0; i<targetPosition.length(); ++i)
         newPosition.addDouble(targetPosition(i));
     
     Bottle bReply;
     
     if (!checkConnection())
         return false;
     this->write(cmdOpc, bReply);
     
     return bReply.size()!=0 && bReply.get(0).asVocab()==Vocab::encode("ack");
  
}
/**********************************************************/
int ObjectPropertiesCollectorPort::createActionTarget(std::string actionName, yarp::sig::Vector targetPosition)
{
    Bottle cmdOpc;
    cmdOpc.addVocab(VOCAB3('a','d','d'));
    Bottle &target=cmdOpc.addList();
    Bottle &entity=target.addList();
    entity.addString("entity");
    entity.addString("action");
    Bottle &newProperty=target.addList();
    newProperty.addString("targetPosition");
    Bottle &newPosition=newProperty.addList();
    for (size_t i=0; i<targetPosition.length(); ++i)
        newPosition.addDouble(targetPosition(i));
    Bottle &name=target.addList();
    name.addString("name");
    name.addString(actionName.c_str());
    
    Bottle bReply;
    if (!checkConnection())
        return false;
    this->write(cmdOpc, bReply);
     
    if(bReply.size()==0 || bReply.get(0).asVocab()==Vocab::encode("nack") || !bReply.check("id"))
        return -1;
    return bReply.find("id").asInt();
}
/**********************************************************/
bool ObjectPropertiesCollectorPort::get2DPositionFromMemory(const string &object, Vector &position)
{
    int id=-1;
    Bottle cmdOpc;
    cmdOpc.addVocab(VOCAB3('a','s','k'));
    Bottle &conditions=cmdOpc.addList();
    Bottle &entity= conditions.addList();
    entity.addString("entity");
    entity.addString("==");
    entity.addString("object");
    conditions.addString("&&");
    Bottle &name= conditions.addList();
    name.addString("name");
    name.addString("==");
    name.addString(object.c_str());
    Bottle bReply;
    if (!checkConnection())
        return false;
    this->write(cmdOpc, bReply);
    // [ack] ("id" (<num0> <num1> ...)) 
    
    if (bReply.size()<1 || bReply.get(0).asVocab()==Vocab::encode("nack") || !bReply.check("id"))
        return false;
    else
        id = bReply.find("id").asList()->get(0).asInt();
    
    cmdOpc.clear();
    cmdOpc.addVocab(VOCAB3('g','e','t'));
    Bottle &trajId=cmdOpc.addList();
    trajId.addString("id");
    trajId.addInt(id);
    bReply.clear();
    if (!checkConnection())
        return false;
    this->write(cmdOpc, bReply);

    if (bReply.size()<1 || bReply.get(0).asVocab()==Vocab::encode("nack"))
        return false;

    //Bottle *pos = bReply.find("position_2d").asList();
    Bottle *pos = bReply.get(1).asList()->find("position_2d_left").asList(); 
    if (pos == NULL)
        return false;
    position.resize(2);
    position[0] = pos->get(0).asDouble();
    position[1] = pos->get(1).asDouble();
        
    return true;
}

// dmp::DMP* ObjectPropertiesCollectorPort::get_information_for(int32_t id)
// {
//     yarp::sig::Vector targetVector;
//     yarp::sig::Matrix trajectoryMatrix;
//     dmp::DMP* myDmp=NULL;
//     
//     Bottle cmdOpc;
//     cmdOpc.addVocab(VOCAB3('g','e','t'));
//     Bottle &trajId=cmdOpc.addList();
//     trajId.addString("id");
//     trajId.addInt(id);
//     Bottle bReply;
//     if (!checkConnection())
//         return myDmp;
//     this->write(cmdOpc, bReply);
//     if (bReply.size()<1 || bReply.get(0).asVocab()==Vocab::encode("nack") || !bReply.check("id") || !bReply.check("entity") || bReply.find("entity").asString()!="action")
//          return myDmp;
//     Bottle* trajectory=bReply.find("traj_endeff").asList();
//     if (trajectory!=NULL)
//     {
//         int noSamples=trajectory->size();
//         if (noSamples>0)
//         {
//             Bottle* sample = trajectory->get(0).asList();
//             int dof=sample->size()-1;    //last element is the timestamp
//             
//             trajectoryMatrix.resize(noSamples, dof+1);
//             for (int ct=0; ct<noSamples; ++ct)     
//             {
//                 sample= trajectory->get(ct).asList();
//                 for(int ctDof=0; ctDof<dof; ++ctDof)
//                 {  
//                     trajectoryMatrix[ct][ctDof+1]=sample->get(ctDof).asDouble(); // fist column is for time
//                 }
//                     
//                 trajectoryMatrix[ct][0]=sample->get(dof).asDouble();          
//             }
//         }
//      }
//      
//      Bottle *targetPosition=bReply.find("targetPosition").asList();
//      if(targetPosition)
//      {
//         targetVector.resize(targetPosition->size());
//         for (int i=0; i<targetPosition->size(); ++i)
//         {
//           targetVector(i)=targetPosition->get(i).asDouble();
//         }
//      }
//      
// //      if (bReply.check("name"))
// //          learnerModule->actionName=bReply.find("name").asString().c_str();
// //      
//      Bottle* dmp =bReply.find("DMP").asList();
//      if(dmp)
//      {
//          int N=dmp->find("N").asInt();
//          int dof=dmp->find("dof").asInt();
//          double alpha_x=dmp->find("alphax").asDouble();
//          double alpha_z=dmp->find("alphaz").asDouble();
//          double beta_z=dmp->find("betaz").asDouble();
//          myDmp=new DMPstructure(dof,N,alpha_x,alpha_z,beta_z);
//          
//          double tau=dmp->find("tau").asDouble();
//          myDmp->set_tau(tau);
//          Bottle* weights=dmp->find("weights").asList(); //they are serialized row by row
//          if (weights)
//          {
//              for (int i1=0; i1<weights->size(); ++i1)
//              {
//                  Bottle* weightRow=weights->get(i1).asList();
//                  if (weightRow)
//                  {
//                      yarp::sig::Vector weightRowVector(weightRow->size());
//                      for (int i2=0; i2<weightRow->size(); ++i2)
//                          weightRowVector(i2)=weightRow->get(i2).asDouble();
//                      myDmp->set_weights(i1, weightRowVector);
//                  }
//             }
//          }
// 
//          Bottle* goal=dmp->find("goal").asList();
//          if(goal)
//          {
//              yarp::sig::Vector goalVector(goal->size());
//              for(int i1=0; i1<goal->size(); ++i1)
//                  goalVector(i1)=goal->get(i1).asDouble();
//              myDmp->set_goal(goalVector);
//          }
//      }
//      
//   return *myDmp;
//          
// }