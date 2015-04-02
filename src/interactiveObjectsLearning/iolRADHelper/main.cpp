/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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
\defgroup icub_iolRADHelper Helper funtions to assist RAD
@ingroup icub_interactiveObjectsLearning
 
A container collecting useful functions that help RAD 
accomplishing its tasks.

\section proto_sec Protocol 
Available requests queried through the rpc port: 
-# <b>NAME</b>. The format is: [name] "name_rad_0" "name_rad_1" 
   ... This request allows merging the list provided by RAD
   within the request (i.e. "name_rad_0" "name_rad_1" ...) with
   further objects names stored within the
   objectsPropertiesCollector database. \n
   The format of the reply is: [nack]/[ack] "name_rad_0"
   "name_rad_1" ... "name_opc_0" "name_opc_1" ...
-# <b>SET NAV LOCATION</b>. The format is: [navs] 
   "location_name" x y theta. This request allows creating a
   location within the database. \n The format of the reply is:
   [nack]/[ack].
-# <b>GET NAV LOCATION</b>. The format is: [navg] 
   "location_name". This request allows retrieving a location
   from the database. \n The format of the reply is:
   [nack]/[ack] x y theta.
 
\section lib_sec Libraries 
- YARP libraries

\section parameters_sec Parameters
--name <string> 
- To specify the module's name; all the open ports will be 
  tagged with the prefix /<moduleName>/. If not specified
  \e iolRADHelper is assumed.
 
\section portsc_sec Ports Created
- \e /<moduleName>/rpc to be accessed by RAD. 
 
- \e /<moduleName>/opc to be connected to the 
  objectsPropertiesCollector port.
 
\section tested_os_sec Tested OS
Linux and Windows.

\author Ugo Pattacini
*/ 

#include <yarp/os/all.h>

#include <stdio.h>
#include <string>

using namespace yarp::os;
using namespace std;


/************************************************************************/
class iolRADHelperModule: public RFModule
{
    RpcClient opcPort;
    Port      rpcPort;

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();
        opcPort.open(("/"+name+"/opc").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);
        return true;
    }

    /************************************************************************/
    bool close()
    {
        opcPort.close();
        rpcPort.close();
        return true;
    }

    /************************************************************************/
    bool getNames(Bottle &names)
    {
        names.clear();
        if (opcPort.getOutputCount()>0)
        {
            Bottle opcCmd,opcReply,opcReplyProp;
            opcCmd.addVocab(Vocab::encode("ask"));
            Bottle &content=opcCmd.addList().addList();
            content.addString("entity");
            content.addString("==");
            content.addString("object");
            opcPort.write(opcCmd,opcReply);
            
            if (opcReply.size()>1)
            {
                if (opcReply.get(0).asVocab()==Vocab::encode("ack"))
                {
                    if (Bottle *idField=opcReply.get(1).asList())
                    {
                        if (Bottle *idValues=idField->get(1).asList())
                        {
                            // cycle over items
                            for (int i=0; i<idValues->size(); i++)
                            {
                                int id=idValues->get(i).asInt();

                                // get the relevant properties
                                // [get] (("id" <num>) ("propSet" ("name")))
                                opcCmd.clear();
                                opcCmd.addVocab(Vocab::encode("get"));
                                Bottle &content=opcCmd.addList();
                                Bottle &list_bid=content.addList();
                                list_bid.addString("id");
                                list_bid.addInt(id);
                                Bottle &list_propSet=content.addList();
                                list_propSet.addString("propSet");
                                list_propSet.addList().addString("name");
                                opcPort.write(opcCmd,opcReplyProp);

                                // append the name (if any)
                                if (opcReplyProp.get(0).asVocab()==Vocab::encode("ack"))
                                    if (Bottle *propField=opcReplyProp.get(1).asList())
                                        if (propField->check("name"))
                                            names.addString(propField->find("name").asString().c_str());
                            }
                        }
                    }
                }
            }

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    int getLocation(const Bottle &body, Bottle &location)
    {
        int id=-1;
        location.clear();        
        if ((opcPort.getOutputCount()>0) && (body.size()>0))
        {
            Bottle opcCmd,opcReply,opcReplyProp;
            opcCmd.addVocab(Vocab::encode("ask"));
            Bottle &content=opcCmd.addList();
            Bottle &cond1=content.addList();
            cond1.addString("entity");
            cond1.addString("==");
            cond1.addString("navloc");
            content.addString("&&");
            Bottle &cond2=content.addList();
            cond2.addString("name");
            cond2.addString("==");
            cond2.addString(body.get(0).asString().c_str());
            opcPort.write(opcCmd,opcReply);

            if (opcReply.size()>1)
            {
                if (opcReply.get(0).asVocab()==Vocab::encode("ack"))
                {
                    if (Bottle *idField=opcReply.get(1).asList())
                    {
                        if (Bottle *idValues=idField->get(1).asList())
                        {
                            if (idValues->size()>0)
                            {                            
                                // consider just the first element
                                id=idValues->get(0).asInt();

                                // get the relevant properties
                                // [get] (("id" <num>) ("propSet" ("location")))
                                opcCmd.clear();
                                opcCmd.addVocab(Vocab::encode("get"));
                                Bottle &content=opcCmd.addList();
                                Bottle &list_bid=content.addList();
                                list_bid.addString("id");
                                list_bid.addInt(id);
                                Bottle &list_propSet=content.addList();
                                list_propSet.addString("propSet");
                                list_propSet.addList().addString("location");
                                opcPort.write(opcCmd,opcReplyProp);

                                // append the name (if any)
                                if (opcReplyProp.get(0).asVocab()==Vocab::encode("ack"))
                                    if (Bottle *propField=opcReplyProp.get(1).asList())
                                        if (propField->check("location"))
                                            if (Bottle *loc=propField->find("location").asList())
                                                for (int i=0; i<loc->size(); i++)
                                                    location.addDouble(loc->get(i).asDouble());
                            }
                        }
                    }
                }
            }
        }

        return id;
    }

    /************************************************************************/
    bool setLocation(const int id, const Bottle &body)
    {
        if ((opcPort.getOutputCount()>0) && (body.size()>3))
        {
            Bottle opcCmd,opcReply;
            Bottle *pContent=NULL;
            if (id<0)
            {
                opcCmd.addVocab(Vocab::encode("add"));
                Bottle &content=opcCmd.addList();
                Bottle &list_ent=content.addList();
                list_ent.addString("entity");
                list_ent.addString("navloc");
                Bottle &list_name=content.addList();
                list_name.addString("name");
                list_name.addString(body.get(0).asString().c_str());
                pContent=&content;
            }
            else
            {
                opcCmd.addVocab(Vocab::encode("set"));
                Bottle &content=opcCmd.addList();
                Bottle &list_id=content.addList();
                list_id.addString("id");
                list_id.addInt(id);
                pContent=&content;
            }

            Bottle &list_loc=pContent->addList();
            list_loc.addString("location");
            Bottle &list_data=list_loc.addList();
            list_data.addDouble(body.get(1).asDouble());
            list_data.addDouble(body.get(2).asDouble());
            list_data.addDouble(body.get(3).asDouble());
            opcPort.write(opcCmd,opcReply);

            return (opcReply.get(0).asVocab()==Vocab::encode("ack"));
        }
        else
            return false;
    }

    /************************************************************************/
    Bottle mergeList(const Bottle &b1, const Bottle &b2)
    {
        Bottle ret=b1;
        for (int i=0; i<b2.size(); i++)
        {
            string name=b2.get(i).asString().c_str();
            bool toBeAppended=true;

            for (int j=0; j<ret.size(); j++)
            {
                if (name==ret.get(j).asString().c_str())
                {
                    toBeAppended=false;
                    break;
                }
            }

            if (toBeAppended)
                ret.append(name.c_str());
        }

        return ret;
    }

    /************************************************************************/
    bool respond(const Bottle &cmd, Bottle &reply)
    {
        switch (cmd.get(0).asVocab())
        {
            //-----------------
            case VOCAB4('n','a','m','e'):
            {
                Bottle names;
                if (getNames(names))
                {
                    reply.addString("ack");
                    reply.append(mergeList(cmd.tail(),names));
                }
                else
                    reply.addString("nack");

                return true;
            }

            //-----------------
            case VOCAB4('n','a','v','g'):
            {
                Bottle location;
                if (getLocation(cmd.tail(),location)>=0)
                {
                    reply.addString("ack");
                    reply.append(location);
                }
                else
                    reply.addString("nack");

                return true;
            }

            //-----------------
            case VOCAB4('n','a','v','s'):
            {
                Bottle location;
                Bottle body=cmd.tail();
                int id=getLocation(body,location);
                if (setLocation(id,body))
                    reply.addString("ack");
                else
                    reply.addString("nack");

                return true;
            }

        //-----------------
            default:
                return RFModule::respond(cmd,reply);
        }

        reply.addString("nack");
        return false;
    }

    /************************************************************************/
    bool updateModule()
    {        
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 1.0;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefault("name","iolRADHelper");
    rf.configure("ICUB_ROOT",argc,argv);

    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    iolRADHelperModule module;
    return module.runModule(rf);
}



