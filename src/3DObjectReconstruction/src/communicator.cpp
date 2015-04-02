#include "communicator.h"
#include <stdio.h>
using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

Communicator::Communicator()
{
    isOpen=false;
}

Communicator::~Communicator()
{
    if (isOpen)
        close();
}

bool Communicator::open(const Property &options)
{
    Property &opt=const_cast<Property&>(options);

    string milPortName=opt.check("milPortName",Value("/objectReconstr/mil")).asString().c_str();
    string opcPortName=opt.check("opcPortName",Value("/objectReconstr/opc")).asString().c_str();
    string milPortRemote=opt.check("milPortRemote",Value("/iolStateMachineHandler/human:rpc")).asString().c_str();
    string opcPortRemote=opt.check("opcPortRemote",Value("/memory/rpc")).asString().c_str();

    mil.open(milPortName.c_str());
    opc.open(opcPortName.c_str());

    bool ok=Network::connect(opcPortName.c_str(),opcPortRemote.c_str());
    ok=ok&Network::connect(milPortName.c_str(),milPortRemote.c_str());

    return ok;
}

void Communicator::close()
{
    if (isOpen)
    {
        mil.interrupt();
        opc.interrupt();
        mil.close();
        opc.close();
        isOpen=false;
    }
}

bool Communicator::disableAttention()
{
    if (checkPorts())
    {
        Bottle b,reply;
        b.addString("attention");
        b.addString("stop");
        fprintf(stdout, "%s\n", b.toString().c_str());
        return true;
        mil.write(b,reply);

        if (reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            fprintf(stdout, "Attention stopped\n");
            return true;
        }
    }

    return false;
}

bool Communicator::enableAttention()
{
    Bottle b,reply;
    b.addString("attention");
    b.addString("start");
    mil.write(b,reply);

    if (reply.get(0).asVocab()==Vocab::encode("ack"))
    {
        fprintf(stdout, "Attention started\n");
        return true;
    }

    return false;
}

bool Communicator::retrieveTableHeight(double &tableHeight)
{
    if (checkPorts())
    {
        Bottle b,reply;
        b.addVocab(Vocab::encode("ask"));
        Bottle& sublist=b.addList();
        Bottle& bEntity=sublist.addList();
        bEntity.addString("entity");
        bEntity.addString("==");
        bEntity.addString("table");

        int id=-1;
        opc.write(b,reply);
        if (reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            Bottle* list=reply.get(1).asList();
            if (list->get(0).asString()=="id")
            {
                Bottle* ids=list->get(1).asList();
                if (ids->size()>0)
                    id=ids->get(0).asInt();
                else
                    return false;
            }
            else 
                return false;
        }
        else 
            return false;

        b.clear();
        reply.clear();

        b.addVocab(Vocab::encode("get"));
        Bottle &bsublist=b.addList();
        Bottle &bID=bsublist.addList();
        bID.addString("id");
        bID.addInt(id);
        Bottle &bProp=bsublist.addList();
        bProp.addString("propSet");
        Bottle &properties=bProp.addList();
        properties.addString("height");

        opc.write(b,reply);

        if (reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            Bottle* list1=reply.get(1).asList();
            Bottle* list=list1->get(0).asList();
            if (list->get(0).asString()=="height")
                tableHeight=list->get(1).asDouble();
            else 
                return false;
        }
        else 
            return false;

        return true;
    }

    return false;
}

bool Communicator::retrieveBoundingBox(const string &object, yarp::sig::Vector &leftTopVertex, yarp::sig::Vector &rightDownVertex)
{
    leftTopVertex.resize(2,0.0);
    rightDownVertex.resize(2,0.0);

    if (checkPorts())
    {
        Bottle b,reply;
        b.addVocab(Vocab::encode("ask"));
        Bottle& sublist=b.addList();
        Bottle& bName=sublist.addList();
        bName.addString("name");
        bName.addString("==");
        bName.addString(object.c_str());
        sublist.addString("&&");
        Bottle& bEntity=sublist.addList();
        bEntity.addString("entity");
        bEntity.addString("==");
        bEntity.addString("object");

        int id=-1;
        opc.write(b,reply);
        if (reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            Bottle* list=reply.get(1).asList();
            if (list->get(0).asString()=="id")
            {
                Bottle* ids=list->get(1).asList();
                if (ids->size()>0)
                    id=ids->get(0).asInt();
                else
                    return false;
            }
            else 
                return false;
        }
        else 
            return false;

        b.clear();
        reply.clear();
        
        b.addVocab(Vocab::encode("get"));
        Bottle &bsublist=b.addList();
        Bottle &bID=bsublist.addList();
        bID.addString("id");
        bID.addInt(id);
        Bottle &bProp=bsublist.addList();
        bProp.addString("propSet");
        Bottle &properties=bProp.addList();
        properties.addString("position_2d_left");

        opc.write(b,reply);
        if (reply.get(0).asVocab()==Vocab::encode("ack"))
        {
            Bottle* list1=reply.get(1).asList();
            Bottle* list=list1->get(0).asList();
            Bottle* list2=list->get(1).asList();
            if (list->get(0).asString()=="position_2d_left" && list2->size()==4)
            {
                leftTopVertex[0]=list2->get(0).asInt();
                leftTopVertex[1]=list2->get(1).asInt();
                rightDownVertex[0]=list2->get(2).asInt();
                rightDownVertex[1]=list2->get(3).asInt();
            }
            else 
                return false;
        }
        else 
            return false;

        return true;
    }

    return false;
}

bool Communicator::checkPorts()
{
    return (mil.getOutputCount()>0) && (opc.getOutputCount()>0);
}



