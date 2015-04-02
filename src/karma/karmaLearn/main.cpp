/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
\defgroup karmaLearn Learning module of the KARMA Experiment
 
@ingroup icub_karma  
 
Machine Learning Module that allows the robot to learn online 
and predict a one-to-one map.

\section intro_sec Description 
This module belongs to the suite of KARMA software and is 
responsible for learning a generic map from \f$ R \f$ to \f$ R \f$. 
 
\section lib_sec Libraries 
- YARP libraries. 
- learningMachine library.

\section parameters_sec Parameters 
--context \e context
- To specify the module's context.
 
--from \e file
- To specify the module's configuration file.
 
\section portsc_sec Ports Created 
- \e /karmaLearn/rpc remote procedure call. \n 
    Recognized remote commands:
    - [train] "item" <in> <out>: issue the module to train
      internal machines with the input-output pair for the given
      item. The reply is [nack]/[ack].
    - [predict] "item" <in>: retrieve the predicted output for
      the specified input. The reply is [nack]/[ack] <out>
      <variance>, where the variance is meaningful only if not
      negative. In case the input command is of the form
      [predict] "item" (<in0> <in1> ...), the prediction is
      returned for the values contained in the list and the
      reply will look like [ack] (<out0> <out1> ...)
      (<variance0> <variance1> ...).
    - [span] "item" <step>: retrieve the predicted output for a
      given range of input specified by the parameter step. The
      reply is [nack]/[ack] (<out0> <out1> ...) (<variance0>
      <variance1> ...). The second parameter can be omitted and
      an internal default step size is used.
    - [optimize] "item" [<step>]|[(<val0> <val1> ...)]: for the
      specified item a search with the given step, or
      alternatively over the given list of input values, is
      performed in order to find out the input that elicits the
      maximum output. The reply is [nack]/[ack] <in> <out>. The
      second parameter can be omitted and an internal default
      step size is used.
    - [items]: retrieve the name of the items currently handled
      by the module.
    - [machine] "item": retrieve the content of the machine
      contained within the database that serves to represent the
      input-output relation of the given item.
    - [clear] "item": remove the item from the internal
      machines database. If no argument is given, clean up the
      whole database content. The reply is [nack]/[ack].
    - [save]: save the content of the internal machines database
      within the configuration file. The reply is [ack].
    - [plot] "item" <step>: stream out a yarp image displaying
      the map learned for the specified item as computed over
      the domain explored with the given step.
 
 - \e /karmaLearn/plot:o streams out images containing the
   learned maps.
 
\section conf_file_sec Configuration Files
The configuration file passed through the option \e --from
looks like as follows:
 
\code 
[general]
name      karmaLearn    // the module's stem-name
num_items 0             // the number of items 
in_lb     0.0           // scaler lower bound in
in_ub     360.0         // scaler upper bound in 
out_lb    0.0           // scaler lower bound out
out_ub    2.0           // scaler upper bound out
\endcode 

Moreover, once some learning has been carried out, the 
configuration file will be filled with the sections 
corresponding to the items. 
 
\section tested_os_sec Tested OS
Windows, Linux

\author Ugo Pattacini
*/ 

#include <stdio.h>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>

#include <gsl/gsl_math.h>
#include <cv.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/learningMachine/FixedRangeScaler.h>
#include <iCub/learningMachine/IMachineLearner.h>
#include <iCub/learningMachine/LSSVMLearner.h>

#define DEFAULT_STEP    1.0

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::learningmachine;


/************************************************************************/
class KarmaLearn: public RFModule
{
protected:
    FixedRangeScaler             scalerIn;
    FixedRangeScaler             scalerOut;
    map<string,IMachineLearner*> machines;

    string name;
    string configFileName;

    string plotItem;
    double plotStep;

    Semaphore mutex;
    RpcServer rpcPort;
    Port      plotPort;

    /************************************************************************/
    IMachineLearner *createLearner()
    {
        IMachineLearner *learner=new LSSVMLearner;
        LSSVMLearner *lssvm=dynamic_cast<LSSVMLearner*>(learner);
        lssvm->setDomainSize(1);
        lssvm->setCoDomainSize(1);
        lssvm->setC(100.0);
        lssvm->getKernel()->setGamma(10.0);

        return learner;
    }

    /************************************************************************/
    void extractMinMax(const Bottle &b, double &min, double &max)
    {
        min=scalerOut.getUpperBoundIn();
        max=scalerOut.getLowerBoundIn();

        for (int i=0; i<b.size(); i++)
        {
            double bi=b.get(i).asDouble();

            if (max<bi)
                max=bi;

            if (min>bi)
                min=bi;
        }
    }

    /************************************************************************/
    void train(const string &item, const double input, const double output)
    {
        IMachineLearner *learner;
        map<string,IMachineLearner*>::const_iterator itr=machines.find(item);
        if (itr==machines.end())
        {
            learner=createLearner();
            machines[item]=learner;
        }
        else
            learner=itr->second;

        Vector in(1,input),out(1,output);
        out[0]=std::min(out[0],scalerOut.getUpperBoundIn());

        in[0]=scalerIn.transform(in[0]);
        out[0]=scalerOut.transform(out[0]);

        learner->feedSample(in,out);
        learner->train();
    }

    /************************************************************************/
    bool predict(const string &item, const Bottle &input, Bottle &output,
                 Bottle &variance)
    {
        map<string,IMachineLearner*>::const_iterator itr=machines.find(item);
        if (itr!=machines.end())
        {
            output.clear();
            variance.clear();
            for (int i=0; i<input.size(); i++)
            {
                Vector in(1,input.get(i).asDouble());
                in[0]=scalerIn.transform(in[0]);

                IMachineLearner *learner=itr->second;
                Prediction prediction=learner->predict(in);

                Vector v=prediction.getPrediction();
                output.addDouble(scalerOut.unTransform(v[0]));

                if (prediction.hasVariance())
                {
                    Vector v=prediction.getVariance();
                    variance.addDouble(v[0]);
                }
                else
                    variance.addDouble(-1.0);
            }

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    bool optimize(const string &item, const Bottle &searchDomain, double &input,
                  double &output)
    {
        map<string,IMachineLearner*>::const_iterator itr=machines.find(item);
        if (itr!=machines.end())
        {
            IMachineLearner *learner=itr->second;
            
            input=scalerIn.getLowerBoundIn();
            double maxOut=scalerOut.getLowerBoundOut();

            Bottle bin,bout,bdummy;
            bin.addDouble(input);
            predict(item,bin,bout,bdummy);
            output=bout.get(0).asDouble();

            for (int i=0; i<searchDomain.size(); i++)
            {
                double val=searchDomain.get(i).asDouble();
                Vector in(1,scalerIn.transform(val));
                Prediction prediction=learner->predict(in);
                Vector v=prediction.getPrediction();

                if (v[0]>maxOut)
                {
                    input=val;
                    output=scalerOut.unTransform(v[0]);
                    maxOut=v[0];
                }
            }

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    Bottle items()
    {
        Bottle ret;
        for (map<string,IMachineLearner*>::const_iterator itr=machines.begin(); itr!=machines.end(); itr++)
            ret.addString(itr->first.c_str());

        return ret;
    }

    /************************************************************************/
    bool machineContent(const string &item, string &content)
    {
        map<string,IMachineLearner*>::iterator itr=machines.find(item);
        if (itr!=machines.end())
        {
            content=itr->second->toString().c_str();
            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    void clear()
    {
        for (map<string,IMachineLearner*>::const_iterator itr=machines.begin(); itr!=machines.end(); itr++)
            delete itr->second;

        machines.clear();
        plotItem="";
    }

    /************************************************************************/
    bool clear(const string &item)
    {
        map<string,IMachineLearner*>::iterator itr=machines.find(item);
        if (itr!=machines.end())
        {
            delete itr->second;
            machines.erase(itr);

            if (plotItem==item)
                plotItem="";

            return true;
        }
        else
            return false;
    }

    /************************************************************************/
    void save()
    {
        ofstream fout(configFileName.c_str());

        fout<<"[general]"<<endl;
        fout<<"name      "<<name<<endl;
        fout<<"num_items "<<machines.size()<<endl;
        fout<<"in_lb     "<<scalerIn.getLowerBoundIn()<<endl;
        fout<<"in_ub     "<<scalerIn.getUpperBoundIn()<<endl;
        fout<<"out_lb    "<<scalerOut.getLowerBoundIn()<<endl;
        fout<<"out_ub    "<<scalerOut.getUpperBoundIn()<<endl;
        fout<<endl;

        int i=0;
        for (map<string,IMachineLearner*>::const_iterator itr=machines.begin(); itr!=machines.end(); itr++, i++)
        {
            fout<<"[item_"<<i<<"]"<<endl;
            fout<<"name    "<<itr->first<<endl;
            fout<<"learner "<<("("+string(itr->second->toString().c_str())+")").c_str()<<endl;
            fout<<endl;
        }

        fout.close();
    }

    /************************************************************************/
    void plot()
    {
        if ((plotItem!="") && (plotPort.getOutputCount()>0))
        {
            Bottle input,output,variance;
            for (double d=scalerIn.getLowerBoundIn(); d<scalerIn.getUpperBoundIn(); d+=plotStep)
                input.addDouble(d);

            if (predict(plotItem,input,output,variance))
            {
                ImageOf<PixelMono> img; img.resize(320,240);
                for (int x=0; x<img.width(); x++)
                    for (int y=0; y<img.height(); y++)
                        img(x,y)=255;

                CvFont font; cvInitFont(&font,CV_FONT_HERSHEY_SIMPLEX,0.5,0.5,0,1);
                cvPutText(img.getIplImage(),plotItem.c_str(),cvPoint(250,20),&font,cvScalar(0));

                double x_min=scalerIn.getLowerBoundIn();
                double x_max=scalerIn.getUpperBoundIn();
                double x_range=x_max-x_min;

                double y_min,y_max;
                extractMinMax(output,y_min,y_max);
                y_min*=(y_min>0.0?0.8:1.2);
                y_max*=(y_max>0.0?1.2:0.8);
                double y_range=y_max-y_min;
                
                {
                    ostringstream tag; tag.precision(3);
                    tag<<x_min;
                    cvPutText(img.getIplImage(),tag.str().c_str(),cvPoint(10,230),&font,cvScalar(0));
                }

                {
                    ostringstream tag; tag.precision(3);
                    tag<<x_max;
                    cvPutText(img.getIplImage(),tag.str().c_str(),cvPoint(280,230),&font,cvScalar(0));
                }

                {
                    ostringstream tag; tag.precision(3);
                    tag<<y_min;
                    cvPutText(img.getIplImage(),tag.str().c_str(),cvPoint(10,215),&font,cvScalar(0));
                }

                {
                    ostringstream tag; tag.precision(3);
                    tag<<y_max;
                    cvPutText(img.getIplImage(),tag.str().c_str(),cvPoint(10,20),&font,cvScalar(0));
                }

                CvPoint pold;
                for (int i=0; i<input.size(); i++)
                {
                    CvPoint p;
                    p.x=int((img.width()/x_range)*(input.get(i).asDouble()-x_min));
                    p.y=img.height()-int((img.height()/y_range)*(output.get(i).asDouble()-y_min));
                    
                    if (i>0)
                        cvLine(img.getIplImage(),p,pold,cvScalar(0),2);

                    pold=p;
                }

                plotPort.write(img);
            }
        }
    }

    /************************************************************************/
    bool respond(const Bottle &command, Bottle &reply)
    {
        mutex.wait();
        if (command.size()>=1)
        {
            int header=command.get(0).asVocab();
            Bottle payload=command.tail();
            if (header==Vocab::encode("train"))
            {
                if (payload.size()>=3)
                {
                    string item=payload.get(0).asString().c_str();
                    double input=payload.get(1).asDouble();
                    double output=payload.get(2).asDouble();

                    train(item,input,output);
                    reply.addVocab(Vocab::encode("ack"));

                    // trigger a change for the "plot"
                    plotItem=item;
                }
                else
                    reply.addVocab(Vocab::encode("nack"));
            }
            else if (header==Vocab::encode("predict"))
            {
                if (payload.size()>=2)
                {
                    Bottle output,variance;
                    string item=payload.get(0).asString().c_str();
                    if (payload.get(1).isDouble())
                    {
                        Bottle input; input.addDouble(payload.get(1).asDouble());
                        if (predict(item,input,output,variance))
                        {
                            reply.addVocab(Vocab::encode("ack"));
                            reply.addDouble(output.get(0).asDouble());
                            reply.addDouble(variance.get(0).asDouble());
                        }
                        else
                            reply.addVocab(Vocab::encode("nack"));
                    }
                    else if (payload.get(1).isList())
                    {
                        if (predict(item,*payload.get(1).asList(),output,variance))
                        {
                            reply.addVocab(Vocab::encode("ack"));
                            reply.addList().append(output);
                            reply.addList().append(variance);
                        }
                        else
                            reply.addVocab(Vocab::encode("nack"));
                    }
                    else
                        reply.addVocab(Vocab::encode("nack"));
                }
                else
                    reply.addVocab(Vocab::encode("nack"));
            }
            else if (header==Vocab::encode("span"))
            {
                if (payload.size()>=1)
                {                    
                    double step=DEFAULT_STEP;
                    string item=payload.get(0).asString().c_str();
                    if (payload.size()>=2)
                        step=payload.get(1).asDouble();

                    Bottle input,output,variance;
                    for (double d=scalerIn.getLowerBoundIn(); d<scalerIn.getUpperBoundIn(); d+=step)
                        input.addDouble(d);
                    
                    if (predict(item,input,output,variance))
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        reply.addList().append(output);
                        reply.addList().append(variance);
                    }
                    else
                        reply.addVocab(Vocab::encode("nack"));
                }
                else
                    reply.addVocab(Vocab::encode("nack"));
            }
            else if (header==Vocab::encode("optimize"))
            {
                if (payload.size()>=1)
                {
                    Bottle searchDomain;
                    double step=DEFAULT_STEP;

                    string item=payload.get(0).asString().c_str();
                    if (payload.size()>=2)
                    {
                        if (payload.get(1).isDouble())
                            step=payload.get(1).asDouble();
                        else if (payload.get(1).isList())
                            searchDomain=*payload.get(1).asList();
                    }

                    if (searchDomain.size()==0)
                        for (double d=scalerIn.getLowerBoundIn(); d<scalerIn.getUpperBoundIn(); d+=step)
                            searchDomain.addDouble(d);

                    double input,output;
                    if (optimize(item,searchDomain,input,output))
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        reply.addDouble(input);
                        reply.addDouble(output);
                    }
                    else
                        reply.addVocab(Vocab::encode("nack"));
                }
                else
                    reply.addVocab(Vocab::encode("nack"));
            }
            else if (header==Vocab::encode("items"))
            {
                reply.addVocab(Vocab::encode("ack"));
                reply.append(items());
            }
            else if (header==Vocab::encode("machine"))
            {
                if (payload.size()>=1)
                {
                    string item=payload.get(0).asString().c_str();
                    string content;
                    if (machineContent(item,content))
                    {
                        reply.addVocab(Vocab::encode("ack"));
                        reply.addString(content.c_str());
                    }
                    else
                        reply.addVocab(Vocab::encode("nack"));
                }
                else
                    reply.addVocab(Vocab::encode("nack"));
            }
            else if (header==Vocab::encode("clear"))
            {
                if (payload.size()>=1)
                {
                    string item=payload.get(0).asString().c_str();
                    if (clear(item))
                        reply.addVocab(Vocab::encode("ack"));
                    else
                        reply.addVocab(Vocab::encode("nack"));
                }
                else
                {
                    clear();
                    reply.addVocab(Vocab::encode("ack"));
                }
            }
            else if (header==Vocab::encode("save"))
            {
                save();
                reply.addVocab(Vocab::encode("ack"));
            }
            else if (header==Vocab::encode("plot"))
            {
                if (payload.size()>=1)
                {
                    string item=payload.get(0).asString().c_str();
                    if (payload.size()>=2)
                        plotStep=payload.get(1).asDouble();

                    if (machines.find(item)!=machines.end())
                    {
                        plotItem=item;
                        reply.addVocab(Vocab::encode("ack"));
                    }
                    else
                        reply.addVocab(Vocab::encode("nack"));
                }
                else
                    reply.addVocab(Vocab::encode("nack"));
            }
            else
                reply.addVocab(Vocab::encode("nack"));
        }
        else
            reply.addVocab(Vocab::encode("nack"));

        mutex.post();
        return true;
    }

public:
    /************************************************************************/
    bool configure(ResourceFinder &rf)
    {
        // default values
        name="karmaLearn";
        int nItems=0;

        double in_lb=0.0;
        double in_ub=360.0;
        double out_lb=0.0;
        double out_ub=2.0;

        Bottle &generalGroup=rf.findGroup("general");
        if (!generalGroup.isNull())
        {
            name=generalGroup.check("name",Value("karmaLearn")).asString().c_str();
            nItems=generalGroup.check("num_items",Value(0)).asInt();
            in_lb=generalGroup.check("in_lb",Value(0.0)).asDouble();
            in_ub=generalGroup.check("in_ub",Value(360.0)).asDouble();
            out_lb=generalGroup.check("out_lb",Value(0.0)).asDouble();
            out_ub=generalGroup.check("out_ub",Value(2.0)).asDouble();
        }

        scalerIn.setLowerBoundIn(in_lb);
        scalerIn.setUpperBoundIn(in_ub);
        scalerIn.setLowerBoundOut(0.0);
        scalerIn.setUpperBoundOut(1.0);

        scalerOut.setLowerBoundIn(out_lb);
        scalerOut.setUpperBoundIn(out_ub);
        scalerOut.setLowerBoundOut(0.0);
        scalerOut.setUpperBoundOut(1.0);

        // retrieve machines for each item
        for (int i=0; i<nItems; i++)
        {
            ostringstream item; item<<"item_"<<i;
            Bottle &itemGroup=rf.findGroup(item.str().c_str());
            if (!itemGroup.isNull())
            {
                if (!itemGroup.check("name"))
                    continue;

                IMachineLearner *learner=createLearner();
                if (itemGroup.check("learner"))
                    learner->fromString(itemGroup.find("learner").asList()->toString().c_str());

                machines[itemGroup.find("name").asString().c_str()]=learner;
            }
        }

        // save the file name
        configFileName=rf.findPath("from");

        plotItem="";
        plotStep=1.0;

        plotPort.open(("/"+name+"/plot:o").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    /************************************************************************/
    bool interruptModule()
    {
        plotPort.interrupt();
        rpcPort.interrupt();
        return true;
    }

    /************************************************************************/
    bool close()
    {
        save();
        clear();
        plotPort.close();
        rpcPort.close();
        return true;
    }

    /************************************************************************/
    double getPeriod()
    {
        return 0.25;
    }

    /************************************************************************/
    bool updateModule()
    {
        mutex.wait();
        plot();
        mutex.post();

        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        printf("YARP server not available!\n");
        return -1;
    }

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("karma/conf");
    rf.setDefaultConfigFile("karmaLearn.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    KarmaLearn karmaLearn;
    return karmaLearn.runModule(rf);
}



