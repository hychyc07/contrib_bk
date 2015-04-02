/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Carlo Ciliberto
 * email:  carlo.ciliberto@iit.it
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

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/PortReport.h>
#include <yarp/os/PortInfo.h>

#include <yarp/sig/Vector.h>
#include <yarp/sig/Image.h>

#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>

#include <highgui.h>
#include <cv.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <iCub/boostMIL/ClassifierFactory.h>
#include <iCub/boostMIL/ClassifierInput.h>
#include <iCub/boostMIL/WeakClassifier.h>
#include <iCub/boostMIL/MILClassifier.h>
#include <iCub/boostMIL/OnlineBoost.h>

#include <stdio.h>
#include <string>
#include <deque>
#include <algorithm>
#include <vector>
#include <iostream>
#include <fstream>

#include "SiftGPU_Extractor.h"

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::boostMIL;

#define CMD_TRAIN               VOCAB4('t','r','a','i')
#define CMD_CLASSIFY            VOCAB4('c','l','a','s')
#define CMD_LOCK                VOCAB4('l','o','c','k')
#define CMD_UNLOCK              VOCAB4('u','n','l','o')
#define CMD_FORGET              VOCAB4('f','o','r','g')
#define CMD_LOAD                VOCAB4('l','o','a','d')
#define CMD_DETAILS             VOCAB4('d','e','t','a')






class ObjectPropertiesCollectorPort: public RpcClient, public PortReport
{
private:
    bool scheduleUpdate;

public:
    ObjectPropertiesCollectorPort()
        :scheduleUpdate(false)
    {
        this->setReporter(*this);
    }

    void report(const PortInfo &info)
    {
        if(info.created && !info.incoming)
            scheduleUpdate=true;
    }

    bool isUpdateNeeded()
    {
        if(scheduleUpdate)
        {
            scheduleUpdate=false;
            return true;
        }

        return false;
    }
};



class MILPort: public BufferedPort<Image>
{
private:
   ResourceFinder                      &rf;

   Semaphore                           mutex;


   ObjectPropertiesCollectorPort       opcPort;


   string                              models_path;

   string                              weaklearner_type;
   string                              booster_type;

   int                                 feature_size;

   bool                                verbose;

   cv::Ptr<cv::FeatureDetector>        detector;
   cv::Ptr<cv::DescriptorExtractor>    extractor;

   map<string,OnlineBoost*>            classifiers;

   vector<SiftGPU::SiftKeypoint>       keypoints;
   vector<float>                       descriptors;

   Inputs                              *dictionary;

   bool                                negative_training;

   Port                                outPort;
   Port                                siftsPort;

   bool                                imageLocked;
   volatile bool                       gotNewImage;

   double                              waitTimeout;
   
   SiftGPU_Extractor                   siftGPU_extractor;


    bool save(const string &object_name)
    {
        if(classifiers.count(object_name)>0)
            return classifiers[object_name]->saveAsString(("/"+models_path+"/"+object_name+".mil"));
        else
            return false;
    }

    bool load(const string &object_name)
    {
        if(classifiers.count(object_name)==0)
        {
            classifiers[object_name]=new OnlineBoost(booster_type,rf);

            if(!classifiers[object_name]->load(("/"+models_path+"/"+object_name+".mil")))
            {
                delete classifiers[object_name];
                return false;
            }
            else
                return true;
        }
        else
            return false;
    }


    void forget(const string &class_name, Bottle &reply)
    {
        if(classifiers.count(class_name)>0)
        {
            delete classifiers[class_name];
            classifiers.erase(classifiers.find(class_name));

            reply.addString(("'"+class_name+"' deleted.").c_str());
        }
        else
            reply.addString(("Object '"+class_name+"' not present. Nothing to delete.").c_str());
    }

    void forget_all()
    {
        for(map<string,OnlineBoost*>::iterator cl=classifiers.begin(); cl!=classifiers.end(); cl++)
            delete cl->second;

        classifiers.clear();
    }



   Inputs *loadDictionary(const string &path)
   {
       ifstream fin(path.c_str());
       Vector *vec_input=NULL;

       if(fin.is_open())
       {
           stringstream strstr;
           strstr << fin.rdbuf();
           Bottle bDictionary(strstr.str().c_str());

           if(bDictionary.size()>0)
           {
               int feature_size=bDictionary.get(0).asList()->size();
               vec_input=new Vector(feature_size*(bDictionary.size()));

               int idx_input=0;

               for(int i=0; i<bDictionary.size(); i++)
               {
                   for(int j=0; j<feature_size;j++)
                       vec_input->data()[idx_input+j]=bDictionary.get(i).asList()->get(j).asDouble();
                   idx_input+=feature_size;
               }
           }
           fin.close();
       }

       if(vec_input==NULL)
           return NULL;

       return new Inputs(+1,weaklearner_type,vec_input);
   }


   //Extracts two inputs samples: one positive from within the specified blob, the other negative from the remaining aread
   void extractFromMask(Bottle *mask, Inputs *&p_input, Inputs *&n_input)
   {
       p_input=NULL;
       n_input=NULL;

       int x_min=mask->get(0).asInt();
       int y_min=mask->get(1).asInt();
       int x_max=mask->get(2).asInt();
       int y_max=mask->get(3).asInt();

       //count the number of elements in the window
       int cnt=0;
       for(unsigned int i=0; i<keypoints.size(); i++)
       {
           if(x_min<=keypoints[i].x && keypoints[i].x<=x_max &&
              y_min<=keypoints[i].y && keypoints[i].y<=y_max   )
               cnt++;
       }

       Vector *p_positions,*n_positions,*p_descriptors,*n_descriptors;
       p_positions=n_positions=p_descriptors=n_descriptors=NULL;

       if(cnt!=0)
       {
           p_positions=new Vector(2*cnt);
           p_descriptors=new Vector(feature_size*cnt);
       }

       if(cnt!=keypoints.size())
       {
           n_positions=new Vector(2*(keypoints.size()-cnt));
           n_descriptors=new Vector(feature_size*(keypoints.size()-cnt));
       }

       //fill the positive and negative samples
       int p_cnt=0;
       int n_cnt=0;
       for(unsigned int i=0; i<keypoints.size(); i++)
       {
           if(x_min<=keypoints[i].x && keypoints[i].x<=x_max &&
              y_min<=keypoints[i].y && keypoints[i].y<=y_max   )
           {
               p_positions->data()[2*p_cnt]=keypoints[i].x;
               p_positions->data()[2*p_cnt+1]=keypoints[i].y;
               for(int j=0; j<feature_size;j++)
                   p_descriptors->data()[feature_size*p_cnt + j]=descriptors[feature_size*i + j];

               p_cnt++;
           }
           else
           {
               n_positions->data()[2*n_cnt]=keypoints[i].x;
               n_positions->data()[2*n_cnt+1]=keypoints[i].y;
               for(int j=0; j<feature_size;j++)
                   n_descriptors->data()[feature_size*n_cnt + j]=descriptors[feature_size*i + j];

               n_cnt++;
           }
       }

       if(p_positions!=NULL)
       {
           p_input=new Inputs(1,weaklearner_type,p_descriptors,p_positions);
           delete p_positions;
           delete p_descriptors;
       }

       if(n_positions!=NULL)
       {
           n_input=new Inputs(-1,weaklearner_type,n_descriptors,n_positions);
           delete n_positions;
           delete n_descriptors;
       }
   }


   //Extracts input samples from single blob
   void extractFromBlob(Bottle *blob, Inputs *&input)
   {
       input=NULL;

       int x_min=blob->get(0).asInt();
       int y_min=blob->get(1).asInt();
       int x_max=blob->get(2).asInt();
       int y_max=blob->get(3).asInt();

       //count the number of elements in the window
       int cnt=0;
       for(unsigned int i=0; i<keypoints.size(); i++)
       {
           if(x_min<=keypoints[i].x && keypoints[i].x<=x_max &&
              y_min<=keypoints[i].y && keypoints[i].y<=y_max   )
               cnt++;
       }

       if(cnt!=0)
       {
           Vector *tmp_positions=new Vector(2*cnt);
           Vector  *tmp_descriptors=new Vector(feature_size*cnt);

           int tmp_cnt=0;
           for(unsigned int i=0; i<keypoints.size(); i++)
           {
               if(x_min<=keypoints[i].x && keypoints[i].x<=x_max &&
                  y_min<=keypoints[i].y && keypoints[i].y<=y_max   )
               {
                   tmp_positions->data()[2*tmp_cnt]=keypoints[i].x;
                   tmp_positions->data()[2*tmp_cnt+1]=keypoints[i].y;
                   for(int j=0; j<feature_size;j++)
                       tmp_descriptors->data()[feature_size*tmp_cnt + j]=descriptors[feature_size*i+j];

                   tmp_cnt++;
               }
           }

           input=new Inputs(0,weaklearner_type,tmp_descriptors,tmp_positions);
           delete tmp_positions;
           delete tmp_descriptors;
       }
   }


   virtual void onRead(Image &img)
   {
       static double in = Time::now();
       //fprintf(stdout, "the time is %lf\n",Time::now()-in);
       mutex.wait();
       if(!imageLocked)
       {
       
        
       
            IplImage *ipl=(IplImage*) img.getIplImage();
            IplImage *dst = 0;
        
            //cvCvtColor( ipl, ipl, CV_BGR2RGB );

            // CV_GAUSSIAN_5x5 A 5x5 Gaussian lowpass filter.  
            // This filter uses the kernel A/571,where  
            //         2  7  12  7  2  
            //         7 31  52 31  7  
            //    A =  12 52 127 52 12  
            //         7 31  52 31  7  
            //         2  7  12  7  2  

            float k[25] = {  2.f/571, 7.f/571, 12.f/571, 7.f/571, 2.f/571,  
                        7.f/571, 31.f/571, 52.f/571, 31.f/571, 7.f/571,  
                        12.f/571, 52.f/571, 127.f/571, 52.f/571, 12.f/571,
                        7.f/571, 31.f/571, 52.f/571, 31.f/571, 7.f/571,
                        2.f/571, 7.f/571, 12.f/571, 7.f/571, 2.f/571};

            CvMat Km;  
            //cvInitMatHeader( &Km, 3, 3, CV_32FC1, k, CV_AUTOSTEP );  
            Km = cvMat( 5, 5, CV_32F, k );
            //cvCopy( ipl, dst );  
            dst = cvCloneImage( ipl );
        
            cvFilter2D( ipl, dst, &Km, cvPoint(-1,-1));  

            //extract sift (who could've tell from the method's name?)
            double t=Time::now();

           siftGPU_extractor.extractSift(dst);
           
           int feature_num=siftGPU_extractor.getFeatureNum();

           if(verbose)
               fprintf(stdout,"%d features extracted in %f [s]\n",feature_num,Time::now()-t);

           double *pos,*vec;
           pos=vec=NULL;
           if(feature_num>0)
           {
               siftGPU_extractor.getFeatureVector(&keypoints,&descriptors);
           }

           //double td=Time::now();
           //detector->detect((IplImage*)img.getIplImage(),keypoints);
           //td=Time::now()-td;
           //double te=Time::now();
           //extractor->compute((IplImage*)img.getIplImage(),keypoints,descriptors);
           //te=Time::now()-te;

           gotNewImage=true;

           //if(verbose)
               //fprintf(stdout,"Detection: %f [s]\nExtraction: %f [s]\n\n",td,te);

           if(outPort.getOutputCount())
           {
                for(unsigned int i=0; i<keypoints.size(); i++)
                {   
                    int x = cvRound(keypoints[i].x);
                    int y = cvRound(keypoints[i].y);
                    cvCircle(img.getIplImage(),cvPoint(x,y),2,cvScalar(255),2);
                }
                outPort.write(img);
           }
           
           if(siftsPort.getOutputCount())
           {
                
                Bottle sifts;
                for(unsigned int i=0; i<keypoints.size(); i++)
                {   
                    int x = cvRound(keypoints[i].x);
                    int y = cvRound(keypoints[i].y);
                    
                    Bottle &item = sifts.addList();
                    item.addInt(x);
                    item.addInt(y);
                }
                
                siftsPort.write(sifts);
           }

            cvReleaseImage( &dst );
       }
       mutex.post();
   }

   void lock(Bottle &reply)
   {
       mutex.wait();
       imageLocked=true;

       reply.addString("image locked");
       mutex.post();
   }

   void unlock(Bottle &reply)
   {
       mutex.wait();
       imageLocked=false;

       reply.addString("image unlocked");
       mutex.post();
   }

   void classify(Bottle *blobs, Bottle &reply)
   {
       double initialTime=Time::now();
       while(!gotNewImage && (Time::now()-initialTime)<waitTimeout)
           Time::delay(0.01);

       if((gotNewImage || imageLocked) && blobs->size()!=0)
       {
           //for each blob, extract an input sample containing the features present in it
           for(int b=0; b<blobs->size(); b++)
           {
               Bottle &blob_scorelist=reply.addList();
               blob_scorelist.addString(blobs->get(b).asList()->get(0).asString().c_str());

               Bottle &scores=blob_scorelist.addList();

               Inputs *input=NULL;
               mutex.wait();

               extractFromBlob(blobs->get(b).asList()->get(1).asList(),input);

               mutex.post();

               if(input!=NULL)
               {
                   for(map<string,OnlineBoost*>::iterator cl=classifiers.begin(); cl!=classifiers.end(); cl++)
                   {
                       if(cl->second->isReady())
                       {
                           Bottle &classifier_score=scores.addList();
                           classifier_score.addString(cl->first.c_str());
                           classifier_score.addDouble(cl->second->margin(input));
                       }
                   }

                   delete input;
               }
           }

           gotNewImage=false;
       }
       else
           reply.addString("failed");
       cout << "reply is: " << reply.toString().c_str() << endl;
   }


   void train(Bottle *locations, Bottle &reply)
   {
       double initialTime=Time::now();
       while(!gotNewImage && (Time::now()-initialTime)<waitTimeout)
           Time::delay(0.01);

       if((gotNewImage || imageLocked) && locations->size()!=0)
       {
           for(int i=0; i<locations->size(); i++)
           {
               string object_name=locations->get(i).asList()->get(0).asString().c_str();

               //WARNING!!!
               //if the object to be trained does not exist yet, create it.
               if(classifiers.count(object_name)==0)
               {
                   classifiers[object_name]=new OnlineBoost(booster_type,rf);

                   //initialized the classifier with a dictionary
                   if(dictionary!=NULL)
                       classifiers[object_name]->initialize(dictionary);
               }

               Inputs *p_input=NULL;
               Inputs *n_input=NULL;


               //fill the classifier with positive features.
               while(!classifiers[object_name]->isReady())
               {
                   mutex.wait();
                   extractFromMask(locations->get(i).asList()->get(1).asList(),p_input,n_input);
                   mutex.post();

                   if(n_input!=NULL)
                       delete n_input;

                   if(p_input!=NULL)
                   {
                       classifiers[object_name]->initialize(p_input);
                       delete p_input;
                   }
                   else
                   {
                       reply.addString(("object '"+object_name+"' not trained").c_str());
                       return;
                   }
               }


               mutex.wait();
               extractFromMask(locations->get(i).asList()->get(1).asList(),p_input,n_input);
               mutex.post();

               if(n_input!=NULL)
               {
                   classifiers[object_name]->train(n_input);
                   delete n_input;
               }

               if(p_input!=NULL)
               {
                   classifiers[object_name]->train(p_input);

                   //use the positive input as a negative sample for the remaining classifiers
                   if(negative_training)
                   {
                       p_input->setLabel(-1);
                       for(map<string,OnlineBoost*>::iterator itr=classifiers.begin(); itr!=classifiers.end(); itr++)
                       {
                           if((*itr).first!=object_name)
                               (*itr).second->train(p_input);
                       }
                   }

                   delete p_input;
               }

                //save the model on file
                save(object_name);

               reply.addString(("object '"+object_name+"' trained").c_str());

               gotNewImage=false;
           }

       }
       else
           reply.addString("failed");
   }
   
   
   void train_and_return_details(Bottle *locations, Bottle &reply)
   {
       double initialTime=Time::now();
       while(!gotNewImage && (Time::now()-initialTime)<waitTimeout)
           Time::delay(0.01);

       if((gotNewImage || imageLocked) && locations->size()!=0)
       {
           for(int i=0; i<locations->size(); i++)
           {
               string object_name=locations->get(i).asList()->get(0).asString().c_str();

               //WARNING!!!
               //if the object to be trained does not exist yet, create it.
               if(classifiers.count(object_name)==0)
               {
                   classifiers[object_name]=new OnlineBoost(booster_type,rf);

                   //initialized the classifier with a dictionary
                   if(dictionary!=NULL)
                       classifiers[object_name]->initialize(dictionary);
               }

               Inputs *p_input=NULL;
               Inputs *n_input=NULL;


               //fill the classifier with positive features.
               while(!classifiers[object_name]->isReady())
               {
                   mutex.wait();
                   extractFromMask(locations->get(i).asList()->get(1).asList(),p_input,n_input);
                   mutex.post();

                   if(n_input!=NULL)
                       delete n_input;

                   if(p_input!=NULL)
                   {
                       classifiers[object_name]->initialize(p_input);
                       delete p_input;
                   }
                   else
                   {
                       //reply.addString(("object '"+object_name+"' not trained").c_str());
                       return;
                   }
               }


               mutex.wait();
               extractFromMask(locations->get(i).asList()->get(1).asList(),p_input,n_input);
               mutex.post();

                //sample ( (info <info>) (features <features>) )

                //<info> = ((size <size>) (type <type))
                Bottle &init_info_header=reply.addList();
                init_info_header.addString("info");
                Bottle &info_header=init_info_header.addList();

                Bottle *tmp_info=&info_header.addList();
                tmp_info->addString("size");
                tmp_info->addInt(feature_size);  

                tmp_info=&info_header.addList();
                tmp_info->addString("type");
                tmp_info->addString("SIFT");  

                tmp_info=&info_header.addList();
                tmp_info->addString("object_name");
                tmp_info->addString(object_name.c_str());  

                //<feature> = ( (<label> (descriptor (<descriptor>) ) (positions (<positions>) ) ) ... ) 
                Bottle &feature_header=reply.addList();
                feature_header.addString("features");
                Bottle &feature_list=feature_header.addList();

                Bottle &tmp_neg=feature_list.addList();
                tmp_neg.addInt(-1);

                Bottle &neg_descriptors_header=tmp_neg.addList();
                neg_descriptors_header.addString("descriptors");
                Bottle &neg_descriptors=neg_descriptors_header.addList();

                Bottle &neg_positions_header=tmp_neg.addList();
                neg_positions_header.addString("positions");
                Bottle &neg_positions=neg_positions_header.addList();

                if(n_input!=NULL)
                {
                    //put everything in the reply bottle
                    neg_descriptors.fromString(n_input->getInput("mil")->toString());
                    neg_positions.fromString(n_input->getPositions("mil")->toString());

                    classifiers[object_name]->train(n_input);
                    delete n_input;
                }

                Bottle &tmp_pos=feature_list.addList();
                tmp_pos.addInt(+1);

                Bottle &pos_descriptors_header=tmp_pos.addList();
                pos_descriptors_header.addString("descriptors");
                Bottle &pos_descriptors=pos_descriptors_header.addList();

                Bottle &pos_positions_header=tmp_pos.addList();
                pos_positions_header.addString("positions");
                Bottle &pos_positions=pos_positions_header.addList();

                if(p_input!=NULL)
                {
                    //put everything in the reply bottle
                    pos_descriptors.fromString(p_input->getInput("mil")->toString());
                    pos_positions.fromString(p_input->getPositions("mil")->toString());

                    classifiers[object_name]->train(p_input);

                    //use the positive input as a negative sample for the remaining classifiers
                    if(negative_training)
                    {
                        p_input->setLabel(-1);
                        for(map<string,OnlineBoost*>::iterator itr=classifiers.begin(); itr!=classifiers.end(); itr++)
                        {
                            if((*itr).first!=object_name)
                               (*itr).second->train(p_input);
                        }
                    }

                    delete p_input;
                }

                //save the model on file
                save(object_name);
                //reply.addString(("object '"+object_name+"' trained").c_str());
                gotNewImage=false;
           }

       }
       else
           reply.addString("failed");
   }

public:
   MILPort(ResourceFinder &_rf)
       :BufferedPort<Image>(),rf(_rf)
   {
       verbose=rf.check("verbose");

       //features parameters initialization
       Bottle &bGeneral=rf.findGroup("general");

       negative_training=rf.check("negative_training") || bGeneral.check("negative_training");



        if(bGeneral.check("models_path"))
            models_path=bGeneral.find("models_path").asString().c_str();
        else
        {
            int argc=0;
            char **argv=NULL;
            ResourceFinder modelsFinder;
            modelsFinder.setVerbose(verbose);
            modelsFinder.setDefaultContext("milClassifier/models");
            modelsFinder.configure("ICUB_ROOT",argc,argv);

            models_path=modelsFinder.getContextPath().c_str();
        }

       booster_type=bGeneral.find("booster_type").asString().c_str();
       weaklearner_type=bGeneral.find("weaklearner_type").asString().c_str();


       string feature_name=bGeneral.find("feature_name").asString().c_str();



       Bottle &bFeature=rf.findGroup(feature_name.c_str());

       feature_size=bFeature.find("feature_size").asInt();

       //insert the feature size in the mil ResourceFinder
       Bottle &b=rf.findGroup(weaklearner_type.c_str()).addList();
       b.addString("feature_size");
       b.addInt(feature_size);

       //register the weaklearner
       ClassifierFactory::instance().registerClassifier(new MILClassifier(weaklearner_type,rf));



       string detector_type=bFeature.check("detector",Value(feature_name.c_str())).asString().c_str();
       string extractor_type=bFeature.check("extractor",Value(feature_name.c_str())).asString().c_str();

       detector=cv::FeatureDetector::create(detector_type);
       extractor=cv::DescriptorExtractor::create(extractor_type);

       string dictionary_path=bGeneral.find("dictionary_path").asString().c_str();
       dictionary=loadDictionary(rf.findFile(dictionary_path.c_str()).c_str());

       if(dictionary!=NULL && verbose)
           fprintf(stdout,"Found Dictionary!\n");


       string name=rf.find("name").asString().c_str();

       opcPort.open(("/"+name+"/OPC:io").c_str());

       outPort.open(("/"+name+"/img:o").c_str());
       siftsPort.open(("/"+name+"/sifts:o").c_str());

       BufferedPort<Image>::useCallback();

       waitTimeout=rf.check("wait_timeout",Value(3.0)).asDouble();
       gotNewImage=false;
       imageLocked=false;
   }

   virtual void close()
   {
       forget_all();

       outPort.close();
       siftsPort.close();
       BufferedPort<Image>::close();
   }


   bool execReq(const Bottle &command, Bottle &reply)
   {
       switch(command.get(0).asVocab())
       {
           case(CMD_CLASSIFY):
           {
               classify(command.get(1).asList(),reply);

               return true;
           }

           case(CMD_TRAIN):
           {
               train(command.get(1).asList(),reply);

               return true;
           }

           case(CMD_LOCK):
           {
               lock(reply);

               return true;
           }

           case(CMD_UNLOCK):
           {
               unlock(reply);

               return true;
           }

           case(CMD_FORGET):
           {
               if(command.size()>1 && command.get(1).asString()!="everything" && command.get(1).asString()!="all")
                   forget(command.get(1).asString().c_str(),reply);
               else
               {
                   forget_all();
                   reply.addString("Memory cleared.");
               }
               return true;
           }

           case(CMD_LOAD):
           {
               if(command.size()>1)
               {
                   string object_name=command.get(1).asString().c_str();
                    if(load(object_name))
                        reply.addString(("'"+object_name+"' loaded").c_str());
                    else
                        reply.addString(("'"+object_name+"' already loaded").c_str());
               }

               return true;
           }
           case(CMD_DETAILS):
           {
                reply.clear();
                train_and_return_details(command.get(1).asList(),reply);
                
                return true;
           }

           default:
               return false;
       }
   }



   
    //if an update has been scheduled, load objects from file
    void update()
    {
        if(!opcPort.isUpdateNeeded())
            return;

        forget_all();

        //ask for all items stored in memory
        Bottle bAsk,bReply,bGet,bReplyProp;
        bAsk.addVocab(Vocab::encode("ask"));
        Bottle &bAskTemp=bAsk.addList().addList();
        bAskTemp.addString("entity"); bAskTemp.addString("=="); bAskTemp.addString("object");

        opcPort.write(bAsk,bReply);

        if(bReply.get(0).asVocab()==Vocab::encode("ack"))
        {
            if(Bottle *idValues=bReply.get(1).asList()->get(1).asList())
            {
                for(int i=0; i<idValues->size(); i++)
                {
                    int id=idValues->get(i).asInt();
                    //get the relevant properties: [get] (("id" <num>) ("propSet" ("name")))
                    bGet.clear();
                    bGet.addVocab(Vocab::encode("get"));
                    Bottle &bGetContent=bGet.addList();
                    Bottle &bGetList=bGetContent.addList();
                    bGetList.addString("id"); bGetList.addInt(id);
                    Bottle &bGetPropSet=bGetContent.addList();
                    bGetPropSet.addString("propSet"); bGetPropSet.addList().addString("name");

                    opcPort.write(bGet,bReplyProp);

                    if(bReplyProp.get(0).asVocab()==Vocab::encode("ack"))
                    {
                        if(Bottle *propField=bReplyProp.get(1).asList())
                        {
                            if(propField->check("name"))
                            {
                                string object_name=propField->find("name").asString().c_str();
                                //update internal database
                                load(object_name);
                            }
                        }
                    }


                }


            }
        }
    }


};


class MILModule: public RFModule
{
protected:
    MILPort         *milPort;
    Port            rpcPort;

public:
    MILModule()
    {}

    virtual bool configure(ResourceFinder &rf)
    {
        string name=rf.find("name").asString().c_str();

        Time::turboBoost();

        milPort=new MILPort(rf);
        milPort->open(("/"+name+"/img:i").c_str());
        rpcPort.open(("/"+name+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    virtual bool close()
    {
        milPort->interrupt();
        milPort->close();
        delete milPort;

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if(milPort->execReq(command,reply))
            return true;
        else
            return RFModule::respond(command,reply);
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule()
    {
        milPort->update();

        return true;
    }

};




int main(int argc, char *argv[])
{
   Network yarp;

   if (!yarp.checkNetwork())
       return -1;

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultContext("milClassifier/conf");
   rf.setDefaultConfigFile("config.ini");
   rf.configure("ICUB_ROOT",argc,argv);
   rf.setDefault("name","milClassifier");
   MILModule mod;

   return mod.runModule(rf);
}

