// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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
  
/**
 * @file BMLEngine.cpp
 * @brief Implementation of the module of the application ( see BMLEngine.h )
 */

#include <iCub/BMLEngine.h>
#include <string.h>


#define T_TOUCH 1000
#define COOLING_RATE 4

//
static BMLEngine *engineModule;

// Image Receiver
static YARPImgRecv *ptr_imgRecv;
#define _imgRecv (*(ptr_imgRecv))

//Image been read
/*
static yarp::sig::ImageOf<yarp::sig::PixelRgb> *ptr_inputImg;
#define _inputImg (*(ptr_inputImg))
*/

// Semaphore
static yarp::os::Semaphore *ptr_semaphore;
#define _semaphore (*(ptr_semaphore))

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool BMLEngine::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/bmlEngine"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";

    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port

    //initilisation of variables
    open();

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}



void BMLEngine::openCommandPort(){
    _pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    printf("Registering port %s on network %s...\n", getName("/cmd:o").c_str(),"default");
    bool ok = _pOutPort->open(getName("/cmd:o").c_str());
    if  (ok)
        printf("Port registration succeed!\n");
    else 
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
        }
}

void BMLEngine::closeCommandPort(){
    //_pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    //printf("Closing port %s on network %s...\n", getName("/cmd:o"),"default");
    _pOutPort->close();//("/rea/BMLEngine/outCommand");
}

/**
* function that opens the port where the inputImage is read
*/
bool BMLEngine::openPortImage(){
    bool ret = false;
    //int res = 0;
    // Registering Port(s)
    //reduce verbosity --paulfitz
    printf("Registering port %s on network %s...\n", "/bmlEngine/inputImage","default");
    string portName(this->name);
    portName.append("/inputImage");
    ret = _imgRecv.Connect((char*)portName.c_str(),"default");
    if (ret == true)
        {
            //reduce verbosity --paulfitz
            printf("Port registration succeed!\n");
        }
    else
        {
            printf("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }
}

/**
* function that opens the port where the inputImage is read
*/
bool  BMLEngine::closePortImage(){
    bool ret = true;
    //int res = 0;
    // Registering Port(s)
    //reduce verbosity --paulfitz
    printf("Closing port %s on network %s...\n", "/inputImage","default");
    _imgRecv.Disconnect();//("/rea/BMLEngine/inputImage","default");
    return ret;
}


bool getImage(){
    bool ret = false;
    ret = _imgRecv.Update();

    if (ret == false){
        return false;
    }

    
    _semaphore.wait();
    ret = _imgRecv.GetLastImage(engineModule->ptr_inputImage);
    //ret = _imgRecv.GetLastImage(&_inputImg);
    //engineModule->ptr_inputImage=&_inputImg;
    _semaphore.post();
    

    engineModule->inputImage_flag=true;
    
    //printf("GetImage: out of the semaphore \n");
    return true;
}


void createObjects() {
    ptr_imgRecv = new YARPImgRecv;
    //ptr_inputImg = new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    //ptr_inputImg2= new yarp::sig::ImageOf<yarp::sig::PixelRgb>;
    ptr_semaphore = new yarp::os::Semaphore;
}


bool BMLEngine::outCommandPort(){
    if(strcmp(outCommand->c_str(),"")){	
        Bottle& outBot1=_pOutPort->prepare();
        //bOptions.addString("to");
        //bOptions.addString("Layer0");
        printf("outPort %s \n",outCommand->c_str());
        outBot1.fromString(outCommand->c_str());
        outBot1.addList()=bOptions;
        this->_pOutPort->writeStrict();
        outCommand->clear();
        bOptions.clear();
    }
    return true;
}

bool BMLEngine::open() {
        resized = false;
        inputColour = false;
        count=0;
        scaleFactorX=10;
        scaleFactorY=10;
        ptr_inputImage=0;
        clampingThreshold=1;
        countLayer=0;
        currentLayer=0;
        
        createObjects();
        openPortImage();
        openCommandPort();
        outCommand=new string("");

        portMono.open(getName("/bmlEngine/imageMono:i")); 
        port.open(getName("/bmlEngine/image:i")); 
        //port0.open(getName("/bmlEngine/layer0:o"));
        //port1.open(getName("/bmlEngine/layer1:o"));
        //port2.open(getName("/bmlEngine/layer2:o")); 
        //weight01.open(getName("/bmlEngine/weight01:o"));
        //weight12.open(getName("/bmlEngine/weight12:o"));
        //weight23.open(getName("/bmlEngine/weight23:o")); 
        portCmd.open(getName("/bmlEngine/cmd"));
        portCmd.setStrict();
        
        img0=new ImageOf<PixelRgb>;
        img0->resize(320,240);
        img2=new ImageOf<PixelRgb>;
        img2->resize(320,240);

        ptr_inputImage2=0;
        ptr_inputImage=0;
        ptr_inputImageMono=0;
        tmp=0;
        tmpMono=0;

        //setting of the flags
        enableDraw=false;
        inputImage_flag=false;
        runFreely=false;
        runClamped=false;
        probClamped_flag=false;
        probFreely_flag=false;

        layer0Image=0;
        layer1Image=0;
        layer2Image=0;
        layer3Image=0;
            

        engineModule = this;
        dataCollector = 0;
        return true;
    }

bool  BMLEngine::interruptModule() {
        portMono.interrupt();
        port.interrupt();
        //port0.interrupt();
        //port1.interrupt();
        //port2.interrupt();
        //weight01.interrupt();
        //weight12.interrupt();
        //weight23.interrupt();
        portCmd.interrupt();
        handlerPort.interrupt();
        return true;
    }

bool BMLEngine::close() {
    //stopping threads
    if(layer0Image!=0)
        layer0Image->stop();
    if(layer1Image!=0)
        layer1Image->stop();
    if(layer2Image!=0)
        layer2Image->stop();
    if(dataCollector!=0) {
        dataCollector->stop();
    }
    delete layer0Image;
    delete layer1Image;
    delete layer2Image;
    delete layer3Image;
    delete dataCollector;

    //closing ports
    portMono.close();
    port.close();
    handlerPort.close();
    portCmd.close();//(getName("inCmd"));
    closePortImage();
    closeCommandPort();

    //freeing memory
    delete img0;
    delete img2;
    if(ptr_inputImage2!=0)
        delete ptr_inputImage2;
    if(ptr_inputImage!=0)
        delete ptr_inputImage;
    if(ptr_inputImageMono!=0)
        delete ptr_inputImageMono;
    if(tmp!=0)
        delete tmp;
    if(tmpMono!=0)
        delete tmpMono;

    delete mb;

    return true;
}


/** 
 *Function updateModule is executed when the Module is updated, 
 * 1.It reads the command from a port, convertes it into parameters
 * 2.It evolves the Boltzmann machine 
 * 3.Extracts the output images of the active layers of the Boltzmann machine
 *
 * @param none
 * @return true if there were no errors in the funciton
 *
*/
bool BMLEngine::updateModule() {
    /*
    if(inputColour) {
        tmp=port.read(false);
        if(tmp!=0) {
            if(!resized) {
                int widthInput=tmp->width();
                int heightInput=tmp->height();
                ptr_inputImage=new ImageOf<PixelRgb>;
                ptr_inputImage->resize(widthInput,heightInput);
                resized=true;
            }
            unsigned char* ptmp=tmp->getRawImage();
            unsigned char* pinput= ptr_inputImage->getRawImage();
            int padding = tmp->getPadding();
            for(int r=0;r<tmp->height();r++) {
                for(int c=0;c<tmp->width();c++) {
                    *pinput = *ptmp;
                    pinput++; ptmp++;
                }
                pinput += padding;
                ptmp +=padding;
            }
            //cvCopy(tmp,ptr_inputImage);
        }
        if(ptr_inputImage!=0) {
            inputImage_flag=true;
        }
    }
    else {
        tmpMono=portMono.read(false);
        if(tmpMono!=0) {
            if(!resized) {
                int widthInput=tmpMono->width();
                int heightInput=tmpMono->height();
                ptr_inputImageMono=new ImageOf<PixelMono>;
                ptr_inputImageMono->resize(widthInput,heightInput);
                resized=true;
            }
            unsigned char* ptmp=tmpMono->getRawImage();
            unsigned char* pinput= ptr_inputImageMono->getRawImage();
            int padding = tmpMono->getPadding();
            for(int r=0;r<tmpMono->height();r++) {
                for(int c=0;c<tmpMono->width();c++) {
                    *pinput = *ptmp;
                    pinput++; ptmp++;
                }
                pinput += padding;
                ptmp +=padding;
            }
        }
        if(ptr_inputImageMono!=0) {
            inputImage_flag=true;
        }
    }
    */
    Bottle *bot=portCmd.read(false);
    if(bot!=NULL){
        string *commandTOT=new string(bot->toString().c_str());
        string command,option;
        string optionName1,optionValue1,optionName2, optionValue2;
        printf("Bottle  is: %s\n",commandTOT->c_str());
        unsigned int parOpen=commandTOT->find("(");
        if(parOpen==-1){
            printf("Simple command \n ");
            command=commandTOT->substr(0,commandTOT->size());
        }
        else
        {
            //first parameter list
            command=commandTOT->substr(0,parOpen-1);
            option=commandTOT->substr(parOpen+1,commandTOT->size()-parOpen);
            
            
            unsigned int  parPos1=option.find("(");
            unsigned int parPos2=option.find(")");
            unsigned int spacePos=option.find(" ");
            
            if(spacePos!=-1){
                printf("Presence of a space detected \n");
                optionName1=option.substr(parPos1+1,spacePos-parPos1);
                optionValue1= option.substr(spacePos+1,parPos2-spacePos-1);
                unsigned int dim=option.size();
                if(dim>parPos2+2){
                    option=option.substr(parPos2+2,dim-2-parPos2);

                    parPos1=option.find("(");
                    if(parPos1!=-1){
                        printf("found the second parentesis \n");
                        parPos2=option.find(")");
                        spacePos=option.find(" ");
                        optionName2=option.substr(parPos1+1,spacePos-parPos1);
                        optionValue2= option.substr(spacePos+1,parPos2-spacePos-1);
                        option=option.substr(parPos2,option.size()-parPos2);
                    }
                }else{
                    optionName2.append("");
                    optionValue2.append("");
                }

            
                //string name=option.substr(1,spacePos-1);
                //string value=option.substr(spacePos+1,option.size()-spacePos);
            }
            
            printf("option: |%s| \n",option.c_str());
            printf("name1: |%s| \n",optionName1.c_str());
            printf("value1: |%s| \n",optionValue1.c_str());
            
            
            printf("option: |%s| \n",option.c_str());
            printf("name2: |%s| \n",optionName2.c_str());
            printf("value2: |%s| \n",optionValue2.c_str());
        }

        printf("command: |%s| \n",command.c_str());

        if(!strcmp(command.c_str(),"EvolveFreely")){
            printf("ExecuteFreely \n");
            mb->setTemperature(T_TOUCH);
            runFreely=true;
            runClamped=false;
        }
        else if(!strcmp(command.c_str(),"EvolveClamped")){
            printf("ExecuteClamped \n");
            mb->setTemperature(T_TOUCH);
            runFreely=false;
            runClamped=true;
        }
        else if(!strcmp(command.c_str(),"ConnectLayer")){
            int value1=-1,value2=-1;
            printf("ConnectLayer \n");


            if(!strcmp(optionValue1.c_str(),"layer0")){
                value1=0;
            }
            else if(!strcmp(optionValue1.c_str(),"layer1")){
                value1=1;
            }

            if(!strcmp(optionValue2.c_str(),"layer0")){
                value2=0;
            }
            else if(!strcmp(optionValue2.c_str(),"layer1")){
                value2=1;
            }

            if((value1!=-1)&&(value2!=-1)) {
                map<std::string,Layer>::iterator iterK;
                map<std::string,Layer>::iterator iterL;
                map<std::string,Layer*>::iterator iterE1,iterE2;
                
                int j=0;
                for(iterE1=mb->elementList.begin(); iterE1!=mb->elementList.end()&&j<value1;iterE1++){
                    j++;
                }
                j=0;
                for(iterE2=mb->elementList.begin(); iterE2!=mb->elementList.end()&&j<value2;iterE2++){
                    j++;
                }
                int valueA = iterE1->second->getRow();
                int valueB = iterE1->second->getCol();
                printf("call to the library for layer interconnection \n");
                mb->interconnectLayers(iterE1->second,iterE2->second);
                printf("layers correctly connected \n");
            }
        }
        else if(!strcmp(command.c_str(),"Stop")){
            printf("ExecuteStop \n");
            runFreely=false;
            runClamped=false;
        }
        else if(!strcmp(command.c_str(),"Learn")){
            printf("Learn \n");
            mb->training();
        }
        else if(!strcmp(command.c_str(),"setProbabilityFreely")){
            printf("setProbabilityFreely \n");
            this->probFreely_flag=true;
            this->probClamped_flag=false;
            
        }
        else if(!strcmp(command.c_str(),"setProbabilityClamped")){
            printf("setProbabilityClamped \n");
            this->probFreely_flag=false;
            this->probClamped_flag=true;
            
        }
        else if(!strcmp(command.c_str(),"setProbabilityNull")){
            printf("setProbabilityNull \n");
            this->probFreely_flag=false;
            this->probClamped_flag=false;
            
        }
        else if(!strcmp(command.c_str(),"dataSetStart")){
            printf("dataSet:start \n");
            startMakeBatches();
        }
        else if(!strcmp(command.c_str(),"dataSetStop")){
            printf("dataSet:stop \n");
            stopMakeBatches();
        }
        else if(!strcmp(command.c_str(),"addSample")){
            printf("addSample \n");
            addSample();
        }
        else if(!strcmp(command.c_str(),"ClampLayer")){
            printf("ClampLayer \n");
            int value1=0;
            if(!strcmp(optionValue1.c_str(),"layer0")){
                value1=0;
            }
            else if(!strcmp(optionValue1.c_str(),"layer1")){
                value1=1;
            }
            map<std::string,Layer>::iterator iterK;
            map<std::string,Layer>::iterator iterL;
            map<std::string,Layer*>::iterator iterE1,iterE2;
            
            int j=0;
            for(iterE1=mb->elementList.begin(); iterE1!=mb->elementList.end()&&j<value1;iterE1++){
                j++;
            }
            clampLayer(iterE1->second);
        }
        else if(!strcmp(command.c_str(),"ClampPattern")){
            printf("ClampPattern \n");
            printf("OptionValue1 %s \n" , optionValue1.c_str() );
            int bit;
            unsigned int comma_pos=optionValue1.find(",");
            string bit_str=optionValue1.substr(1,comma_pos-1);
            bit=atoi(bit_str.c_str());
            printf("bit_str %d \n", bit);
            mb->addClampedUnit(288+6+1+0,bit);
            optionValue1=optionValue1.substr(comma_pos+1,optionValue1.size()-comma_pos);
            printf("optionValue1 %s \n", optionValue1.c_str());
            comma_pos=optionValue1.find(",");
            bit_str=optionValue1.substr(1,comma_pos-1);
            bit=atoi(bit_str.c_str());
            optionValue1=optionValue1.substr(comma_pos+1,optionValue1.size()-comma_pos);
            printf("bit_str %d \n", bit);
            printf("optionValue1 %s \n", optionValue1.c_str());
            mb->addClampedUnit(288+6+1+1,bit);
        }
        else if(!strcmp(command.c_str(),"outHeadBehaviour")){
            printf("outHeadBehaviour \n");
            list<Unit>::iterator iter;
            int totUnits=6;
            bool end_loop=false;
            if(mb->_unitList.size()==0){
                return true;
            }
            for(iter=mb->_unitList.begin(); !end_loop;iter++){
                printf("unit name:%s,%d \n",iter->getName().c_str(),iter->getState());
                unsigned int posR=iter->getName().find('R');
                string layerName=iter->getName().substr(1,posR-1);
                int layerNumber=2;
                printf("layer name:%s where as looking for %d \n",layerName.c_str(), layerNumber);
                int extractNumber=atoi(layerName.c_str());
                if(extractNumber==layerNumber)
                    end_loop=true;
            }
            Bottle tmp;
            iter--;
            //layer row of dimension 6 not considered
            for(int i=0;i<totUnits;i++)
                iter++;
            //first unit not considered
            iter++;
            printf("%d,",iter->getState());
            //behaviour0
            if(iter->getState()){
                tmp.addString("behaviour0");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            printf("%d,",iter->getState());
            //behaviour1
            if(iter->getState()){
                tmp.addString("behaviour1");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            printf("%d,",iter->getState());
            //behaviour2
            if(iter->getState()){
                
                tmp.addString("behaviour2");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            printf("%d,",iter->getState());
            //behaviour3
            if(iter->getState()){
                
                tmp.addString("behaviour3");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            printf("%d,",iter->getState());
            //behaviour4
            if(iter->getState()){
                
                tmp.addString("behaviour4");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            printf("%d,",iter->getState());
            //behaviour4
            if(iter->getState()){
                
                tmp.addString("behaviour4");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            outCommand->assign("outHeadBehaviour");
            outCommand->assign("outHeadBehaviour");
        }
        else if(!strcmp(command.c_str(),"outEyesBehaviour")){
            printf("outEyesBehaviour \n");
            list<Unit>::iterator iter;
            int totUnits;
            bool end_loop=false;
            for(iter=mb->_unitList.begin(); !end_loop;iter++){
                printf("unit name:%s,%d \n",iter->getName().c_str(),iter->getState());
                unsigned int posR=iter->getName().find('R');
                string layerName=iter->getName().substr(1,posR-1);
                int layerNumber=2;
                printf("layer name:%s where as looking for %d \n",layerName.c_str(), layerNumber);
                int extractNumber=atoi(layerName.c_str());
                if(extractNumber==layerNumber)
                    end_loop=true;
            }
            Bottle tmp;
            iter--;
            printf("%d,",iter->getState());
            //behaviour0
            if(iter->getState()){
                tmp.addString("behaviour0");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            printf("%d,",iter->getState());
            //behaviour1
            if(iter->getState()){
                tmp.addString("behaviour1");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            printf("%d,",iter->getState());
            //behaviour2
            if(iter->getState()){
                
                tmp.addString("behaviour2");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            printf("%d,",iter->getState());
            //behaviour3
            if(iter->getState()){
                
                tmp.addString("behaviour3");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            printf("%d,",iter->getState());
            //behaviour4
            if(iter->getState()){
                
                tmp.addString("behaviour4");
                tmp.addInt(10);
                bOptions.addList()=tmp;
                tmp.clear();
            }
            iter++;
            outCommand->assign("outHeadBehaviour");
        }
        else if((!strcmp(command.c_str(),"Learn")))
            printf("Learn \n");
        else if(!strcmp(command.c_str(),"Save")){
            printf("save Configuration \n");
            this->mb->saveConfiguration();
        }
        else if((!strcmp(command.c_str(),"Load"))){
            printf("loadConfiguration \n");
            this->loadConfiguration("configuration.ini");
        }
        else if((!strcmp(command.c_str(),"AddLayer"))){
            
            int valueRow=atoi(optionValue1.c_str());
            int valueCol=atoi(optionValue2.c_str());
            printf("addLayer %d %d %d \n", countLayer, valueRow, valueCol);
            if((valueRow!=0)||(valueCol!=0)){
                addLayer(countLayer,valueCol,valueRow);
                countLayer++;
            }
        }
        else if((!strcmp(command.c_str(),"CurrentLayer"))){
            
            int valueLayer=atoi(optionValue1.c_str());
            printf("CurrentUnit %d \n",valueLayer);
            //int valueCol=atoi(optionValue2.c_str());
            this->setCurrentLayer(valueLayer);
        }
    }

    count=(count+1)%40;
    //cout<<count;
    //cout<<".";
    list<Unit>::iterator iter;
    list<Unit>::iterator iter2;

    //extract every unit present in the BoltzmannMachine
    //int psb;
    //IppiSize srcsize={320,240};
    
    
    int k=0;

    outCommandPort();
    outLayers();
    return true;
}



void BMLEngine::evolve1() {
    /*
    if(enableDraw){
        img.resize(320,240);
        //img2->resize(320,240);
        img_tmp.resize(320,240);
        int countLayer=-1;
        //for(iterE=mb->elementList.begin(); iterE!=mb->elementList.end();iterE++){	//iterE=mb->elementList.begin();

            

            //Layer layer=iterE->second;
            int totUnits;
            int countUnit=0;
            int ycount=-1;
            string layerName("");
            string layerName_pr("");
            
            for(iter=mb->_unitList.begin(); iter!=mb->_unitList.end();iter++){
                //countUnit++;
                ct = ((countUnit%10)-1)*24;
                
                //printf("unit name:%s,%d \n",iter->getName().c_str(),iter->getState());
                unsigned int posR=iter->getName().find('R');
                layerName=iter->getName().substr(0,posR-0);
                //printf("layer name:%s \n",layerName.c_str());
                
                if(strcmp(layerName_pr.c_str(),layerName.c_str())){
                    //count the number of element in the row
                    bool end_loop=false;
                    totUnits=-1;
                    iter2=iter;
                    //count the number of units
                    for(;!end_loop;iter2++){
                        if(iter2==mb->_unitList.end()){
                            totUnits++;
                            break;
                        }
                        unsigned int posR=iter2->getName().find('R');
                        unsigned int posU=iter2->getName().find('U');
                        string number_str=iter2->getName().substr(posR+1,posU-posR-1);
                        int number=atoi(number_str.c_str());
                        if(number!=0){
                            end_loop=true;
                        }
                        totUnits++;
                    }
                    //printf("totUnits: %d of layer %s \n",totUnits, layerName.c_str());
                    //printf("countUnits: %d", totUnits);
                    //printf("Change!");
                    //_______  EXIT CONDITION _________________
                    if(strcmp(layerName_pr.c_str(),"")){
                        string countLayer_str=layerName_pr.substr(1,1);
                        countLayer=atoi(countLayer_str.c_str());
                        //printf("number Layer %d \n",countLayer);
                        // output the image
                        //ippiCopy_8u_C1R(red_tmp,psb,im_tmp[0],psb,srcsize);
                        //ippiCopy_8u_C1R(green_tmp,psb,im_tmp[1],psb,srcsize);
                        //ippiCopy_8u_C1R(blue_tmp,psb,im_tmp[2],psb,srcsize);
                        im_tmp[0]=red_tmp;
                        im_tmp[1]=green_tmp;
                        im_tmp[2]=blue_tmp;
                        

                        //ippiCopy_8u_C1R(green_tmp,psb,im_tmp[1],psb,srcsize);
                        //ippiCopy_8u_C1R(blue_tmp,psb,im_tmp[2],psb,srcsize);
                        //ippiCopy_8u_C1R(red_tmp,psb,img2->getPixelAddress(0,0),320,srcsize);
                        
                        // output the image
                        if(countLayer==0){
                            ippiCopy_8u_P3C3R(im_tmp,psb,img2->getPixelAddress(0,0),320*3,srcsize);
                            port0.prepare() = *img2;
                            //if((unsigned int)ptr_inputImage2!=0xcccccccc)
                            //	port0.prepare()= *ptr_inputImage2;
                            port0.write();
                        }
                        else if(countLayer==1){
                            ippiCopy_8u_P3C3R(im_tmp,psb,img2->getPixelAddress(0,0),320*3,srcsize);
                            port1.prepare() = *img2;
                            port1.write();
                        }
                        else if(countLayer==2){
                            ippiCopy_8u_P3C3R(im_tmp,psb,img0->getPixelAddress(0,0),320*3,srcsize);
                            port2.prepare() = *img2;
                            port2.write();
                        }
                        else if(countLayer==3){
                            ippiCopy_8u_P3C3R(im_tmp,psb,img0->getPixelAddress(0,0),320*3,srcsize);
                            port3.prepare() = *img2;
                            port3.write();
                        }
                    }


                    //_______
                    
                    
                    for(int i=0;i<320*240;i++){
                        red_tmp[i]=255;
                        blue_tmp[i]=255;
                        green_tmp[i]=255;
                    }
                    
                    layerName_pr.assign(layerName);
                    countUnit=0;
                    end_loop=false;
                    ycount=0;
                } // close if(strcmp(layerName_pr.c_str(),layerName.c_str()))
                
                
                

                if(countUnit%totUnits==0){
                    ycount++;
                }
                
                //produces the image binary image
                if(iter->getState()){
                    for(int x=0;x<scaleFactorX;x++)
                        for(int y=0;y<scaleFactorY;y++){
                            red_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=255;
                            blue_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=255;
                            green_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=255;
                        }
                }
                else{
                    for(int x=0;x<scaleFactorX;x++)
                        for(int y=0;y<scaleFactorY;y++){
                            red_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=50;
                            blue_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=50;
                            green_tmp[(countUnit%totUnits)*scaleFactorX+x+(ycount*scaleFactorY+y)*320]=50;
                        }
                }
                countUnit++;
            } // close  for(iter=mb->_unitList.begin(); iter!=mb->_unitList.end();iter++)
            


            //printf(" Final picture : totUnits: %d of layer %s \n",totUnits, layerName.c_str());

            string countLayer_str=layerName.substr(1,1);
            countLayer=atoi(countLayer_str.c_str());
            //printf("number Layer %d \n",countLayer);
            // output the image
            //ippiCopy_8u_C1R(red_tmp,psb,im_tmp[0],psb,srcsize);
            //ippiCopy_8u_C1R(green_tmp,psb,im_tmp[1],psb,srcsize);
            //ippiCopy_8u_C1R(blue_tmp,psb,im_tmp[2],psb,srcsize);
            im_tmp[0]=red_tmp;
            im_tmp[1]=green_tmp;
            im_tmp[2]=blue_tmp;

            //ippiCopy_8u_C1R(green_tmp,psb,im_tmp[1],psb,srcsize);
            //ippiCopy_8u_C1R(blue_tmp,psb,im_tmp[2],psb,srcsize);
            //ippiCopy_8u_C1R(red_tmp,psb,img2->getPixelAddress(0,0),320,srcsize);
            ippiCopy_8u_P3C3R(im_tmp,psb,img2->getPixelAddress(0,0),320*3,srcsize);
            // output the image
            if(countLayer==0){
                port0.prepare() = *img2;
                //if((unsigned int)ptr_inputImage2!=0xcccccccc)
                //	port0.prepare()= *ptr_inputImage2;
                port0.write();
            }
            else if(countLayer==1){
                port1.prepare() = *img2;
                port1.write();
            }
            else if(countLayer==2){
                port2.prepare() = *img2;
                port2.write();
            }
            else if(countLayer==3){
                port3.prepare() = *img2;
                port3.write();
            }

        
            for(int i=0;i<320*240;i++){
                red_tmp[i]=255;
                blue_tmp[i]=255;
                green_tmp[i]=255;
            }

            layerName_pr.assign(layerName);
            countUnit=0;
            ycount=0;
        //}
    }

    */

}


void BMLEngine::evolve2() {
    //---------------------EVOLVE the Boltzmann Machine
    if(runFreely){
        mb->evolveFreely(2,0);
        int value=mb->getTemperature()+COOLING_RATE;
        mb->setTemperature(value);
        count++;
    }
    else if(runClamped){
        mb->evolveClamped(2,1);
        int value=mb->getTemperature()+COOLING_RATE;
        mb->setTemperature(value);
        count++;
    }
    else if(probFreely_flag){
        mb->setProbabilityFreely();
    }
    else if(probClamped_flag){
        mb->setProbabilityClamped();
    }
    if((count>10)&&(count<20)){
        //mb->evolveFreely(2);
    }
    if(false){
            enableDraw=true;
            //equilibrium of the freely mode or clamped mode
            //it is now possible of set the probability either of freely mode or clamped modo
            if(runFreely){
                mb->setProbabilityFreely();
                count=0;
            }
            else if(runClamped){
                mb->setProbabilityClamped();
                count=0;
            }
            //mb->setProbabilityFreely();
    }

    if((count>20)&&(count<30)){
        //mb->evolveClamped(2);
    }
    if(count==30){
        //equilibrium of clampled mode
        //it is now possible of set the probability of clamped mode
        //mb->setProbabilityClamped();
        //LEARNING COMMAND
        //mb->learn();
    }
    if((count>30)&&(count<40)){
        //run freely with input layer clamped
    }
}

void BMLEngine::outLayers(){
    /*
    if(layer0Image!=0){
        if((layer0Image->image2!=0)&&(port0.getOutputCount()!=0)){
            ptr_inputImage=new ImageOf<PixelRgb>;
            ptr_inputImage->resize(100,100);
            cvCvtColor(ptr_inputImageMono->getIplImage(),ptr_inputImage->getIplImage(),CV_GRAY2RGB);
            //port0.prepare()=*(layer0Image->image2);

            port0.prepare()=*ptr_inputImage;
            port0.write();
        }
        if((layer0Image->imageWeights!=0)&&(weight01.getOutputCount()!=0)){
            weight01.prepare()=*(layer0Image->imageWeights);
            weight01.write();
        }
    }
      
    if(this->layer1Image!=0){
        if((layer1Image->image2!=0)&&(port1.getOutputCount()!=0)){
            port1.prepare()=*(layer1Image->image2);
            port1.write();
        }
        if((layer1Image->imageWeights!=0)&&(weight12.getOutputCount()!=0)){
            weight12.prepare()=*(layer1Image->imageWeights);
            weight12.write();
        }    
    }

    if(this->layer2Image!=0){
        if((layer2Image->image2!=0)&&(port2.getOutputCount()!=0)){
            port2.prepare()=*(layer2Image->image2);
            port2.write();
        }
        if((layer2Image->imageWeights!=0)&&(weight23.getOutputCount()!=0)){
            weight23.prepare()=*(layer2Image->imageWeights);
            weight23.write();
        }
    }
    
    */
}



void BMLEngine::setName(const char *name) {
        this->name = (char *) name;
}


/** 
* function that sets the scaleFactorX
* @param value new value of the scaleFactorX
*/
void BMLEngine::setScaleFactorX(int value){
    this->scaleFactorX=value;
}
/** 
* function that sets the scaleFactorY
* @param value new value of the scaleFactorY
*/
void BMLEngine::setScaleFactorY(int value){
    this->scaleFactorY=value;
}



/** 
* function that set the number of the layer active 
* @param value number of the layer actually active
*/
void BMLEngine::setCurrentLayer(int value){
    this->currentLayer=value;
}

void BMLEngine::loadConfiguration(string filename){
    //attach(port); // process input to this port
    string fileName("configuration.ini");
    FILE * pFile;
    int n;
    char name [100];
    cout<<"Checking for the Boltzmann Machine file "<<fileName<<"............"<<endl;
    pFile = fopen (fileName.c_str(),"r");
    if(pFile==NULL){
        cout<<"Creating the Boltzmann Machine from scratch "<<endl;
        mb=new MachineBoltzmann();
        cout<<"Successfully Created the Boltzmann machine"<<endl;
    }
    else{
        cout<<"Loading the Boltzmann Machine from file"<<endl;
        cout<<"Creating a default Boltzmann Machine...."<<endl;
        mb=new MachineBoltzmann();
        cout<<"Loading configuration from file"<<endl;
        mb->loadConfiguration();
        cout<<"Successfully loaded the configuration"<<endl;
    }	
    
    //cout<<"Number of allocated elements:"<< mb->getCountElements()<<endl;
    if(this->countLayer>0)
        enableDraw=true;
    
}

void BMLEngine::addLayer(int number,int colDimension, int rowDimension){
    char number_str[3];
    int n=sprintf(number_str,"%d",number);
    string n_string(number_str,n);
    string layerName("L");
    layerName.append(n_string);
    printf("layerName:%s",layerName.c_str());
    Layer *layer=new Layer(layerName,colDimension,rowDimension);
    mb->addLayer(layer);
    //mb->migrateLayer(*layer);
    //mb->interconnectLayer(number);
    enableDraw=true;
    // allocate the relative threads 	bmlEngine.exe!Layer::Layer(const Layer & __that={...})  + 0xc2 bytes	C++

    if(number==0){
        string name(this->name);
        name.append("/layer0");
        layer0Image=new imageThread(50,name.c_str());
        layer0Image->setLayer(layer);
        layer0Image->start();
    }
    else if(number==1){
        string name(this->name);
        name.append("/layer1");
        layer1Image=new imageThread(50,name.c_str());
        layer1Image->setLayer(layer);
        layer1Image->start();
    }
    else if(number==2){
        string name(this->name);
        name.append("layer2:o");
        layer2Image=new imageThread(50,name.c_str());
        layer2Image->setLayer(layer);
        layer2Image->start();
    }
}

void BMLEngine::clampLayer(int layerNumber){
    int width=320;
    int height=240;
    //IppiSize srcsize={width,height};
    //1.extracts 3 planes
    /*Ipp8u* im_out = ippiMalloc_8u_C1(320,240,&psb);
    Ipp8u* im_tmp[3];
    Ipp8u* im_tmp_tmp[3];
    im_tmp_tmp[0]=ippiMalloc_8u_C1(320,240,&psb);
    im_tmp_tmp[1]=ippiMalloc_8u_C1(320,240,&psb);
    im_tmp_tmp[2]=ippiMalloc_8u_C1(320,240,&psb);
    Ipp8u* im_tmp_red=ippiMalloc_8u_C1(320,240,&psb);
    Ipp8u* im_tmp_green=ippiMalloc_8u_C1(320,240,&psb);
    Ipp8u* im_tmp_blue=ippiMalloc_8u_C1(320,240,&psb);
    Ipp8u* im_tmp_tmp2=ippiMalloc_8u_C1(320,240,&psb);
    im_tmp[0]=ippiMalloc_8u_C1(320,240,&psb);
    im_tmp[1]=ippiMalloc_8u_C1(320,240,&psb);
    im_tmp[2]=ippiMalloc_8u_C1(320,240,&psb);*/
    
    //if((unsigned int)ptr_inputImage==0xcccccccc)
    //	return;
    //ippiCopy_8u_C3P3R(this->ptr_inputImage->getPixelAddress(0,0),320*3,im_tmp,psb,srcsize);
    //ippiCopy_8u_C1R(im_tmp[0],psb,im_tmp_red,psb,srcsize);
    //ippiCopy_8u_C1R(im_tmp[1],psb,im_tmp_green,psb,srcsize);
    //ippiCopy_8u_C1R(im_tmp[2],psb,im_tmp_blue,psb,srcsize);
    
    //2. gets the maximum value between planes
    /*for(int i=0;i<320*240;i++){
        if(im_tmp_red[i]<im_tmp_green[i])
            if(im_tmp_green[i]<im_tmp_blue[i])
                im_out[i]=im_tmp_blue[i];
            else
                im_out[i]=im_tmp_green[i];
        else
            if(im_tmp_red[i]<im_tmp_blue[i])
                im_out[i]=im_tmp_blue[i];
            else
                im_out[i]=im_tmp_red[i];
    }*/
    //ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[0],psb,srcsize);
    //ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[1],psb,srcsize);
    //ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[2],psb,srcsize);
    //ippiCopy_8u_C1C3R(im_out,psb,this->ptr_inputImage2->getPixelAddress(0,0),320*3,srcsize);
    //ippiCopy_8u_P3C3R(im_tmp_tmp,psb,this->ptr_inputImage2->getPixelAddress(0,0),320*3,srcsize);
    
    
    //im_tmp_tmp[0]=im_out;
    //im_tmp_tmp[1]=im_out;
    //im_tmp_tmp[2]=im_out;

    //2.Extract the necessary information
    //finds the starting unit of the layer
    string layerName;
    list<Unit>::iterator iter;
    list<Unit>::iterator iter2;
    int totUnits;
    bool end_loop=false;
    for(iter=mb->_unitList.begin(); !end_loop;iter++){
                printf("unit name:%s,%d \n",iter->getName().c_str(),iter->getState());
                unsigned int posR=iter->getName().find('R');
                layerName=iter->getName().substr(1,posR-1);
                printf("layer name:%s where as looking for %d \n",layerName.c_str(), layerNumber);
                int extractNumber=atoi(layerName.c_str());
                if(extractNumber==layerNumber)
                    end_loop=true;
    }

    //count the number of element in the row
    end_loop=false;
    bool end_loop_layer=false;
    totUnits=0;
    int previous_number=0;
    int totRows=0;
    iter2=iter;
    for(;!end_loop_layer;){
        totUnits=0;
        printf("Counting number of units and number of rows");
        for(;((!end_loop)&&(iter2!=mb->_unitList.end()));iter2++){
            unsigned int posR=iter2->getName().find('R');
            unsigned int posU=iter2->getName().find('U');
            string number_str=iter2->getName().substr(posR+1,posU-posR-1);
            int number=atoi(number_str.c_str());
            if(number!=previous_number){
                end_loop=true;
                previous_number=number;
            }
            totUnits++;
        }
        if(iter2==mb->_unitList.end()){
            totRows++;
            totUnits++;
            break;
    }
    printf("countUnits: %d", totUnits);
    unsigned int posR=iter2->getName().find('R');
    layerName=iter2->getName().substr(1,posR-1);
    printf("layer name:%s \n",layerName.c_str());
    int extractNumber=atoi(layerName.c_str());
    if(extractNumber!=layerNumber)
        end_loop_layer=true;
    end_loop=false;
    totRows++;
    
    }

    printf("rowDimension %d \n",totRows);


    //3.maps the intensity on to the layer
    // it does not consider the hidden units of the layer
    int rectDimX=320/(totUnits-2);
    int rectDimY=240/(totRows-2);
    int sum=0;
    for(int boltzmannMachineRow=0;boltzmannMachineRow<totRows-2;boltzmannMachineRow++)
        for(int boltzmannMachineCol=0;boltzmannMachineCol<totUnits-2;boltzmannMachineCol++){
            sum=0;
            for(int y=0;y<rectDimY;y++){
                for(int x=0;x<rectDimX;x++){
                    //sum+=im_out[boltzmannMachineRow*rectDimY*320+boltzmannMachineCol*rectDimX+x+320*y];
                }
            }
            float mean=sum/(rectDimX*rectDimY);
            printf("mean of the unit %f ----> ",mean);
            //set the threashold to decide whether the unit has to be elicited
            if(mean>clampingThreshold){
                mb->addClampedUnit(boltzmannMachineCol+1+(boltzmannMachineRow+1)*totUnits+layerNumber*totUnits*totRows,1);
            }
            else{
                mb->addClampedUnit(boltzmannMachineCol+1+(boltzmannMachineRow+1)*totUnits+layerNumber*totUnits*totRows,0);
            }
        
        }
    


    //4. free memory
    //ippiFree(im_out);
    //ippiFree(im_tmp);
    //ippiFree(im_tmp_tmp);
}

void BMLEngine::clampLayer(Layer* layer){
    int width=ptr_inputImage->width();
    int height=ptr_inputImage->height();
    //IppiSize srcsize={width,height};
    //1.extracts 3 planes
    IplImage* im_tmp_ipl = cvCreateImage( cvSize(width,height), 8, 1 );
    //Ipp8u* im_out = ippiMalloc_8u_C1(320,240,&psb);
    //Ipp8u* im_tmp[3];
    //Ipp8u* im_tmp_tmp[3];
    //im_tmp_tmp[0]=ippiMalloc_8u_C1(320,240,&psb);
    //im_tmp_tmp[1]=ippiMalloc_8u_C1(320,240,&psb);
    //im_tmp_tmp[2]=ippiMalloc_8u_C1(320,240,&psb);
    //Ipp8u* im_tmp_red=ippiMalloc_8u_C1(320,240,&psb);
    //Ipp8u* im_tmp_green=ippiMalloc_8u_C1(320,240,&psb);
    //Ipp8u* im_tmp_blue=ippiMalloc_8u_C1(320,240,&psb);
    //Ipp8u* im_tmp_tmp2=ippiMalloc_8u_C1(320,240,&psb);
    //im_tmp[0]=ippiMalloc_8u_C1(320,240,&psb);
    //im_tmp[1]=ippiMalloc_8u_C1(320,240,&psb);
    //im_tmp[2]=ippiMalloc_8u_C1(320,240,&psb);
    
    if(!inputImage_flag)
        return;
    cvCvtColor(ptr_inputImage->getIplImage(),im_tmp_ipl,CV_RGB2GRAY);
    //ippiCopy_8u_C3P3R(this->ptr_inputImage->getPixelAddress(0,0),320*3,im_tmp,psb,srcsize);
    //ippiCopy_8u_C1R(im_tmp[0],psb,im_tmp_red,psb,srcsize);
    //ippiCopy_8u_C1R(im_tmp[1],psb,im_tmp_green,psb,srcsize);
    //ippiCopy_8u_C1R(im_tmp[2],psb,im_tmp_blue,psb,srcsize);
    //2. gets the maximum value between planes
    
    /*for(int i=0;i<320*240;i++){
        if(im_tmp_red[i]<im_tmp_green[i])
            if(im_tmp_green[i]<im_tmp_blue[i])
                im_out[i]=im_tmp_blue[i];
            else
                im_out[i]=im_tmp_green[i];
        else
            if(im_tmp_red[i]<im_tmp_blue[i])
                im_out[i]=im_tmp_blue[i];
            else
                im_out[i]=im_tmp_red[i];
    }*/
    //ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[0],psb,srcsize);
    //ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[1],psb,srcsize);
    //ippiCopy_8u_C1R(im_out,psb,im_tmp_tmp[2],psb,srcsize);
    //ippiCopy_8u_C1C3R(im_out,psb,this->ptr_inputImage2->getPixelAddress(0,0),320*3,srcsize);
    //ippiCopy_8u_P3C3R(im_tmp_tmp,psb,this->ptr_inputImage2->getPixelAddress(0,0),320*3,srcsize);
    //cvCopy(im_tmp_ipl,ptr_inputImage2->getIplImage());
    
    //im_tmp_tmp[0]=im_out;
    //im_tmp_tmp[1]=im_out;
    //im_tmp_tmp[2]=im_out;

    //2.Extract the necessary information
    int dim=layer->stateVector->length();
    int totRows=layer->getRow();
    int totUnits=layer->getCol();

    printf("state vector dimension %d \n",dim);
    printf("layer number cols %d \n",totUnits);
    printf("layer number rows %d \n",totRows);


    //3.maps the intensity on to the layer
    int rectDimX=width/totUnits;
    int rectDimY=height/totRows;
    int sum=0;
    uchar* data=(uchar*)(im_tmp_ipl->imageData);
    int step       = im_tmp_ipl->widthStep/sizeof(uchar);
    //printf("step of the gray image as input %d",step);

    for(int boltzmannMachineRow=0;boltzmannMachineRow<totRows;boltzmannMachineRow++)
        for(int boltzmannMachineCol=0;boltzmannMachineCol<totUnits;boltzmannMachineCol++){
            sum=0;
            for(int y=0;y<rectDimY;y++){
                for(int x=0;x<rectDimX;x++){
                    sum+=data[boltzmannMachineRow*rectDimY*width+boltzmannMachineCol*rectDimX+width*y+x];
                    //sum+=im_tmp_ipl[boltzmannMachineRow*rectDimY*320+boltzmannMachineCol*rectDimX+320*y+x];
                }
            }
            double mean=sum/(rectDimX*rectDimY);
            //printf("mean of the unit %f ----> ",mean);
            //set the threashold to decide whether the unit has to be elicite
            (*layer->stateVector)(boltzmannMachineCol+boltzmannMachineRow*totUnits)=mean/255;
        }
    

    //4. free memory
    //ippiFree(im_out);
    //ippiFree(im_tmp);
    //ippiFree(im_tmp_tmp);
}

bool BMLEngine::respond(const Bottle &command,Bottle &reply){
        
    bool ok = false;
    bool rec = false; // is the command recognized?

    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("help");

            reply.addString("\n");
            reply.addString("get fn \t: general get command \n");
            

            reply.addString("\n");
            reply.addString("set s1 <s> \t: general set command \n");

            reply.addString("\n");
            reply.addString("shut down : run the rgb processor \n");
            reply.addString("run yuv : run the yuv processor");
            

            reply.addString("\n");


            ok = true;
        }
        break;
    case COMMAND_VOCAB_NAME:
        rec = true;
        {
            // check and change filter name to pass on to the next filter
            string fName(command.get(1).asString());
            string subName;
            Bottle subCommand;
            int pos=1;
            //int pos = fName.find_first_of(filter->getFilterName());
            if (pos == 0){
                pos = fName.find_first_of('.');
                if (pos  > -1){ // there is a subfilter name
                    subName = fName.substr(pos + 1, fName.size()-1);
                    subCommand.add(command.get(0));
                    subCommand.add(Value(subName.c_str()));
                }
                for (int i = 2; i < command.size(); i++)
                    subCommand.add(command.get(i));
                //ok = filter->respond(subCommand, reply);
            }
            else{
                printf("filter name  does not match top filter name ");
                ok = false;
            }
        }
        break;
    case COMMAND_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_SALIENCE_THRESHOLD:{
                double thr = command.get(2).asDouble();
            }
                break;
            case COMMAND_VOCAB_NUM_BLUR_PASSES:{
                int nb = command.get(2).asInt();
                //reply.addString("connection 2");
              
                ok=true;
            }
                break;
            /*case COMMAND_VOCAB_TEMPORAL_BLUR:{
                int size = command.get(2).asInt();
                ok = this->setTemporalBlur(size);
            }*/
                break;
            case COMMAND_VOCAB_NAME:{
                string s(command.get(2).asString().c_str());
                reply.addString("connection 1");
                ok=true;
            }
                break;
            case COMMAND_VOCAB_CHILD_NAME:{
                int j = command.get(2).asInt();
                string s(command.get(3).asString().c_str());
            }
                break;
            case COMMAND_VOCAB_WEIGHT:{
                double w = command.get(2).asDouble();
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHT:{
                int j = command.get(2).asInt();
                double w = command.get(3).asDouble();
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHTS:{
                Bottle weights;
                for (int i = 2; i < command.size(); i++)
                    weights.addDouble(command.get(i).asDouble());
            }
                break;
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_SET" << endl;
                break;
            }
        }
        break;
     case COMMAND_VOCAB_RUN:
        rec = true;
        {
            
        }
        break;
    case COMMAND_VOCAB_GET:
        rec = true;
        {
            reply.addVocab(COMMAND_VOCAB_IS);
            reply.add(command.get(1));
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_SALIENCE_THRESHOLD:{
                double thr=0.0;
                reply.addDouble(thr);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_NUM_BLUR_PASSES:{
                int nb = 0;
                reply.addInt(nb);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_NAME:{
                string s(" ");
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_NAME:{
                int j = command.get(2).asInt();
                string s(" ");
                reply.addString(s.c_str());
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_COUNT:{
                int count =0;
                reply.addInt(count);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_WEIGHT:{
                double w = 0.0;
                reply.addDouble(w);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHT:{
                int j = command.get(2).asInt();
                double w = 0.0;
                reply.addDouble(w);
                ok = true;
            }
                break;
            case COMMAND_VOCAB_CHILD_WEIGHTS:{
                Bottle weights;
                //ok = filter->getChildWeights(&weights);
                for (int k = 0; k < weights.size(); k++)
                    reply.addDouble(0.0);
            }
                break;
            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;

    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;
} 


void BMLEngine::addSample() {
    int totRows=mb->getLayer(0)->getRow();
    int totUnits=mb->getLayer(0)->getCol();
    const int dim=totRows * totUnits;
    Vector sample(dim);
    
    int width,height;
    IplImage* im_tmp_ipl;
    if(inputImage_flag){
        if(inputColour) {
            //1.extracts 3 planes
            width=ptr_inputImage->width();
            height=ptr_inputImage->height();
            im_tmp_ipl = cvCreateImage( cvSize(width,height), 8, 1 );
            cvCvtColor(ptr_inputImage->getIplImage(),im_tmp_ipl,CV_RGB2GRAY);
        }
        else {
            //2.copying one channel planes
            width=ptr_inputImageMono->width();
            height=ptr_inputImageMono->height();
            im_tmp_ipl = cvCreateImage( cvSize(width,height), 8, 1 );
            cvCopy(ptr_inputImageMono->getIplImage(),im_tmp_ipl);
        }
        
        //2.Extract the necessary information
        printf("state vector dimension %d \n",dim);
        printf("layer number cols %d \n",totUnits);
        printf("layer number rows %d \n",totRows);


        //3.maps the intensity on to the layer
        int rectDimX=width/totUnits;
        int rectDimY=height/totRows;
        int sum=0;
        uchar* data=(uchar*)(im_tmp_ipl->imageData);
        int step       = im_tmp_ipl->widthStep/sizeof(uchar);
        //printf("step of the gray image as input %d",step);

        for(int r=0;r<totRows;r++){
            for(int c=0;c<totUnits;c++){
                sum=0;
                for(int y=0;y<rectDimY;y++){
                    for(int x=0;x<rectDimX;x++){
                        sum+=data[r*rectDimY*width+c*rectDimX+width*y+x];
                        //sum+=im_tmp_ipl[boltzmannMachineRow*rectDimY*320+boltzmannMachineCol*rectDimX+320*y+x];
                    }
                }
                double mean=sum/(rectDimX*rectDimY);
                sample[r*totUnits+c]=mean/255;
                printf("%2.2f,",mean/255);
            }
            printf("\n");
        }
        mb->addSample(sample);
    }
}


void BMLEngine::startMakeBatches() {
    printf("starting make batches \n");
    if(dataCollector==0) {
        printf("null data collector \n");
        dataCollector = new dataCollectorThread();
        dataCollector->setName(getName().c_str());
        dataCollector->setInputData(&portMono);
        int totRows = mb->getLayer(0)->getRow();
        printf(" tot rows in the layer 0 %d \n", totRows);
        dataCollector->setMachineBoltzmann(mb);
        dataCollector->start();
    }
    else {
        printf("The thread has been already instantiated. ! \n");
        printf("The instantiated thread will be resumed \n");
        delete dataCollector;
        dataCollector=new dataCollectorThread();
        dataCollector->setName(getName().c_str());
        dataCollector->setInputData(&portMono);
        dataCollector->setMachineBoltzmann(mb);
        dataCollector->start();
    }
}

void BMLEngine::stopMakeBatches() {
    if(dataCollector!=0) {
        dataCollector->setInhibit(true);
        //dataCollector->stop();
    }
}

