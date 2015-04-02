// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <iCub/BMLInterface.h>



//YARP include
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

//openCV include
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//within Project Include
//#include <iCub/ImageProcessor.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


BMLInterface::BMLInterface(){
    this->command=new string("");
    this->image_out=new ImageOf<PixelRgb>;
    image_out->resize(320,240);
    //---
    /*meanColour_flag=true;
    contrastLP_flag=false;
    blobCataloged_flag=true;*/

    _pOutPort=new BufferedPort<Bottle>;
    this->rowDim=10;
    this->colDim=10;
}

BMLInterface::~BMLInterface(){
    delete image_out;
    delete _pOutPort;
}




// try to interrupt any communications or resource usage
bool BMLInterface::interruptModule() {
    port_in.interrupt();
    port_out.interrupt();
    return true;
}

bool BMLInterface::close() {
        closePorts();
        if(gui!=0){
            gui->close();
            //delete gui;
        }
        return true;
    }

void BMLInterface::setOptions(yarp::os::Property opt){
    options	=opt;
}

bool BMLInterface::updateModule() {    
    outPorts();
    return true;
}




void BMLInterface::setRowDim(int number){
    this->rowDim=number;
}

void BMLInterface::setColDim(int number){
    this->colDim=number;
}

bool BMLInterface::outPorts(){
    bool ret = true;
    //IplImage *cvImage = cvCreateImage(cvSize(320,240),8, 3);
    //cvCvtColor((IplImage*)outMeanColourLP->getIplImage(), cvImage, CV_BGR2RGB);
    //image_out->wrapIplImage(cvImage);
    /*int psb;
    IppiSize srcsize={320,240};
    //printf("Entered in outPorts \n");
    Ipp8u* im_out = ippiMalloc_8u_C1(320,240,&psb);
    Ipp8u* im_tmp[3];
    Ipp8u* im_tmp_tmp= ippiMalloc_8u_C1(320,240,&psb);
    im_tmp[0]=im_out;
    im_tmp[1]=im_out;
    im_tmp[2]=im_out;
    ippiCopy_8u_C3P3R(this->outMeanColourLP->getPixelAddress(0,0),320*3,im_tmp,psb,srcsize);
    ippiCopy_8u_C1R(im_tmp[2],psb,im_tmp_tmp,psb,srcsize);
    ippiCopy_8u_C1R(im_tmp[0],psb,im_tmp[2],psb,srcsize);
    ippiCopy_8u_C1R(im_tmp[2],psb,blobFov->getPixelAddress(0,0),psb,srcsize);*/
    //ippiCopy_8u_P3C3R(im_tmp,psb,image_out->getPixelAddress(0,0),320*3,srcsize);
    //this->_pOutPort2->prepare()=*(this->image_out);

    //this->_pOutPort2->prepare()=*(this->image_out);
    //this->_pOutPort2->write();


    //this->_pOutPort2->write();
    //this->_pOutPort3->write();
    //ippiFree(im_out);
    //ippiFree(im_tmp_tmp);
    //ippiFree(im_tmp);
    if(strcmp(command->c_str(),"")){
        string optionValue1, optionValue2, tag;
        string option;
        Bottle& outBot1=_pOutPort->prepare();
        outBot1.clear();
        if(!bOptions.isNull()){
            outBot1.addString(command->c_str());
            outBot1.append(bOptions);
            //outBot1.append(bOptions);
        
        }

        /*size_t found1, found2;
        found1=command->find("(");
        if(found1!=string::npos){
            tag=command->substr(0,found1-1);
            option=command->substr(found1+1,command->size()-found1);
            
            
            unsigned int  parPos1=option.find("(");
            unsigned int parPos2=option.find(")");
            unsigned int spacePos=option.find(" ");
            if(spacePos!=-1){
                printf("Presence of a space detected \n");
                optionValue1=option.substr(parPos1+1,spacePos-parPos1);
                optionValue2= option.substr(spacePos+1,parPos2-spacePos-1);
                   
            
            }
            
            bOptions.addString(optionValue1.c_str());
            bOptions.addString(optionValue2.c_str()); 
            
            //outBot1.addString("to");
        }*/
        else{
            outBot1.addString(command->c_str());
        }
        //outBot1.addList()=bOptions;

        _pOutPort->write();
        command->clear();
        bOptions.clear();
    }
    return ret;
}



/*void BMLInterface::deleteObjects() {
    delete ptr_imgRecv;
    delete ptr_inputImg;
    delete ptr_semaphore;
}*/

bool BMLInterface::openPorts(){
    cmdPort.open(getName("cmd")); // optional command port
    attach(cmdPort); // cmdPort will work just like terminal
    _pOutPort->open(getName("command:o")); // optional command port
    return true;
}

bool BMLInterface::closePorts(){
    port_in.close();
    port_out.close();	
    return true;
}





//static GtkWidget *mainWindow = NULL;
bool BMLInterface::open(Searchable& config) {
    //ct = 0;
    //port_in.open(getName("in"));
    //ConstString portName2 = options.check("name",Value("/worker2")).asString();
    //port_out.open(getName("out"));
    //port_Blobs.open(getName("outBlobs"));
    openPorts();
    gui=new graphicThread();
    gui->setImageProcessModule(this);
    gui->start();

    

    return true;	
}



