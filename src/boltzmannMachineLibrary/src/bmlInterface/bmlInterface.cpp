// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
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

#include "bmlInterface.h"

#include <yarp/os/Property.h> 
#include <yarp/os/Network.h> 
#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Bottle.h>

#include <string>
#include <sstream>
#include <cstring>
#include <cmath>

using namespace yarp::os;
using namespace std;

GtkWidget *mainWindow=0;
static GdkPixbuf *frame = NULL;

BufferedPort<yarp::sig::FlexImage> *ptr_inputPort=0;
std::string* command;   // reference to the string refering to the last command to send
int _frameN;            // Frame Number
bool _savingSet;        //Save Set of Images mode
yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort=0;      //Output Point Port
yarp::os::Bottle _outBottle;                                //Output Bottle Container

pgmOptions _options;

GtkWidget *saveSingleDialog;
GtkWidget *saveSetDialog;
GtkWidget *menubar;
GtkWidget *fileMenu, *imageMenu, *helpMenu;
GtkWidget *fileItem, *helpItem;
GtkWidget *fileSingleItem, *fileSetItem, *fileQuitItem;
GtkWidget *imageSizeItem, *imageRatioItem, *imageFreezeItem, *imageFramerateItem;
GtkWidget *synchroDisplayItem;
GtkWidget *helpAboutItem;
// StatusBar
GtkWidget *statusbar;
GtkWidget *fpsStatusBar;
GtkWidget *fpsStatusBar2;

guint timeout_ID=0;
guint timeout_update_ID=0;

ViewerResources _resources;
double timeCentroid;               //time costant for the centroid ouput
double targetRed;                  //colour red for the target object
double targetBlue;                 //colour green for the target object
double targetGreen;                //colour blue for the target object
double salienceTD;                 //saliency top-down coefficient
double salienceBU;                  //saliency bottom-up coefficient
double reactivity;                 //time costant
double minBoundingArea;            //min dimension of the blob
int minBLOB;                    //min dimension of the blob
int maxBLOB;                    //max dimension of the blob
double targetRED;                  //colour RER for the target
double targetGREEN;                //colour GREEN for the target
double targetBLUE;                  //colour BLUE for the target

bool watershed_flag;            //flag watershed
bool tagged_flag;               //flag for the tag
bool meanColour_flag;           //flag minColour
bool maxSaliencyBlob_flag;      //flag for maxSaliencyBlob
bool blobList_flag;             //flag for the blobList
bool colorVQ_flag;              //flag for the colorVQ
bool contrastLP_flag;           //flag contrastLP (saliency map)
bool foveaBlob_flag;            //flag for foveaBlob

static void createObjects() {
    ptr_inputPort = new BufferedPort<yarp::sig::FlexImage>;
    message=new string("");
    //ptr_portCallback = new InputCallback;
}

static void deleteObjects() {
    /*if (ptr_inputPort!=0)
        delete ptr_inputPort;
    if (ptr_portCallback!=0)
        delete ptr_portCallback;*/
}




//-------------------------------------------------
// Main Window Callbacks
//-------------------------------------------------
/**
* usual callback function 
*/
static void callback( GtkWidget *widget,gpointer   data ){
    printf ("Hello again - %s was pressed \n", (char *) data);

    message=new string();

    if(!strcmp((char *)data,"Execute")){
        printf("Execute");
        
        /*
        if(wModule->runFreely_flag)
            _command.assign("ExecuteFreely");
        else if(wModule->runClamped_flag)
            _command.assign("ExecuteClamped");
        else if(wModule->stopEvolution_flag)
            _command.assign("ExecuteStop");
         */
        //wModule->command->assign(_command); 
    }
    else if(!strcmp((char *)data,"Learn")){
        printf("Learn");
        string _command("Learn");
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"outHeadBehaviour")){
        printf("outHeadBehaviour");
        string _command("outHeadBehaviour");
        //wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"outEyesBehaviour")){
        printf("outEyesBehaviour");
        string _command("outEyesBehaviour");
        //wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"EvolveFreely")){
        printf("EvolveFreely");
        string _command("EvolveFreely");
        //wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"EvolveClamped")){
        printf("EvolveClamped");
        string _command("EvolveClamped");
        //wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"Stop")){
        printf("Stop");
        string _command("Stop");
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"Load")){
        printf("Load");
        string _command("Load");
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"AddLayer")){
        printf("AddLayer request \n");
        string _command("AddLayer");
        yarp::os::Bottle tmp;
        tmp.addString("row");
        tmp.addInt(rowDim);
        bOptions.clear();
        bOptions.addList()=tmp;
        tmp.clear();
        tmp.addString("col");
        tmp.addInt(colDim);
        bOptions.addList()=tmp;
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"ConnectLayer")){
        printf("ConnectLayer request \n");
        string _command("ConnectLayer");
        string Aname("");
        string Bname("");
        
        if(inLayer0_flag){
            printf("LayerA: layer0 \n");
            Aname.append("layer0");
        }
        else if(inLayer1_flag){
            printf("LayerA: layer1 \n");
            Aname.append("layer1");
        }

        if(SelectLayer0_flag){
            printf("LayerB: layer0 \n");
            Bname.append("layer0");
        }
        else if(SelectLayer1_flag){
            printf("LayerB: layer1 \n");
            Bname.append("layer1");
        }
        
        Bottle tmp;
        bOptions.clear();
        tmp.addString("LayerA");
        tmp.addString(Aname.c_str());
        bOptions.addList()=tmp;
        tmp.clear();
        tmp.addString("LayerB");
        tmp.addString(Bname.c_str());
        bOptions.addList()=tmp;
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"ClampPattern")){
        printf("ClampPattern \n");
        string _command("ClampPattern");
        Bottle tmp;
        tmp.addString("pattern");
        tmp.addString("1,0,0,0");
        bOptions.addList()=tmp;
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"ClampLayer")){
        printf("ClampLayer \n");
        string _command("ClampLayer");
        Bottle tmp;
        tmp.addString("layer");
        
        if(inLayer0_flag)
            tmp.addString("layer0");
        else if(inLayer1_flag)
            tmp.addString("layer1");
        bOptions.addList()=tmp;
        message->assign(_command);
        
    }
    else if(!strcmp((char *)data,"setProbabilityFreely")){
        printf("setProbabilityFreely \n");
        string _command("setProbabilityFreely");
        //wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"setProbabilityNull")){
        printf("setProbabilityNull \n");
        string _command("setProbabilityNull");
        //wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"setProbabilityClamped")){
        printf("setProbabilityClamped \n");
        string _command("setProbabilityClamped");
        //wModule->command->assign(_command);
    }
    else if(!strcmp((char *)data,"addSample")){
        printf("addSample \n");
        string _command("addSample");
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"dataSet:start")){
        printf("dataSet start populating \n");
        string _command("dataSetStart");
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"dataSet:stop")){
        printf("dataSet stop populating \n");
        string _command("dataSetStop");
        message->assign(_command);
    }
    else if(!strcmp((char *)data,"drawFoveaBlob3")){
        printf("drawFoveaBlob3");
    }
    else if(!strcmp((char *)data,"drawVQColor1")){
        printf("drawColorVQ1 function");
    }
    else if(!strcmp((char *)data,"drawVQColor2")){
        printf("drawFoveaBlob2");
    }
    else if(!strcmp((char *)data,"drawVQColor3")){
        printf("drawFoveaBlob3");
    }
    else if(!strcmp((char *)data,"maxSalienceBlob1")){
        printf("drawColorVQ1 function");
    }
    else if(!strcmp((char *)data,"maxSalienceBlob2")){
        printf("drawFoveaBlob2");
    }
    else if(!strcmp((char *)data,"maxSalienceBlob3")){
        printf("drawFoveaBlob3");
    }


    if(strcmp(message->c_str(),"")){
        string optionValue1, optionValue2, tag;
        string option;
        Bottle& outBot1=_pOutPort->prepare();
        outBot1.clear();
        if(!bOptions.isNull()){
            outBot1.addString(message->c_str());
            outBot1.append(bOptions);
        }
        else {
            outBot1.addString(message->c_str());
        }

        _pOutPort->write();
        message->clear();
        bOptions.clear();
    }
}

static void cb_draw_value( GtkToggleButton *button )
{
    /* Turn the value display on the scale widgets off or on depending
     *  on the state of the checkbutton */
    printf("callbacks from draw value %s \n",button->button.label_text);
    string _command;
    if(!strcmp(button->button.label_text,"inputImage")){
        printf("inputImage request \n");
        if(button->active){
            inputImage_flag=true;
        }
        else {
            inputImage_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"B:=Layer0")){
        printf("Select Layer0 \n");
        if(button->active){
            SelectLayer0_flag=true;
            SelectLayer1_flag=false;
            SelectLayer2_flag=false;
            SelectLayer3_flag=false;
            SelectLayer4_flag=false;
            SelectLayer5_flag=false;
            SelectLayer6_flag=false;
            SelectLayer7_flag=false;
            SelectLayer8_flag=false;
        }
        else {
            SelectLayer0_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"B:=Layer1")){
        printf("Select B: Layer1 \n");
        if(button->active){
             SelectLayer0_flag=false;
             SelectLayer1_flag=true;
             SelectLayer2_flag=false;
             SelectLayer3_flag=false;
             SelectLayer4_flag=false;
             SelectLayer5_flag=false;
             SelectLayer6_flag=false;
             SelectLayer7_flag=false;
             SelectLayer8_flag=false;
            
        }
        else {
             SelectLayer1_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"B:=Layer2")){
        printf("Select B: Layer2 \n");
        if(button->active){
            
             SelectLayer0_flag=false;
             SelectLayer1_flag=false;
             SelectLayer2_flag=true;
             SelectLayer3_flag=false;
             SelectLayer4_flag=false;
             SelectLayer5_flag=false;
             SelectLayer6_flag=false;
             SelectLayer7_flag=false;
             SelectLayer8_flag=false;
            
        }
        else {
             SelectLayer2_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"B:=Layer3")){
        printf("Select B Layer3 \n");
        if(button->active){
             SelectLayer0_flag=false;
             SelectLayer1_flag=false;
             SelectLayer2_flag=false;
             SelectLayer3_flag=true;
             SelectLayer4_flag=false;
             SelectLayer5_flag=false;
             SelectLayer6_flag=false;
             SelectLayer7_flag=false;
             SelectLayer8_flag=false;
            
        }
        else {
             SelectLayer3_flag=true;
        }
    }
    else if(!strcmp(button->button.label_text,"B:=Layer4")){
        printf("Select B: Layer4 \n");
        if(button->active){
            
             SelectLayer0_flag=false;
             SelectLayer1_flag=false;
             SelectLayer2_flag=false;
             SelectLayer3_flag=false;
             SelectLayer4_flag=true;
             SelectLayer5_flag=false;
             SelectLayer6_flag=false;
             SelectLayer7_flag=false;
             SelectLayer8_flag=false;
            
        }
        else {
             SelectLayer4_flag=false;
        }

    }
    else if(!strcmp(button->button.label_text,"B:=Layer5")){
        printf("Select B: Layer5 \n");
        if(button->active){
            
             SelectLayer0_flag=false;
             SelectLayer1_flag=false;
             SelectLayer2_flag=false;
             SelectLayer3_flag=false;
             SelectLayer4_flag=false;
             SelectLayer5_flag=true;
             SelectLayer6_flag=false;
             SelectLayer7_flag=false;
             SelectLayer8_flag=false;
            
        }
        else {
             SelectLayer5_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"B:=Layer6")){
        printf("Select B: Layer6 \n");
        if(button->active){
            
             SelectLayer0_flag=false;
             SelectLayer1_flag=false;
             SelectLayer2_flag=false;
             SelectLayer3_flag=false;
             SelectLayer4_flag=false;
             SelectLayer5_flag=false;
             SelectLayer6_flag=true;
             SelectLayer7_flag=false;
             SelectLayer8_flag=false;
            
        }
        else {
             SelectLayer6_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"B:=Layer7")){
        printf("Select B: Layer7 \n");
        if(button->active){
            
             SelectLayer0_flag=false;
             SelectLayer1_flag=false;
             SelectLayer2_flag=false;
             SelectLayer3_flag=false;
             SelectLayer4_flag=false;
             SelectLayer5_flag=false;
             SelectLayer6_flag=false;
             SelectLayer7_flag=true;
             SelectLayer8_flag=false;
            
        }
        else {
             SelectLayer7_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"B:=Layer8")){
        printf("Select B: Layer8 \n");
        if(button->active){
            
             SelectLayer0_flag=false;
             SelectLayer1_flag=false;
             SelectLayer2_flag=false;
             SelectLayer3_flag=false;
             SelectLayer4_flag=false;
             SelectLayer5_flag=false;
             SelectLayer6_flag=false;
             SelectLayer7_flag=false;
             SelectLayer8_flag=true;
            
        }
        else {
             SelectLayer8_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"A:=Layer0")){
        printf("A:=Layer0 request \n");
        if(button->active){
            
             inLayer0_flag=true;
             inLayer1_flag=false;
             inLayer2_flag=false;
             inLayer3_flag=false;
             inLayer4_flag=false;
             inLayer5_flag=false;
             inLayer6_flag=false;
             inLayer7_flag=false;
             inLayer8_flag=false;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(0);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldwModule->bOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
             inLayer0_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"A:=Layer1")){
        printf("A:=Layer1 request \n");
        if(button->active){
             inLayer0_flag=false;
             inLayer1_flag=true;
             inLayer2_flag=false;
             inLayer3_flag=false;
             inLayer4_flag=false;
             inLayer5_flag=false;
             inLayer6_flag=false;
             inLayer7_flag=false;
             inLayer8_flag=false;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(1);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldwModule->bOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
             inLayer1_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"A:=Layer2")){
        printf("A:=Layer2 request \n");
        if(button->active){
             inLayer0_flag=false;
             inLayer1_flag=false;
             inLayer2_flag=true;
             inLayer3_flag=false;
             inLayer4_flag=false;
             inLayer5_flag=false;
             inLayer6_flag=false;
             inLayer7_flag=false;
             inLayer8_flag=false;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(2);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldwModule->bOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
             inLayer2_flag=false;
        }
    }
    if(!strcmp(button->button.label_text,"A:=Layer3")){
        if(button->active){
             inLayer0_flag=false;
             inLayer1_flag=false;
             inLayer2_flag=false;
             inLayer3_flag=true;
             inLayer4_flag=false;
             inLayer5_flag=false;
             inLayer6_flag=false;
             inLayer7_flag=false;
             inLayer8_flag=false;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(3);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldbOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
             inLayer3_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"A:=Layer4")){
        if(button->active){
             inLayer0_flag=false;
             inLayer1_flag=false;
             inLayer2_flag=false;
             inLayer3_flag=false;
             inLayer4_flag=true;
             inLayer5_flag=false;
             inLayer6_flag=false;
             inLayer7_flag=false;
             inLayer8_flag=false;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(4);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldwModule->bOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
             inLayer4_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"A:=Layer5")){
        if(button->active){
             inLayer0_flag=false;
             inLayer1_flag=false;
             inLayer2_flag=false;
             inLayer3_flag=false;
             inLayer4_flag=false;
             inLayer5_flag=true;
             inLayer6_flag=false;
             inLayer7_flag=false;
             inLayer8_flag=false;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(5);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldwModule->bOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
             inLayer5_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"A:=Layer6")){
        if(button->active){
            inLayer0_flag=false;
            inLayer1_flag=false;
            inLayer2_flag=false;
            inLayer3_flag=false;
            inLayer4_flag=false;
            inLayer5_flag=false;
            inLayer6_flag=true;
            inLayer7_flag=false;
            inLayer8_flag=false;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(6);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldwModule->bOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
             inLayer6_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"A:=Layer7")){
        if(button->active){
            inLayer0_flag=false;
            inLayer1_flag=false;
            inLayer2_flag=false;
            inLayer3_flag=false;
            inLayer4_flag=false;
            inLayer5_flag=false;
            inLayer6_flag=false;
            inLayer7_flag=true;
            inLayer8_flag=false;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(7);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldwModule->bOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
             inLayer7_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"A:=Layer8")){
        if(button->active){
            inLayer0_flag=false;
            inLayer1_flag=false;
            inLayer2_flag=false;
            inLayer3_flag=false;
            inLayer4_flag=false;
            inLayer5_flag=false;
            inLayer6_flag=false;
            inLayer7_flag=false;
            inLayer8_flag=true;
            
            string _command("CurrentLayer");
            Bottle tmp;
            tmp.addString("value");
            tmp.addInt(8);
            bOptions.addList()=tmp;
            tmp.clear();
            //_oldtmp.addString("col");
            //_oldtmp.addInt(wModule->colDim);
            //_oldwModule->bOptions.addList()=tmp;
            message->assign(_command);
        }
        else {
            inLayer8_flag=false;
        }
    }
    if(!strcmp(button->button.label_text,"EvolveFreely-->")){
        if(button->active) {
            //wModule->runFreely_flag=true;
        }
        else {
            //wModule->runFreely_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"EvolveClamped-->")){
        if(button->active) {
            //wModule->runClamped_flag=true;
        }
        else {
            //wModule->runClamped_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"StopEvolution")){
        if(button->active) {
            //wModule->stopEvolution_flag=true;
        }
        else {
            //wModule->stopEvolution_flag=false;
        }
    }
    else if(!strcmp(button->button.label_text,"Blue2-->")){
        if(button->active){
            //imageProcessModule->processor2->redPlane_flag=0;
            //imageProcessModule->processor2->greenPlane_flag=0;
            //imageProcessModule->processor2->bluePlane_flag=1;
        }
    }
    if(!strcmp(button->button.label_text,"Red3-->")){
        if(button->active){
            //imageProcessModule->processor3->redPlane_flag=1;
            //imageProcessModule->processor3->greenPlane_flag=0;
            //imageProcessModule->processor3->bluePlane_flag=0;
        }
    }
    else if(!strcmp(button->button.label_text,"Green3-->")){
        if(button->active){
            //imageProcessModule->processor3->redPlane_flag=0;
            //imageProcessModule->processor3->greenPlane_flag=1;
            //imageProcessModule->processor3->bluePlane_flag=0;
        }

    }
    else if(!strcmp(button->button.label_text,"Blue3-->")){
        if(button->active){
            //imageProcessModule->processor3->redPlane_flag=0;
            //imageProcessModule->processor3->greenPlane_flag=0;
            //imageProcessModule->processor3->bluePlane_flag=1;
        }
    }
}



//-------------------------------------------------
// Call Backs
//-------------------------------------------------
static void cb_digits_scale( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    salienceBU=adj->value/100;
    //printf("salienceBU: %f",salienceBU);
    std::string str("");
    sprintf((char *)str.c_str(),"set kbu %2.2f",salienceBU);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

/*
static void cb_digits_scale2( GtkAdjustment *adj )
{
    salienceTD=adj->value/100;
    //printf("salienceTD: %f",salienceTD);
    std::string str("");
    sprintf((char *)str.c_str(),"set ktd %2.2f",salienceTD);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}
*/

static void cb_digits_scale3( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    maxBLOB=(int)floor(adj->value);
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_MAXDB);
        bot.addInt(maxBLOB);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scale4( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    minBLOB=(int)floor(adj->value);
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_MINDB);
        bot.addInt(minBLOB);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaler( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    targetRED=adj->value;
    //printf("targetRED: %f",targetRED);
    std::string str("");
    sprintf((char *)str.c_str(),"set rin %2.2f",targetRED);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaleg( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    targetGREEN=adj->value;
    //printf("targetGREEN: %f",targetGREEN);
    std::string str("");
    sprintf((char *)str.c_str(),"set gin %2.2f",targetGREEN);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaleb( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    targetBLUE=adj->value;
    //printf("targetBLUE: %f",targetBLUE);
    std::string str("");
    sprintf((char *)str.c_str(),"set bin %2.2f",targetBLUE);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scalemin( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    minBoundingArea=adj->value;
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_MBA);
        bot.addDouble(minBoundingArea);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaletime( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    reactivity=adj->value;
    //printf("constant time for the iKinControlGaze: %f",reactivity/10);
    std::string str("");
    sprintf((char *)str.c_str(),"set tco %2.2f",reactivity/10);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}

static void cb_digits_scaletime2( GtkAdjustment *adj )
{
    /* Set the number of decimal places to which adj->value is rounded */
    timeCentroid=adj->value;
    //printf("constant time for the controlGaze2: %f",timeCentroid/10);
    std::string str("");
    sprintf((char *)str.c_str(),"set tce %2.2f",timeCentroid/10);
    message->assign(str.c_str());
    if (_pOutPort!=NULL) {
        yarp::os::Bottle& bot = _pOutPort->prepare();
        bot.clear();
        bot.addString(message->c_str());
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write();
    }
}
gint timeout_update_CB(gpointer data) {
    //portFpsData.getStats(av, min, max);
    //portFpsData.reset();
    gchar *msg;
    //gdk_threads_enter();

    msg=g_strdup_printf("selectiveAttentionInterface");
    updateStatusbar(fpsStatusBar, msg);
    g_free(msg);

    //displayFpsData.getStats(av, min, max);
    //displayFpsData.reset();
    
    //periodToFreq(av, min, max, avHz, minHz, maxHz);

    //msg=g_strdup_printf("Display: %.1f (min:%.1f max:%.1f) fps", avHz, minHz, maxHz);
    //updateStatusbar(fpsStatusBar2, msg);
    //g_free(msg);

    //gdk_threads_leave();

    return TRUE;
}

gint timeout_CB (gpointer data) {
    //gdk_threads_enter();
    //gdk_threads_leave();
    return TRUE;
}


gboolean delete_event( GtkWidget *widget, GdkEvent *event, gpointer data ) {
    // If you return FALSE in the "delete_event" signal handler,
    // GTK will emit the "destroy" signal. Returning TRUE means
    // you don't want the window to be destroyed.
    // This is useful for popping up 'are you sure you want to quit?'
    // type dialogs. 
    cleanExit();
    return TRUE;
}

gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    return TRUE;
}

static gboolean configure_event( GtkWidget *widget, GdkEventConfigure *event ) {
    _resources.configure(widget, 
    widget->allocation.width,
    widget->allocation.height);
    return TRUE;
}

static void cb_digits_scale1( GtkAdjustment *adj )
{
    //wModule->setColDim((int)adj->value);
    printf("ColumnDimension: %f",(int) adj->value);
    colDim=(int) adj->value;
}

static void cb_digits_scale2( GtkAdjustment *adj )
{
    //wModule->setColDim((int)adj->value);
    printf("Row Dimension: %f",(double) adj->value);
    rowDim= (int) adj->value;
}

gint menuFileQuit_CB(GtkWidget *widget, gpointer data) {
    cleanExit();
    return TRUE;
}

gint menuHelpAbout_CB(GtkWidget *widget, gpointer data) {
#if GTK_CHECK_VERSION(2,6,0)
    const gchar *authors[] = 
        {
            "Yarp developers",
            NULL
        };
    const gchar *license =
        "Released under the terms of the LGPLv2.1 or later, see LGPL.TXT\n"
        "The complete license description is contained in the\n"
        "COPYING file included in this distribution.\n"
        "Please refer to this file for complete\n"
        "information about the licensing of YARP.\n"
        "\n"
        "DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE\n"
        "SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS\n"
        "DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS\n"
        "EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE\n"
        "SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT\n"
        "OWNERS AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED\n"
        "INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n"
        "FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO\n"
        "EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE\n"
        "LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN\n"
        "ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN\n"
        "ONNECTION WITH THE SOFTWARE.\n";

    gtk_show_about_dialog(GTK_WINDOW(mainWindow),
                          "name", "salBlobFinderInterface",
                          "version", "1.0",
                          "license", license,
                          "website", "http://sourceforge.net/projects/yarp0",
                          "comments", "Interface for selectiveAttentionEngine",
                          "authors", authors,
                          NULL);
#else
    printf("Missing functionality on older GTK version, sorry\n");
#endif

    return TRUE;
}


void setTimedMode(guint dT) {
   // ptr_portCallback->mustDraw(false);
    if (timeout_ID!=0)
        gtk_timeout_remove(timeout_ID);

    timeout_ID = gtk_timeout_add (dT, timeout_CB, NULL);
}

void setSynchroMode() {
    gtk_timeout_remove(timeout_ID);
    timeout_ID=0;
    //ptr_portCallback->mustDraw(true);
}

//-------------------------------------------------
// Non Modal Dialogs
//-------------------------------------------------
GtkWidget* createSaveSingleDialog(void) {

    GtkWidget *dialog = NULL;
    GtkWidget *button;
    GtkWidget *hbox;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Snapshot");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    //gtk_window_resize(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 185, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
    hbox = gtk_hbox_new (TRUE, 8); // parameters (gboolean homogeneous_space, gint spacing);
    button = gtk_button_new_from_stock(GTK_STOCK_SAVE);
    gtk_widget_set_size_request (GTK_WIDGET(button), 150,50);
    gtk_box_pack_start (GTK_BOX (hbox), button, TRUE, TRUE, 16); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    
    //gtk_container_set_border_width(GTK_CONTAINER(hbox), 5);
    
    return dialog;
}

GtkWidget* createSaveSetDialog(void) {
    GtkWidget *dialog = NULL;
    GtkWidget *saveButton;
    GtkWidget *stopButton;
    GtkWidget *hbox;
    dialog = gtk_dialog_new ();
    gtk_window_set_title(GTK_WINDOW(dialog), "Save Image Set");
    gtk_window_set_modal(GTK_WINDOW(dialog), FALSE);
    gtk_window_set_transient_for(GTK_WINDOW(dialog), GTK_WINDOW(mainWindow));
    gtk_window_set_resizable(GTK_WINDOW(dialog), FALSE);
    //gtk_window_set_default_size(GTK_WINDOW(dialog), 190, 40);
    gtk_window_set_destroy_with_parent(GTK_WINDOW(dialog), TRUE);
    gtk_dialog_set_has_separator (GTK_DIALOG(dialog), FALSE);
#if GTK_CHECK_VERSION(2,6,0)
    saveButton = gtk_button_new_from_stock(GTK_STOCK_MEDIA_RECORD);
    stopButton = gtk_button_new_from_stock(GTK_STOCK_MEDIA_STOP);
#else
    printf("Missing functionality on older GTK version, sorry\n");
#endif
    gtk_widget_set_size_request (GTK_WIDGET(saveButton), 80,50);
    gtk_widget_set_size_request (GTK_WIDGET(stopButton), 80,50);

    hbox = gtk_hbox_new (TRUE, 8); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_box_pack_start (GTK_BOX (hbox), saveButton, TRUE, TRUE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (hbox), stopButton, TRUE, TRUE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    gtk_box_pack_start (GTK_BOX (GTK_DIALOG (dialog)->vbox), hbox, FALSE, FALSE, 8); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);

    return dialog;
}

//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* createMenubar(void) {
    GtkWidget *menubar;

    menubar =  gtk_menu_bar_new ();
    GtkWidget *menuSeparator;	
    // Submenus Items on menubar
    fileItem = gtk_menu_item_new_with_label ("File");
    
    helpItem = gtk_menu_item_new_with_label ("Help");
    // Submenu: File 
    fileMenu = gtk_menu_new();
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(fileMenu), menuSeparator);
    fileQuitItem = gtk_menu_item_new_with_label ("Quit");
    gtk_menu_append( GTK_MENU(fileMenu), fileQuitItem);
    gtk_signal_connect( GTK_OBJECT(fileQuitItem), "activate", GTK_SIGNAL_FUNC(menuFileQuit_CB), mainWindow);
    // Submenu: Image  
    imageMenu = gtk_menu_new();
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    synchroDisplayItem = gtk_check_menu_item_new_with_label ("Synch display");
    gtk_menu_append( GTK_MENU(imageMenu), menuSeparator);
    // Submenu: Help
    helpMenu = gtk_menu_new();	
    helpAboutItem = gtk_menu_item_new_with_label ("About..");
    gtk_menu_append( GTK_MENU(helpMenu), helpAboutItem);
    gtk_signal_connect( GTK_OBJECT(helpAboutItem), "activate", GTK_SIGNAL_FUNC(menuHelpAbout_CB), mainWindow);
    // linking the submenus to items on menubar
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(fileItem), fileMenu);
    
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(helpItem), helpMenu);
    // appending the submenus to the menubar
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), fileItem);
    gtk_menu_item_set_right_justified (GTK_MENU_ITEM (helpItem), TRUE);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), helpItem);
  
    return menubar;
}

static GtkWidget *xpm_label_box( gchar     *xpm_filename,gchar *label_text ) {
    GtkWidget *box;
    GtkWidget *label;
    GtkWidget *image;

    /* Create box for image and label */
    box = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box), 2);

    /* Now on to the image stuff */
    if(xpm_filename!=NULL)
        image = gtk_image_new_from_file (xpm_filename);

    /* Create a label for the button */
    label = gtk_label_new (label_text);

    /* Pack the image and label into the box */
    if(xpm_filename!=NULL)
        gtk_box_pack_start (GTK_BOX (box), image, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 3);

    if(xpm_filename!=NULL)
        gtk_widget_show (image);
    gtk_widget_show (label);

    return box;
}

static void scale_set_default_values( GtkScale *scale ) {
    gtk_range_set_update_policy (GTK_RANGE (scale),GTK_UPDATE_CONTINUOUS);
    gtk_scale_set_digits (scale, 2);
    gtk_scale_set_value_pos (scale, GTK_POS_TOP);
    gtk_scale_set_draw_value (scale, TRUE);
}

//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
void updateStatusbar(GtkWidget *statusbar, gchar *msg) {
    GtkStatusbar *sb=GTK_STATUSBAR (statusbar);

    gtk_statusbar_pop (sb, 0); // clear any previous message, underflow is allowed 
    gtk_statusbar_push (sb, 0, msg);
}

//-------------------------------------------------
// Main Window 
//-------------------------------------------------
GtkWidget* createMainWindow(void) {
        //Module=this; //it is necessary to synchronise the static function with this class
    
    GtkRequisition actualSize;
    GtkWidget* window;
    GtkWidget *label;
    GtkWidget *separator;
    GSList *group;
    
    //gtk_init (&argc, &argv);
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "Boltmann Machine Graphical Interface");
    gtk_window_set_default_size(GTK_WINDOW (window), 320, 500); 
    gtk_window_set_resizable (GTK_WINDOW (window), TRUE);
    g_signal_connect (G_OBJECT (window), "destroy",
                      G_CALLBACK (gtk_main_quit),
                      NULL);
    
    // When the window is given the "delete_event" signal (this is given
    // by the window manager, usually by the "close" option, or on the
    // titlebar), we ask it to call the delete_event () function
    // as defined above. The data passed to the callback
    // function is NULL and is ignored in the callback function.
    //g_signal_connect (G_OBJECT (window), "delete_event", G_CALLBACK (delete_event), NULL);

    // Box for main window in a ordered list
    GtkWidget *box,*box2,*boxA,*box3,*box4,*box5, *box34;
    box = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box);
    // MenuBar for main window
    menubar = createMenubar();
    gtk_box_pack_start (GTK_BOX (box), menubar, FALSE, TRUE, 0); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    //gtk_widget_size_request(menubar, &actualSize);

    // Drawing Area : here the image will be drawed
    //da = gtk_drawing_area_new ();
    //g_signal_connect (da, "expose_event", G_CALLBACK (expose_CB), NULL);
    /*if (_options.outputEnabled == 1)
        {
            g_signal_connect (da, "button_press_event", G_CALLBACK (clickDA_CB), NULL);
            // Ask to receive events the drawing area doesn't normally subscribe to
            gtk_widget_set_events (da, gtk_widget_get_events (da) | GDK_BUTTON_PRESS_MASK);
        }
    */
    //gtk_box_pack_start(GTK_BOX(box), da, TRUE, TRUE, 0);

    //Toolbox area
    //creates the area as collection of port processes sequence
    box2 = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box2);
    GtkWidget *button,*button2,*buttonCheck;
    GtkWidget *boxButton,*boxButton2;
    GtkWidget *boxButtons, *boxButtons2;
    GtkWidget *boxSliders;
    boxButtons = gtk_hbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxButtons), 0);
    boxButtons2 = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxButtons2), 0);
    boxSliders = gtk_hbox_new (TRUE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxSliders), 0);
     /* Create a new button */
    button = gtk_button_new ();
    button2 = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "output1");
    g_signal_connect (G_OBJECT (button2), "clicked",G_CALLBACK (callback), (gpointer) "output2");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, (gchar*)"output1");
    boxButton2= xpm_label_box (NULL, (gchar*)"output2");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_widget_show (boxButton2);
    gtk_container_add (GTK_CONTAINER (button2), boxButton2);
    gtk_widget_show (button2);
    //gtk_container_add (GTK_CONTAINER (boxButtons), button);
    //gtk_container_add (GTK_CONTAINER (boxButtons), button2);
    
    gtk_container_add (GTK_CONTAINER (box2), boxButtons);
    
    //---- vSeparator
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);

    
    //-----main section
    GtkWidget *scrollbar;
    
    
    GtkWidget *scale;
    GtkObject *adj1, *adj2;
    GtkWidget *hscale, *vscale;


    adj1 = gtk_adjustment_new (0.0, 0.0, 101.0, 0.1, 1.0, 1.0);
    vscale = gtk_vscale_new (GTK_ADJUSTMENT (adj1));
    scale_set_default_values (GTK_SCALE (vscale));
    gtk_box_pack_start (GTK_BOX (boxSliders), vscale, TRUE, TRUE, 0);
    gtk_widget_show (vscale);

    /*separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (boxSliders), separator, FALSE, FALSE, 0);
    gtk_widget_show (separator);*/

    

    //----------BOXA SECTION:1
    //boxA is the area that contains the two subsection for watershed and saliency operators
    boxA = gtk_hbox_new (FALSE, 0);
    
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
    gtk_widget_show (boxA);
    
    //---- vSeparator
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (boxA), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
    
    //--box3 section A

    adj1 = gtk_adjustment_new (10.0, 0.0, 1000.0, 1.0, 10.0, 10.0);
    adj2 = gtk_adjustment_new (10.0, 0.0, 1000.0, 1.0, 10.0, 10.0);
    box3 = gtk_hbox_new (FALSE, 0);
    
    box5 = gtk_vbox_new (FALSE, 0);

    label = gtk_label_new ("Column Layer Dimension:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 100, 100);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj1), "value_changed",
                      G_CALLBACK (cb_digits_scale1), NULL);

    label = gtk_label_new ("Row Layer Dimension:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj2));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 100, 100);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj2), "value_changed",
                      G_CALLBACK (cb_digits_scale2), NULL);

    gtk_box_pack_start (GTK_BOX (box3), box5, TRUE, TRUE, 0);
    gtk_widget_show (box5);

    //_______

    //label = gtk_label_new ("BM Evolution Control:");
    //gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    //gtk_widget_show (label);


    //scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    // Notice how this causes the scales to always be updated
    // continuously when the scrollbar is moved 
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    box4 = gtk_hbox_new (FALSE, 0);
    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "EvolveFreely");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "EvolveFreely");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "EvolveClamped");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "EvolveClamped");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Stop");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Stop");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    
    box5 = gtk_vbox_new (FALSE, 0);
    
    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "dataSet:start");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "dataSet:start");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box5), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "dataSet:stop");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "dataSet:stop");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box5), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "addSample");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "addSample");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box5), button, TRUE, TRUE, 0);
    gtk_widget_show (button);


    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "setProbabilityNull");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "setProbabilityNull");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box5), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_box_pack_start (GTK_BOX (box3), box5, TRUE, TRUE, 0);
    gtk_widget_show (box5);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Learn");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Learn");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

    //---- vSeparator
    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
    
    //--box3 section B
    //box3 = gtk_hbox_new (FALSE, 0);
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    /*label = gtk_label_new ("Configuration Manager:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);*/


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
        gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "outEyesBehaviour");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "outEyesBehaviour");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "outHeadBehaviour");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "outHeadBehaviour");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Learn");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "Learn");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "Load");
    boxButton = xpm_label_box (NULL, "Load");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    /*button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor1");
    boxButton = xpm_label_box (NULL, "drawVQColor1");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "maxSalienceBlob1");
    boxButton = xpm_label_box (NULL, "maxSalienceBlob1");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);*/

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //------ HSEPARATOR ---------------
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

    //----------BOXA SECTION:2
    //boxA is the area that contains the two subsection for watershed and saliency operators
    boxA = gtk_hbox_new (FALSE, 0);
    
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
    gtk_widget_show (boxA);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

    
    
    //--box3 section A
    box3 = gtk_hbox_new (FALSE, 0);
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    GtkWidget* box33 = gtk_vbox_new (FALSE, 1);
    label = gtk_label_new ("Layer A");
    gtk_box_pack_start (GTK_BOX (box33), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    /*
    label = gtk_label_new ("                                              ");
    gtk_box_pack_start (GTK_BOX (box33), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    label = gtk_label_new ("LayerA in LayerA Box;             LayerB--->");
    gtk_box_pack_start (GTK_BOX (box33), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    */
    //-----Check Buttons
    /* A checkbutton to control whether the value is displayed or not */
    /*buttonCheckGreen = gtk_check_button_new_with_label("inputImage");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheckGreen), FALSE);
    g_signal_connect (G_OBJECT (buttonCheckGreen), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "inputImage");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheckGreen, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheckGreen);
    */
    
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_radio_button_new_with_label(NULL,"A:=Layer0");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), (gchar*) "A:=Layer0");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
    
    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"A:=Layer1");
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "A:=Layer1");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"A:=Layer2");
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "A:=Layer2");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"A:=Layer3");
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "A:=Layer3");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"A:=Layer4");
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "A:=Layer4");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"A:=Layer5");
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "A:=Layer5");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
    
    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"A:=Layer6");
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "A:=Layer6");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"A:=Layer7");
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "A:=Layer7");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"A:=Layer8");
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),(gchar*) "A:=Layer8");
    gtk_box_pack_start (GTK_BOX (boxButtons2), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
    
    gtk_container_add (GTK_CONTAINER (box33), boxButtons2);

    gtk_box_pack_start (GTK_BOX (box3), box33, TRUE, TRUE, 0);
    gtk_widget_show (box33);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
    label = gtk_label_new ("Layer B");
    gtk_box_pack_start (GTK_BOX (box4), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_radio_button_new_with_label(NULL,"B:=Layer0");
    char* valueSL0="B:=Layer0";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL0);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"B:=Layer1");
    char* valueSL1="B:=Layer1";
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"B:=Layer2");
    char* valueSL2="B:=Layer2";
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL2);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"B:=Layer3");
    char* valueSL3="B:=Layer3";
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL3);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"B:=Layer4");
    char* valueSL4="B:=Layer4";
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL4);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"B:=Layer5");
    char* valueSL5="B:=Layer5";
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL5);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);


    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"B:=Layer6");
    char* valueSL6="B:=Layer6";
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL6);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"B:=Layer7");
    char* valueSL7="B:=Layer7";
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL7);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    group = gtk_radio_button_get_group(GTK_RADIO_BUTTON (buttonCheck)); //extracting the group from the previous button
    buttonCheck = gtk_radio_button_new_with_label(group,"B:=Layer8");
    char* valueSL8="B:=Layer8";
    //gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueSL8);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    char* valueCL="ConnectLayer";
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) valueCL);
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "ConnectLayer(A,B)");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    char* valueAL="AddLayer";
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) valueAL);
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "AddLayer()");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "ClampLayer");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "ClampLayer(A)");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);

    //---- vSeparator
    separator = gtk_vseparator_new ();
    gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    gtk_widget_show (separator);
    
    //--box3 section B
    //box3 = gtk_hbox_new (FALSE, 0);
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    label = gtk_label_new ("Options:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
    
    /*-------------
    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);
    ---------------*/

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern1-->");
    char* valueP1="Pattern1";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern2-->");
    char* valueP2="Pattern2";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP2);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern3-->");
    char* valueP3="Pattern3";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP3);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "ClampPattern");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "ClampPattern");	
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    gtk_widget_show (box4);
    //---box 4

    /*button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "DrawAllBlobs2");
    boxButton = xpm_label_box (NULL, "DrawAllBlobs2");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawFoveaBlob2");
    boxButton = xpm_label_box (NULL, "drawFoveaBlob2");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor2");
    boxButton = xpm_label_box (NULL, "drawVQColor2");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);*/

    //------ HSEPARATOR ---------------
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

    //----------BOXA SECTION:3
    //boxA is the area that contains the two subsection for watershed and saliency operators
    boxA = gtk_hbox_new (FALSE, 0);
    box3 = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (boxA), 0);
    gtk_box_pack_start (GTK_BOX (box2), boxA, TRUE, TRUE, 0);
    gtk_widget_show (boxA);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    gtk_widget_show (box3);
    
    //--box3 section A
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    label = gtk_label_new ("Options:");
    gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    gtk_widget_show (label);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    char* valueG1="Green1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueG1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    char* valueR1="Red1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value),valueR1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    char* valueB1="Blue1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueB1);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    //gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    //gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern1-->");
    char* valueP1b="Pattern1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP1b);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern2-->");
    char* valueP2b="Pattern2-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP2b);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    /* A checkbutton to control whether the value is displayed or not */
    buttonCheck = gtk_check_button_new_with_label("Pattern3-->");
    char* valueP3b="Pattern3-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueP3b);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    //-------run button
    button = gtk_button_new ();
    /* Connect the "clicked" signal of the button to our callback */
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "ClampPattern");
    /* This calls our box creating func tion */
    boxButton = xpm_label_box (NULL, "ClampPattern");
    /* Pack and show all our widgets */
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box4), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    //gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    //gtk_widget_show (box4);
    //---box 4

    

    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    //gtk_box_pack_start (GTK_BOX (boxA), box3, TRUE, TRUE, 0);
    //gtk_widget_show (box3);

    //---- vSeparator
    separator = gtk_vseparator_new ();
    //gtk_box_pack_start (GTK_BOX (box3), separator, FALSE, TRUE, 3);
    //gtk_widget_show (separator);
    
    //--box3 section B
    /*hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box3), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);*/

    label = gtk_label_new ("CheckList:");
    //gtk_box_pack_start (GTK_BOX (box3), label, FALSE, FALSE, 0);
    //gtk_widget_show (label);


    scrollbar = gtk_hscrollbar_new (GTK_ADJUSTMENT (adj1));
    /* Notice how this causes the scales to always be updated
     * continuously when the scrollbar is moved */
    /*gtk_range_set_update_policy (GTK_RANGE (scrollbar), 
                                 GTK_UPDATE_CONTINUOUS);
    gtk_box_pack_start (GTK_BOX (box3), scrollbar, TRUE, TRUE, 0);
    gtk_widget_show (scrollbar);*/

    //-----Check Buttons
    box4=  gtk_vbox_new (FALSE, 0);

    /*--------------------
    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Green1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Green1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Red1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), TRUE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Red1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    // A checkbutton to control whether the value is displayed or not 
    buttonCheck = gtk_check_button_new_with_label("Blue1-->");
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), "Blue1");
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    -----*/


    //gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    //gtk_widget_show (box4);

    //-----box4
    box4=  gtk_vbox_new (FALSE, 0);
    buttonCheck = gtk_check_button_new_with_label("ContrastLP3-->");
    char* valueCLP3="ContrastLP3-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueCLP3);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    buttonCheck = gtk_check_button_new_with_label("MeanColoursLP3-->");
    char* valueMCLP3="MeanColoursLP3-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueMCLP3);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    buttonCheck = gtk_check_button_new_with_label("Normalize1-->");
    char* valueN1b="Normalize1-->";
    gtk_toggle_button_set_active (GTK_TOGGLE_BUTTON (buttonCheck), FALSE);
    g_signal_connect (G_OBJECT (buttonCheck), "toggled",G_CALLBACK (cb_draw_value), valueN1b);
    gtk_box_pack_start (GTK_BOX (box4), buttonCheck, TRUE, TRUE, 0);
    gtk_widget_show (buttonCheck);

    //gtk_box_pack_start (GTK_BOX (box3), box4, TRUE, TRUE, 0);
    //gtk_widget_show (box4);
    
    //---box 4

    //-------run button
    /*button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "DrawAllBlobs3");
    boxButton = xpm_label_box (NULL, "DrawAllBlobs3");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawFoveaBlob3");
    boxButton = xpm_label_box (NULL, "drawFoveaBlob3");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);
    gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    gtk_widget_show (button);

    button = gtk_button_new ();
    g_signal_connect (G_OBJECT (button), "clicked",G_CALLBACK (callback), (gpointer) "drawVQColor3");
    boxButton = xpm_label_box (NULL, "drawVQColor3");
    gtk_widget_show (boxButton);
    gtk_container_add (GTK_CONTAINER (button), boxButton);
    gtk_widget_show (button);*/

    //gtk_box_pack_start (GTK_BOX (box3), button, TRUE, TRUE, 0);
    //gtk_widget_show (button);

    //------ HSEPARATOR ---------------
    separator = gtk_hseparator_new ();
    gtk_box_pack_start (GTK_BOX (box2), separator, FALSE, TRUE, 0);
    gtk_widget_show (separator);

    //gtk_container_add (GTK_CONTAINER (box2), boxSliders);
    gtk_box_pack_start(GTK_BOX(box), box2,FALSE,FALSE, 10);
    // StatusBar for main window
    statusbar = gtk_statusbar_new ();
    updateStatusbar(GTK_WIDGET (statusbar),NULL);
    gtk_box_pack_start (GTK_BOX (box), statusbar, FALSE, TRUE, 0);
    gtk_widget_size_request(statusbar, &actualSize);
    //_occupiedHeight += 2*(actualSize.height);

    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, 320, 240);
    // TimeOut used to refresh the screen
    timeout_ID = gtk_timeout_add (100, timeout_CB, NULL);

    mainWindow=window;

    return window;
}
void configure(yarp::os::ResourceFinder rf){
    /* Process all parameters from both command-line and .ini file */
    /* get the module name which will form the stem of all module port names */
    _options.portName      = rf.check("name", 
                           Value("/bmlInterface"), 
                           "module name (string)").asString();
}

void setOptionsToDefault() {
    // Options defaults
    _options.refreshTime = 100;
    _options.outputEnabled = 0;
    _options.windWidth = 300;
    _options.windHeight = 300;
    _options.posX = 100;
    _options.posY = 100;
    _options.saveOnExit = 0;
}

bool openPorts() {
    _pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    //g_print("Registering port %s on network %s...\n", _options.outPortName, "default");
    _options.portName+="/command:o";
    bool ok = _pOutPort->open(_options.portName.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }
    return true;
}

void closePorts() {
    _pOutPort->close();
    bool ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _pOutPort;
}

void cleanExit() {
    if (timeout_ID!=0)
        g_source_remove (timeout_ID);
    timeout_ID = 0;
    
    g_source_remove(timeout_update_ID);
    timeout_update_ID=0;

    gtk_main_quit ();
}

//-------------------------------------------------
// Main
//-------------------------------------------------
#undef main //ace leaves a "main" macro defined

int myMain(int argc, char* argv[]) {
    yarp::os::Network yarp;

    //initialize threads in gtk, copied almost verbatim from
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    //g_thread_init (NULL);
    //gdk_threads_init ();
    //gdk_threads_enter ();
    
    createObjects();
    _frameN = 0;
    timeout_ID = 0;
    setOptionsToDefault();

    yarp::os::ResourceFinder rf;
    //rf=new yarp::os::ResourceFinder();
    rf.setVerbose(true);
    rf.setDefaultConfigFile("bmlInterface.ini"); //overridden by --from parameter
    rf.setDefaultContext("bmlApplication/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
    configure(rf);
    
    // Parse command line parameters, do this before
    // calling gtk_init(argc, argv) otherwise weird things 
    // happens
    if (!openPorts())
        goto exitRoutine;

    // This is called in all GTK applications. Arguments are parsed
    // from the command line and are returned to the application.
    gtk_init (&argc, &argv);

    // create a new window
    mainWindow = createMainWindow();
    
    // Non Modal Dialogs
#if GTK_CHECK_VERSION(2,6,0)
    saveSingleDialog = createSaveSingleDialog();
    saveSetDialog = createSaveSetDialog();
#else
    printf("Functionality omitted for older GTK version\n");
#endif
    // Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
    gtk_window_move(GTK_WINDOW(mainWindow), _options.posX, _options.posY);
    // All GTK applications must have a gtk_main(). Control ends here
    // and waits for an event to occur (like a key press or
    // mouse event).

    //ptr_portCallback->attach(&_resources);
    //ptr_portCallback->attach(&portFpsData);
    //ptr_inputPort->useCallback(*ptr_portCallback);

    if (_options.synch)
    {
        setSynchroMode();
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), true);
    }
    else
    {
        setTimedMode(_options.refreshTime);
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), false);
    }

    gtk_main ();

exitRoutine:
    // leave critical section here. From example
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    //gdk_threads_leave ();

    closePorts();
    deleteObjects();
    return 0;
}

#ifdef YARP_WIN32_NOCONSOLE
#include <windows.h>
// win32 non-console applications define WinMain as the
// entry point for the linker
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
    return myMain (__argc, __argv);
}
#else
int main(int argc, char* argv[]) {
    return myMain(argc, argv);
}
#endif

void printHelp() {
    g_print("selAttentionInterface usage:\n");
    g_print("--name: input port name (default: /selAttentionInterface)\n");
}

