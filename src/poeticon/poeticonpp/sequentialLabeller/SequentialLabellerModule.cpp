/*
 * Copyright (C) 2013 POETICON++, European Commission FP7 project ICT-288382
 * Author: Giovanni Saponaro <gsaponaro@isr.ist.utl.pt>
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 
 *
 */

/* system */
#include <iostream>

#include "SequentialLabellerModule.h"

/**
 * Receive a previously initialized Resource Finder object and process module parameters,
 * both from command line and .ini file.
 */
bool SequentialLabellerModule::configure(ResourceFinder &rf)
{
	/* get the name that will form the prefix of all module port names */
	_moduleName = rf.check( "name",
				Value("/sequentialLabeller"),
				"Module name (string)" ).asString();

	/* before continuing, set the module name */
	setName(_moduleName.c_str());

	/* now, get the remaining parameters */
	rawImgInputPortName         = getName(
                                           rf.check( "raw_image_input_port",
                                                     Value("/rawImg:i"),
                                                     "Raw image input port (string)" ).asString()
                                           );
    rawImgOutputPortName        = getName(
                                           rf.check( "raw_image_output_port",
                                                     Value("/rawImg:o"),
                                                     "Raw image output port (string)" ).asString()
                                           );
                                           
    binaryImgInputPortName      = getName(
                                           rf.check( "binary_image_input_port",
                                                     Value("/binImg:i"),
                                                     "Binary image input port (string)" ).asString()
                                           );
    labeledImgOutputPortName    = getName(
                                           rf.check( "labeled_image_output_port",
                                                     Value("/labeledImg:o"),
                                                     "Labeled image output port (string)" ).asString()
                                           );
	//Network::init();
	
	/* open ports */
	if(! _handlerPort.open(_handlerPortName.c_str()) )
	{
		cout << getName() << ": unable to open port" << _handlerPortName << endl;
		return false;
	}
    if(! rawImgInputPort.open(rawImgInputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << rawImgInputPortName << endl;
        return false;
    }
    if(! binaryImgInputPort.open(binaryImgInputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << binaryImgInputPortName << endl;
        return false;
    }
    if(! rawImgOutputPort.open(rawImgOutputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << rawImgOutputPortName << endl;
        return false;
    }
    if(! labeledImgOutputPort.open(labeledImgOutputPortName.c_str()) )
    {
        cout << getName() << ": unable to open port" << labeledImgOutputPortName << endl;
        return false;
    }
    
    yarpRawInputPtr    = rawImgInputPort.read(true);
	yarpBinaryInputPtr = binaryImgInputPort.read(true);
	
	if (yarpRawInputPtr!=NULL && yarpBinaryInputPtr!=NULL)
	{
	
	    /* check that raw and binary image dimensions are equal */
	    if( (yarpRawInputPtr->width() != yarpBinaryInputPtr->width()) || 
		    (yarpRawInputPtr->height() != yarpBinaryInputPtr->height()) )
	    {
		    cout << getName() << ": input image dimensions differ. Exiting..." << endl;
            return false;
	    }	
        w   = yarpRawInputPtr->width();
        h   = yarpRawInputPtr->height();
        sz  = cvSize(w, h);

	    /* allocate internal image buffers */
	    yarpRawImg.resize(w,h);
        yarpBinaryImg.resize(w,h);
        yarpLabeledImg.resize(w,h);
    
    }

	return true; /* tell RFModule that everything went well, so that it will run the module */
}
	
/**
 * Try to halt operations by threads managed by the module. Called asynchronously
 * after a quit command is received.
 */
bool SequentialLabellerModule::interruptModule()
{
	cout << getName() << ": interrupting module, for port cleanup." << endl;
	_handlerPort.interrupt();
	rawImgInputPort.interrupt();
	rawImgOutputPort.interrupt();
	binaryImgInputPort.interrupt();
	labeledImgOutputPort.interrupt();
	return true;
}
	
/**
 * Close function. Called automatically when the module closes, after the last
 * updateModule call.
 */
bool SequentialLabellerModule::close()
{
	cout << getName() << ": closing module." << endl;
	_handlerPort.close();
	rawImgInputPort.close();
	rawImgOutputPort.close();
	binaryImgInputPort.close();
	labeledImgOutputPort.close();

	// Network::fini();
	return true;
}
   
/**
 * Message handler function. Echo all received messages, quit if required.
 */
bool SequentialLabellerModule::respond(const Bottle &command, Bottle &reply)
{
  	cout << getName() << ": echoing received command." << endl;
  	reply = command;
  	if(command.get(0).asString() == "quit")
		return false;
  	else
  		return true;
}
   
/**
 * Main cycle, called iteratively every getPeriod() seconds.
 */
bool SequentialLabellerModule::updateModule()
{
    Stamp rawstamp, binarystamp, writestamp; 
    
    yarpRawInputPtr    = rawImgInputPort.read(true);
	yarpBinaryInputPtr = binaryImgInputPort.read(true);

	/* check that both images have timestamps */
	if( !rawImgInputPort.getEnvelope(rawstamp) || !binaryImgInputPort.getEnvelope(binarystamp) )
	{
        cout << getName() << ": this module requires ports with valid timestamp data. Stamps are missing. Exiting..." << endl;
		return false;
	}
    /* synchronize the two images, if one of them is delayed, so that they correspond */
	while( rawstamp.getCount() < binarystamp.getCount() )
	{
        cout << "warning: input images have different timestamps." << endl;
		yarpRawInputPtr = rawImgInputPort.read(true);
		rawImgInputPort.getEnvelope(rawstamp);
	}
	while( rawstamp.getCount() > binarystamp.getCount() )
	{
        cout << "warning: input images have different timestamps." << endl;
		yarpBinaryInputPtr = binaryImgInputPort.read(true);
		binaryImgInputPort.getEnvelope(binarystamp);
	}
	
	// here both stamps are equal
	writestamp = rawstamp;

    if( yarpRawInputPtr==NULL || yarpBinaryInputPtr==NULL)
    {
        cout << getName() << ": no data on input port(s). Exiting..." << endl;
		return false;    
    }
    yarpRawImg     = *yarpRawInputPtr;
    yarpBinaryImg  = *yarpBinaryInputPtr;

	// get OpenCV pointers to images, to more easily call OpenCV functions
	// TODO: use OpenCV 2 APIs

    IplImage *opencvRawImg     = (IplImage *) yarpRawImg.getIplImage();
    IplImage *opencvBinaryImg  = (IplImage *) yarpBinaryImg.getIplImage();
    
    // call sequential labelling algorithm
    IplImage *opencvLabeledImg;
    IplImage *opencvTempImg;
    opencvLabeledImg = cvCreateImage( sz, IPL_DEPTH_8U, 1);
    opencvTempImg = cvCreateImage( sz, IPL_DEPTH_32S, 1);
    cvZero(opencvTempImg);
    int seqLabReturn = 0;
    seqLabReturn = cvSeqLabel(opencvBinaryImg, opencvLabeledImg, opencvTempImg);
    
    // convert from OpenCV to yarp format
    if (opencvLabeledImg != NULL)
    {
        cvCopyImage( opencvLabeledImg, (IplImage *)yarpLabeledImg.getIplImage() );
    }

	/* output original (propagated) raw image */
	ImageOf<PixelRgb> &yarpRawOutputImage = rawImgOutputPort.prepare();
	yarpRawOutputImage = yarpRawImg;
	rawImgOutputPort.setEnvelope(writestamp);
	rawImgOutputPort.write();

    /* output sequentially labelled image */	
	ImageOf<PixelMono> &yarpLabeledOutputImage = labeledImgOutputPort.prepare();
	yarpLabeledOutputImage = yarpLabeledImg;
	labeledImgOutputPort.setEnvelope(writestamp);
	labeledImgOutputPort.write();
	
    cvReleaseImage(&opencvLabeledImg);
    cvReleaseImage(&opencvTempImg);

  	return true;
}

double SequentialLabellerModule::getPeriod()
{
  return 0.0;
}
