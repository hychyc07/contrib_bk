
#include <yarp/dev/Drivers.h>
#include "SCSPMClassifier.h"



/** 
\defgroup icub_ScSPM ScSPM
@ingroup icub_interactiveObjectsLearning 
 
The module manages the spatial pyramid representation framework making use of hierarchical image representation in \ref icub_image_representation.

 
\section intro_sec Description 
This module is responsible for the communication with \ref icub_sparseCoder and \ref icub_linearClassifier for learning and classify feature vectors. Input features are passed, the output are the scores of SVM machines.

 
 
The commands sent as bottles to the module port 
/SCSPMClassifier/rpc are the following: 
 
(notation: [.] identifies a vocab, <.> specifies a double,
"." specifies a string) 
 
<b>TRAIN</b> 
format: [train] 
action: starts the training process by waiting for an image and a bounding box where the object is located.
 
<b>CLASSIFY</b> 
format: [classify] 
action: starts the classification process by waiting for an image and a bounding box where the object is located, it returns the output scores in the form "class" "score".
 
<b>FORGET</b> 
format: [forget] "class" 
action: forgets the "class", deleting all the feature vectors in the database. If "class"="all" all the classes are forgotten. 
 
<b>BURST</b> 
format: [burst] [start |stop ]
action: If [start] it starts the training process by waiting for a stream of images and bounding boxes where the object is located. If [stop] it stops the current burst session and automatically trains the new class model.

 
\section lib_sec Libraries 
- YARP libraries. 

- OpenCV 2.2

\section portsc_sec Ports Created 

- \e /SCSPMClassifier/rpc receives rpc requests for training and recognition.

- \e /SCSPMClassifier/img:i receives an input image.
 
 - \e /SCSPMClassifier/img:o outputs the image for the \ref icub_sparseCoder. 
 
- \e /SCSPMClassifier/classify:rpc RPC port that communicates with \ref icub_linearClassifier module.
 
- \e /SCSPMClassifier/SIFTimg:i reads the image with the extracted local descriptors from the \ref icub_sparseCoder. 
 
- \e /SCSPMClassifier/SIFTimg:o outputs the image with the extracted local descriptors. 

- \e /SCSPMClassifier/scores:i reads the classification scores from the \ref icub_linearClassifier. 

- \e /SCSPMClassifier/features:i reads the hierarchical image representation from the \ref icub_sparseCoder. 

- \e /SCSPMClassifier/features:o outpus the hierarchical image representation to the \ref icub_linearClassifier. 
  
 - \e /SCSPMClassifier/opc communication with the Object Property Collection.  
  
\section parameters_sec Parameters 

None.
 
 
\section tested_os_sec Tested OS
Linux, Windows 7

\author Sean Ryan Fanello
*/ 


int main(int argc, char * argv[])
{

   Network yarp;
   SCSPMClassifier SCSPMClassifier; 

   ResourceFinder rf;
   rf.setVerbose(true);
   rf.setDefaultConfigFile("SCSPMClassifier.ini"); 
   rf.setDefaultContext("iolStateMachineHandler/conf");
   rf.configure("ICUB_ROOT", argc, argv);
 

   SCSPMClassifier.runModule(rf);

    return 0;
}
