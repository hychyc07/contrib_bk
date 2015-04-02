/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: David Vernon
 * email:   david@vernon.eu
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


/*
 * Audit Trail
 * -----------
 * 11/11/09  Started development  
 * 23/12/09  Completed initial development  
 * 20/09/11  Restarted development following completion of selective tuning and overt attention modules 
 *           Graph implementation of the procedural memory
 * 22/11/11  New protocol for the database folder: the path now specifies the directory in which both the images and
 *           the database file (i.e. index file) are stored. The path must be valid (i.e. the folder must exist) 
 *           but the index file and images are created if necessary.
 * 04/11/11  Completed graph implementation, including visualization routines and representation of association 
 *           strength between all Pi, Aj, Pk triples.
 * 16/01/12  Amended functionality for the special case where Pi = Pk.  This typically happens in smooth pursuit
 *           or when there is a second micro-saccade in overtAttention to refine the fixation.
 *           In Learning mode, we simply ignore the new input.
 *           In Prediction and Reconstruction modes, we skip the associative recall and simply output:
 *           - the same image id. and a coupling strength of 1 (imageIdOutPort)
 *           - zero azimuth, elevation, and vergence (ActionOutPort)
 * 25/01/12  New module parameter to clear the memory on start up; 
 *           if not set the memory is initialized from the database 
 */ 
 


#include "iCub/proceduralMemory.h"


ProceduralMemory::ProceduralMemory() {
   debug = false;
}

bool ProceduralMemory::configure(yarp::os::ResourceFinder &rf)
{    
   /*
    * Process all parameters from 
    *  - command-line 
    *  - proceduralMemory.ini file (or whatever file is specified by the --from argument)
    */

   /* get the module name which will form the stem of all module port names */

   moduleName            = rf.check("name", 
                           Value("proceduralMemory"), 
                           "module name (string)").asString();

   /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
   
   setName(moduleName.c_str());

   /* get the name of the input and output ports, automatically prefixing the module name by using getName() */

   imageIdInputPortName          = "/";
   imageIdInputPortName         += getName(
                                   rf.check("imageIdInPort", 
                                   Value("/imageId:i"),
                                   "current image id port (string)").asString()
                                   );
   
   modeInputPortName            = "/";
   modeInputPortName           += getName(
                                   rf.check("modePort", 
                                   Value("/mode:i"),
                                   "mode (corresponding to learning, prediction, reconstruction) port (string)").asString()
                                   );
 
   imageIdOutputPortName         = "/";
   imageIdOutputPortName        += getName(
                                   rf.check("imageIdOutPort", 
                                   Value("/imageId:o"),
                                   "current image id port (string)").asString()
                                   );
   
   actionOutputPortName          = "/";
   actionOutputPortName         += getName(
                                   rf.check("actionOutPort", 
                                   Value("/action:o"),
                                   "saccade and action tag port (string)").asString()
                                   );
  
   graphOutputPortName           = "/";
   graphOutputPortName          += getName(
                                   rf.check("graphPort", 
                                   Value("/graph:o"),
                                   "graphs port (string)").asString()
                                   );

   graphWidth                    = rf.check("graphWidth",
					               Value(320),
					               "Width of visualization image (int)").asInt();

   graphHeight                   = rf.check("graphHeight",
					               Value(240),
					               "Height of visualization image (int)").asInt();

   databaseName                  = rf.check("database",
					               Value("proceduralDatabase.txt"),
					               "Database name (string)").asString().c_str();

   path                          = rf.check("path",
				                   Value("~/iCub/app/proceduralMemory"),
    			                   "complete path to database file").asString().c_str(); 
    
   clearMemory                   = rf.check("clear");

   if (debug) {
      printf("proceduralMemory: module name is %s\n",moduleName.c_str());
      printf("proceduralMemory: parameters are\n%s\n%s\n%s\n%s\n%s\n%d\n%d\n%s\n%s\n\n",   
              imageIdInputPortName.c_str(),
              modeInputPortName.c_str(),
              imageIdOutputPortName.c_str(),
              actionOutputPortName.c_str(),
              graphOutputPortName.c_str(),
              graphWidth,
              graphHeight,
              databaseName.c_str(), 
              path.c_str()
            );
       if (clearMemory)     printf("episodicMemory: clear flag is set\n");
       else                 printf("episodicMemory: clear flag is not set\n");

   }
    
   /* do all initialization here */
     
   /* open ports  */ 
       
   if (!imageIdIn.open(imageIdInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << imageIdInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!modeIn.open(modeInputPortName.c_str())) {
      cout << getName() << ": unable to open port " << modeInputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!imageIdOut.open(imageIdOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << imageIdOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }
      
   if (!actionOut.open(actionOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << actionOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }

   if (!graphOut.open(graphOutputPortName.c_str())) {
      cout << getName() << ": unable to open port " << graphOutputPortName << endl;
      return false;  // unable to open; let RFModule know so that it won't run
   }



   /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */

   handlerPortName =  "/";
   handlerPortName += getName();         // use getName() rather than a literal 
 
   if (!handlerPort.open(handlerPortName.c_str())) {           
      cout << getName() << ": Unable to open port " << handlerPortName << endl;  
      return false;
   }

   attach(handlerPort);                  // attach to port
 
   /* create the thread and pass pointers to the module parameters */

   proceduralMemoryThread = new ProceduralMemoryThread(&imageIdIn, 
                                                       &modeIn, 
                                                       &imageIdOut, 
                                                       &actionOut, 
                                                       &graphOut, 
                                                       &graphWidth,
                                                       &graphHeight,
                                                       &databaseName, 
                                                       &path,
                                                       &clearMemory);

   /* now start the thread to do the work */

   proceduralMemoryThread->start(); // this calls threadInit() and it if returns true, it then calls run()

   return true ;      // let the RFModule know everything went well
                      // so that it will then run the module
}


bool ProceduralMemory::interruptModule()
{
   imageIdIn.interrupt();
   modeIn.interrupt();
   imageIdOut.interrupt();
   actionOut.interrupt();
   graphOut.interrupt();
   handlerPort.interrupt(); 

   return true;
}


bool ProceduralMemory::close()
{
   imageIdIn.close();
   modeIn.close();
   imageIdOut.close();
   actionOut.close();
   graphOut.close();
   handlerPort.close(); 

   /* stop the thread */

   proceduralMemoryThread->stop();

   return true;
}


bool ProceduralMemory::respond(const Bottle& command, Bottle& reply) 
{
  string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
                        "quit \n" + 
                        "(where <n> is an real number) \n";

  reply.clear(); 

  if (command.get(0).asString()=="quit") {
       reply.addString("quitting");
       return false;     
   }
   else if (command.get(0).asString()=="help") {
      cout << helpMessage;
      reply.addString("ok");
   }

   return true;
}


/* Called periodically every getPeriod() seconds */

bool ProceduralMemory::updateModule()
{
   return true;
}



double ProceduralMemory::getPeriod()
{
   /* module periodicity (seconds), called implicitly by proceduralMemory */
    
   return 0.1;
}

 

ProceduralMemoryThread::ProceduralMemoryThread(BufferedPort<VectorOf<double> >  *imageIdIn,
                                               BufferedPort<VectorOf<double> >  *modeIn,
                                               BufferedPort<VectorOf<double> >  *imageIdOut,
                                               BufferedPort<Bottle>             *actionOut,
                                               BufferedPort<ImageOf<PixelRgb> > *graphOut,
                                               int                              *graphWidth,
                                               int                              *graphHeight,
                                               string                           *databaseName,
                                               string                           *path,
                                               bool                             *clearMemory)
{
   imageIdInPort         = imageIdIn;
   modeInPort            = modeIn;
   imageIdOutPort        = imageIdOut;
   actionOutPort         = actionOut;
   graphOutPort          = graphOut;
   graphWidthValue       = graphWidth;
   graphHeightValue      = graphHeight;
   databaseNameValue     = databaseName;
   pathValue             = path;
   clearMemoryValue      = clearMemory;

   if (debug) {
      cout << "ProceduralMemoryThread: graph width and height " << *graphWidthValue << " " << *graphHeightValue << endl;
      cout << "ProceduralMemoryThread: database name          " << *databaseNameValue << endl;
      cout << "ProceduralMemoryThread: path                   " << *pathValue << endl;
   }
   if (*clearMemory)
      cout << "ProceduralMemoryThread: clear                  true"   << endl;
   else
      cout << "ProceduralMemoryThread: clear                  false"   << endl;

}

bool ProceduralMemoryThread::threadInit() 
{
   /* initialize variables and create data-structures if needed */

    debug = false;

    width  = 0;
    height = 0;
    depth  = 0;

    imageIdCurrent = -1;
    imageSimilarityCurrent = 0;
    azimuth = 0;
    elevation = 0;
    vergence = 0;
    imageIdPrevious = -1;
    imageSimilarityPrevious = 0;
    coupling = 0;

    graph.setDatabaseContext(*pathValue);
    if (debug)
       std::cout << "ProceduralMemoryThread::threadInit: databaseContext " << (*pathValue).c_str() << endl;
    
	graph.setDatabaseName(*databaseNameValue);
    if (debug)
       std::cout << "ProceduralMemoryThread::threadInit: databaseName    " << (*databaseNameValue).c_str() << endl;
    
    if (!(*clearMemoryValue)) {
       graph.loadDatabase();
    }

    return true;
}

void ProceduralMemoryThread::run(){

   int node1, node2;

   /* 
    * start the procedural memory operating 
    */ 
      
   if (debug) {
      cout << "ProceduralMemoryThread::run: graph width and height " << *graphWidthValue << " " << *graphHeightValue << endl;
      cout << "ProceduralMemoryThread::run: database name          " << *databaseNameValue << endl;
      cout << "ProceduralMemoryThread::run: path                   " << *pathValue << endl;
   }

   while (isStopping() != true) { // the thread continues to run until isStopping() returns true
 
      /* 
       * Step 1: read the mode, the image id, the action, and the previous image id
       * ==========================================================================
       *
       */

      if (debug) {
         cout << "ProceduralMemoryThread::run: Step 1" << endl;
      }

      /* Determine which mode we are in: learning, exploration, or reconstruction */

      modeIn = modeInPort->read(false);              // read a vector containing the mode; don't block

      if (modeIn != NULL) {
         mode = (int) (*modeIn)[0];
      }
      else {
         mode = LEARNING_MODE;
         if (debug) {
            cout << "ProceduralMemoryThread::run: LEARNING MODE" << endl;
         }
      }

            
      do {
         imageIdIn  = imageIdInPort->read(true);                // read a vector containing the image data; do block
      } while ((imageIdIn == NULL) && (isStopping() != true));  // exit loop if shutting down
      
      if (isStopping()) {
         if (debug) {
            cout << "ProceduralMemoryThread::run: aborting" << endl;
         }
         break;                                  // abort this loop to avoid make sure we don't continue and possibly use NULL images 
      }

      imageIdPrevious         =       imageIdCurrent;
      imageSimilarityPrevious =       imageSimilarityCurrent;

      imageIdCurrent          = (int) (*imageIdIn)[0];
      imageSimilarityCurrent  =       (*imageIdIn)[1];
      azimuth                 =       (*imageIdIn)[2];
      elevation               =       (*imageIdIn)[3];
      vergence                =       (*imageIdIn)[4];

      if (mode == LEARNING_MODE) {
    
         if (debug && false) {
            cout << "ProceduralMemoryThread::run: LEARNING MODE" << endl;
         }

         /* new  functionality for the special case where Pi = Pk (i.e. imageIdPrevious = imageIdCurrent) */
         /* This typically happens in smooth pursuit or when there                                        */
         /* is a second micro-saccade in overtAttention to refine the fixation                            */

         /* In Learning mode, we simply ignore the new input                                              */

         if (imageIdCurrent != imageIdPrevious) {

            graph.updateGraph(imageIdPrevious, (int) azimuth, (int) elevation,  imageIdCurrent);  
            graph.predict(graph.getPerceptionNode(imageIdPrevious), node1, node2,  coupling);   
            graph.reconstruct(node1, node2, graph.getPerceptionNode(imageIdCurrent), coupling); 
             
            ImageOf<PixelRgb> &graphImage = graphOutPort->prepare();
            graphImage.resize(*graphWidthValue,*graphHeightValue);
            graph.generateGraphImage(graphImage);
            graphOutPort->write();
         }

      }
      else if (mode == PREDICTION_MODE) {
            
         /* we use the current imageId to retrieve the associated subsequent action and, in turn, the associated image */

         /* new  functionality for the special case where Pi = Pk (i.e. imageIdPrevious = imageIdCurrent) */
         /* This typically happens in smooth pursuit or when there                                        */
         /* is a second micro-saccade in overtAttention to refine the fixation                            */

         /* In Prediction mode, we skip the associative recall and simply output:                         */
         /*                                                                                               */
         /* - the same image id. and a coupling strength of 1                                             */               
         /* - zero azimuth, elevation, and vergence                                                       */

         if (imageIdCurrent == imageIdPrevious) {

            imageSimilarityCurrent = 1;
            coupling = 1;
            azimuth = 0;
            elevation = 0;
            vergence = 0;

            VectorOf<double>& out = imageIdOutPort->prepare();
            out.resize(2,0);
            out[0] = imageIdCurrent;
            out[1] = coupling;
            imageIdOutPort->write();

            Bottle& bot = actionOutPort->prepare();
            bot.addString("rel");      // "rel" since gaze is relative to current fixation 
            bot.addDouble(azimuth);    
            bot.addDouble(elevation);
            bot.addDouble(vergence);    
            actionOutPort->write();
         }
         else {
            imageIdPrevious          = (int)  (*imageIdIn)[0];
            imageSimilarityPrevious  =        (*imageIdIn)[1];

            graph.predict(graph.getPerceptionNode(imageIdPrevious), node1, node2,  coupling); 
            //azimuth   = graph.getSaccadeCoordinate(1,node1);
            //elevation = graph.getSaccadeCoordinate(2,node1);
            //imageIdCurrent = graph.getImageId(node2);

            imageSimilarityCurrent = 0;
 
            VectorOf<double>& out = imageIdOutPort->prepare();
            out.resize(2,0);
            out[0] = imageIdCurrent;
            out[1] = coupling;
            imageIdOutPort->write();

            Bottle& bot = actionOutPort->prepare();
            bot.addString("rel");      // "rel" since gaze is relative to current fixation 
            bot.addDouble(azimuth);    
            bot.addDouble(elevation);
            bot.addDouble(vergence);    
            actionOutPort->write();
         }
      }
      else if (mode == RECONSTRUCTION_MODE) {
             
         /* we use the current imageId to retrieve the associated prior action and, in turn, the associated image */

         /* new  functionality for the special case where Pi = Pk (i.e. imageIdPrevious = imageIdCurrent) */
         /* This typically happens in smooth pursuit or when there                                        */
         /* is a second micro-saccade in overtAttention to refine the fixation                            */

         /* In Reconstruction mode, we skip the associative recall and simply output:                     */
         /*                                                                                               */
         /* - the same image id. and a coupling strength of 1                                             */               
         /* - zero azimuth, elevation, and vergence                                                       */

         if (imageIdCurrent == imageIdPrevious) {
            imageSimilarityCurrent = 1;
            coupling = 1;
            azimuth = 0;
            elevation = 0;
            vergence = 0;

            VectorOf<double>& out = imageIdOutPort->prepare();
            out.resize(2,0);
            out[0] = imageIdCurrent;
            out[1] = coupling;
            imageIdOutPort->write();

            Bottle& bot = actionOutPort->prepare();
            bot.addString("rel");      // "rel" since gaze is relative to current fixation 
            bot.addDouble(azimuth);    
            bot.addDouble(elevation);
            bot.addDouble(vergence);    
            actionOutPort->write();
         } 
         else {

            imageIdCurrent          = (int)  (*imageIdIn)[0];
            imageSimilarityCurrent  =        (*imageIdIn)[1];

            //graph.reconstruct(imageIdPrevious, azimuth, elevation, imageIdCurrent, coupling); 
            graph.reconstruct(node1, node2, graph.getPerceptionNode(imageIdCurrent), coupling); 
            //azimuth   = graph.getSaccadeCoordinate(1,node2);
            //elevation = graph.getSaccadeCoordinate(2,node2);
            //imageIdPrevious = graph.getImageId(node1);


            VectorOf<double>& out = imageIdOutPort->prepare();
            out.resize(2,0);
            out[0] = imageIdPrevious;
            out[1] = coupling;
            imageIdOutPort->write();

            Bottle& bot = actionOutPort->prepare();
            bot.addString("rel");      // "rel" since gaze is relative to current fixation 
            bot.addDouble(-azimuth);   // NB negative angles as we are saccading back to the previous gaze
            bot.addDouble(-elevation);
            bot.addDouble(vergence);    
            actionOutPort->write();
         }
      }

      if (false) {
         if      (mode == LEARNING_MODE)       cout << "ProceduralMemoryThread::run: Learning mode" << endl;
         else if (mode == PREDICTION_MODE)     cout << "ProceduralMemoryThread::run: Prediction mode" << endl;
         else if (mode == RECONSTRUCTION_MODE) cout << "ProceduralMemoryThread::run: Explanation mode" << endl;
         cout << "ProceduralMemoryThread::run: imageIdPrevious         " << imageIdPrevious         << endl;
         cout << "ProceduralMemoryThread::run: imageIdCurrent          " << imageIdCurrent          << endl;
         cout << "ProceduralMemoryThread::run: imageSimilarityCurrent  " << imageSimilarityCurrent  << endl;
         cout << "ProceduralMemoryThread::run: azimuth                 " << azimuth                 << endl;
         cout << "ProceduralMemoryThread::run: elevation               " << elevation               << endl;
         cout << "ProceduralMemoryThread::run: vergence                " << vergence                << endl;
         cout << "ProceduralMemoryThread::run: coupling                " << coupling                << endl;
         cout << endl;
      }

      graph.saveDatabase();
   }
}

void ProceduralMemoryThread::threadRelease() 
{
   cout << "ProceduralMemoryThread::threadRelease: saving database" << endl;

   graph.saveDatabase();

}

 
  
//----------------------------------
// ProceduralMemoryGraph 
// constructor
// ---------------------------------

ProceduralMemoryGraph::ProceduralMemoryGraph()
{
    debug = false;

    databaseName = "proceduralDatabase.txt";
    databaseContext = "";
    saccadeFrameOfReference = CARTESIAN;
    numberOfNodes  = 0;

    for (int i = 0; i < MAX_NUMBER_OF_NODES; i++ ) {
       for (int j = 0; j < MAX_NUMBER_OF_NODES; j++) {
          adjacencyMatrix[i][j] = 0;
       }
    }

    for (int i = 0; i < MAX_NUMBER_OF_NODES; i++ ) {
       for (int j = 0; j < MAX_NUMBER_OF_NODES; j++) {
          for (int k = 0; k < MAX_NUMBER_OF_NODES; k++) {
             associationMatrix[i][j][k] = 0;
          }
       }
    }

    for (int i = 0; i < MAX_NUMBER_OF_NODES; i++ ) {
       nodeList[i].nodeType = 0;
       nodeList[i].imageId  = 0;
       nodeList[i].saccadeFrameOfReference = 0;
       nodeList[i].saccadeCoordinate1      = 0;
       nodeList[i].saccadeCoordinate2      = 0;    
    }

    if (debug) {
       cout << "ProceduralMemoryGraph::ProceduralMemoryGraph: " << databaseName << " number of nodes " << numberOfNodes << endl;
    }
}

// ProceduralMemoryGraph 
// destructor

ProceduralMemoryGraph::~ProceduralMemoryGraph()
{
}


// ProceduralMemoryGraph 
// database context setter

void ProceduralMemoryGraph::setDatabaseContext(string s)
{
    databaseContext = s;
}

// ProceduralMemoryGraph 
// context getter

string ProceduralMemoryGraph::getDatabaseContext()
{
    return databaseContext;
}

// ProceduralMemoryGraph 
// change database name

void ProceduralMemoryGraph::setDatabaseName(string s)
{
    databaseName = s;
}

// ProceduralMemoryGraph 
// get database name

string ProceduralMemoryGraph::getDatabaseName()
{
    return databaseName; 
}


// ProceduralMemoryGraph 
// load the network of perceptions, actions, and association weights in the 'database' file

void ProceduralMemoryGraph::loadDatabase()
{

    string file;
    string databaseFolder;
    ifstream datafile;
    int i, j, k;
    int weight;

    databaseFolder = databaseContext;

    if (debug) 
       cout << "ProceduralMemoryGraph::loadDatabase: reading from " << (databaseFolder + "/" + databaseName).c_str() << endl;

    datafile.open((databaseFolder + "/" + databaseName).c_str(),ios::in);

    if (datafile.is_open()) {

       datafile >> numberOfNodes;

       if (debug) {
          cout << "ProceduralMemoryGraph::loadDatabase: number of nodes " << numberOfNodes << endl;

          if (datafile.eof()) 
             cout << "ProceduralMemoryGraph::loadDatabase: end of file " << endl;
       }

       for (i = 0; i < numberOfNodes; i++ ) {
          datafile >> nodeList[i].nodeType 
                   >> nodeList[i].imageId  
                   >> nodeList[i].saccadeFrameOfReference  
                   >> nodeList[i].saccadeCoordinate1  
                   >> nodeList[i].saccadeCoordinate2;
                    
          cout << i << " " 
               << nodeList[i].nodeType << " " 
               << nodeList[i].imageId  << " "  
               << nodeList[i].saccadeFrameOfReference << " "  
               << nodeList[i].saccadeCoordinate1 << " " 
               << nodeList[i].saccadeCoordinate2 << 
               endl;
       }

       while (!datafile.eof()) {
          datafile >> i >> j >> k >> weight;
          associationMatrix[i][j][k] = weight;
          cout << i << " " << j << " " << k << " " << associationMatrix[i][j][k] << endl;
       }

       // compute the adjacency matrix from the association matrix 
       // by summing along k to derive the adjacency strength between i and j
       // by summing along i to derive the adjacency strength between j and k


       for (i = 0; i < numberOfNodes; i++ ) {
          for (j = 0; j < numberOfNodes; j++) {
             for (k = 0; k < numberOfNodes; k++) {
                adjacencyMatrix[i][j] += associationMatrix[i][j][k];
             }
          }
       }

       for (i = 0; i < numberOfNodes; i++ ) {
          for (j = 0; j < numberOfNodes; j++) {
             for (k = 0; k < numberOfNodes; k++) {
                adjacencyMatrix[j][k] += associationMatrix[i][j][k];
             }
          }
       }
  
       datafile.close();

    }
	else {
		cout << "ProceduralMemoryGraph::loadDatabase: ERROR - unable to open " << (databaseFolder + "/" + databaseName).c_str() << endl;
	}   
}



// ProceduralMemoryGraph 
// save the network of perception, actions, and association weights in the 'database' file

void ProceduralMemoryGraph::saveDatabase()
{
    int i, j, k;
    string file;
    string databaseFolder;

    databaseFolder = databaseContext;

    if (debug) 
       cout << "ProceduralMemoryGraph::saveDatabase: writing to " << (databaseFolder + "/" + databaseName).c_str() << endl;

    ofstream datafile;
       
    datafile.open((databaseFolder + "/" + databaseName).c_str(), ios::out);

    if (datafile.is_open()) {
       datafile << numberOfNodes << endl;

       for (i = 0; i < numberOfNodes; i++ ) {
          datafile << nodeList[i].nodeType << " " 
                   << nodeList[i].imageId  << " "  
                   << nodeList[i].saccadeFrameOfReference << " "  
                   << nodeList[i].saccadeCoordinate1 << " " 
                   << nodeList[i].saccadeCoordinate2 << 
                   endl;
       }


/*
       for (i = 0; i < numberOfNodes; i++ ) {
          for (j = 0; j < numberOfNodes; j++) {
             if (adjacencyMatrix[i][j] != 0) {
                datafile << i << " " << j << " " << adjacencyMatrix[i][j] << endl;
             }
          }
       }
*/

       for (i = 0; i < numberOfNodes; i++ ) {
          for (j = 0; j < numberOfNodes; j++) {
             for (k = 0; k < numberOfNodes; k++) {
                if (associationMatrix[i][j][k] != 0) {
                   datafile << i << " " << j << " " << k << " " << associationMatrix[i][j][k] << endl;
                }
             }
          }
       }


       datafile.close();
    }
	else {
		cout << "ProceduralMemoryGraph::saveDatabase: ERROR - unable to open " << (databaseFolder + "/" + databaseName).c_str() << endl;
	}   

}



// ProceduralMemoryGraph 
// update the graph for a given (P, A, P) triple.

void   ProceduralMemoryGraph::updateGraph(int previousImageId, int saccadeCoordinate1, int saccadeCoordinate2, int nextImageId) {
   
   int i, j, k;
     
   i = getPerceptionNode(previousImageId);
   
   if (i>=0) {

      // don't try inserting the action node if the perception node isn't valid 
      // (this only happens in the case of the first saccade)

      j = getActionNode(saccadeFrameOfReference,saccadeCoordinate1,saccadeCoordinate2);
   }
   else {
      cout << "ProceduralMemoryGraph::updateGraph: SKIPPING THE INSERTION OF AN ACTION NODE " << endl;
   }

   k = getPerceptionNode(nextImageId);
 
     
   if (debug) {
      //cout << "ProceduralMemoryGraph::updateGraph " << i << " (" << nodeList[i].imageId << ") " << j << " (" << saccadeCoordinate1 << "," << saccadeCoordinate2 << ") " << k << " (" << nodeList[k].imageId << ") " << endl;
      cout << "ProceduralMemoryGraph::updateGraph " << i << " " << j << " " << k << ": (" << nodeList[i].imageId << ") " << " (" << saccadeCoordinate1 << "," << saccadeCoordinate2 << ") " << " (" << nodeList[k].imageId << ") " << endl;
   }

   if ((i>=0) && (j>=0) && (k>=0)) {
      adjacencyMatrix[i][j] += 1;
      adjacencyMatrix[j][k] += 1;

      associationMatrix[i][j][k] += 1;
         
      if (false) {
         cout << "ProceduralMemoryGraph::updateGraph adjacencyMatrix   [" << i << "][" << j << "]: " << adjacencyMatrix[i][j] << endl;
         cout << "ProceduralMemoryGraph::updateGraph adjacencyMatrix   [" << j << "][" << k << "]: " << adjacencyMatrix[j][k] << endl;
         cout << "ProceduralMemoryGraph::updateGraph associationMatrix [" << i << "][" << j << "][" << k << "]: " << associationMatrix[i][j][k] << endl;
      }
   }
};


// ProceduralMemoryGraph getPerceptionNode
// 
// find the perception node containing a given imageId and insert it if it doesn't exist.

int ProceduralMemoryGraph::getPerceptionNode(int imageId) {

   int i;
   int nodeNumber;
   nodeNumber = -1;

   if (imageId >= 0) {

      for (i=0; i<numberOfNodes; i++) {
         if ((nodeList[i].nodeType == PERCEPTION_NODE) && (nodeList[i].imageId == imageId) ) {
            nodeNumber = i;
         }
      }

      if (nodeNumber == -1) {

         if (numberOfNodes < MAX_NUMBER_OF_NODES-1) {    
            
            // insert it

            nodeList[numberOfNodes].nodeType = PERCEPTION_NODE;
            nodeList[numberOfNodes].imageId = imageId;
            nodeNumber = numberOfNodes; 

            numberOfNodes = numberOfNodes + 1;
         }
         else {
		    cout << "ProceduralMemoryGraph::getPerceptionNode: ERROR - maximum number of nodes reached" << endl;
         }
      }
   }

   return(nodeNumber);
}

// ProceduralMemoryGraph getActionNode
//
// find the action node containing a given saccade and insert it if it doesn't exist.

int ProceduralMemoryGraph::getActionNode(int saccadeFrameOfReference, int saccadeCoordinate1, int saccadeCoordinate2) {

   int i;
   int nodeNumber;

   nodeNumber = -1;

   for (i=0; i<numberOfNodes; i++) {
      if ((nodeList[i].nodeType == ACTION_NODE) && 
          (nodeList[i].saccadeFrameOfReference == saccadeFrameOfReference) &&
          (nodeList[i].saccadeCoordinate1 == saccadeCoordinate1) &&
          (nodeList[i].saccadeCoordinate2 == saccadeCoordinate2) ){

         nodeNumber = i;
      }
   }

   if (nodeNumber == -1) {

      if (numberOfNodes < MAX_NUMBER_OF_NODES-1) {    

         // insert it

         nodeList[numberOfNodes].nodeType                = ACTION_NODE;
         nodeList[numberOfNodes].saccadeFrameOfReference = saccadeFrameOfReference;
         nodeList[numberOfNodes].saccadeCoordinate1      = saccadeCoordinate1;
         nodeList[numberOfNodes].saccadeCoordinate2      = saccadeCoordinate2;

         nodeNumber = numberOfNodes;
         numberOfNodes++;
      }
      else {
	     cout << "ProceduralMemoryGraph::getActionNode: ERROR - maximum number of nodes reached" << endl;
      }
   }

   if (false) {
      cout << "ProceduralMemoryGraph::getActionNode: number " << nodeNumber << ": " << nodeList[nodeNumber].saccadeCoordinate1   << " "
                                                                                    << nodeList[nodeNumber].saccadeCoordinate2 
                                                                                    << endl;
   }
   return(nodeNumber);
}


// ProceduralMemoryGraph
// generate an image of the procedural memory graph 

void   ProceduralMemoryGraph::generateGraphImage(ImageOf<PixelRgb> &graphImage) {
   
   PixelRgb redPixel;
   PixelRgb greenPixel;
   PixelRgb rgbPixel;
   int i;
   int minX, maxX, minY, maxY;
   double maxWeight;
   double minWeight;
   double scaleFactor;

   /* generate the image for display */

   /* 1. traverse the graph to determine the layout                                                      */

   
   for (i=0; i<numberOfNodes; i++) {
      processed[i]  = false;
      discovered[i] = false;
      parent[i]     = -1;
   }
   layoutDepthFirstTraveral(0); // do a depth-first traversal, beginning with node 0


   /* 2. adjust the layout                                                                               */
   /*                                                                                                    */
   /*    - scale the coordinates so that they are within image bounds                                    */
   /*    - translate the coordinates so that the minimum and maximum coordinates are equidistant from    */
   /*      the image border.                                                                             */

   // first, reposition the action nodes midway between the perception nodes

   
   for (i=0; i<numberOfNodes; i++) {
      processed[i]  = false;
      discovered[i] = false;
      parent[i]     = -1;
   }

   layoutActionNodes(0);     
   


   minX = 1000;
   minY = 1000;
   maxX = 0;
   maxY = 0;

   for (int i = 0; i< numberOfNodes; i++ ) {
      if (debug) {
         // cout << "ProceduralMemoryGraph::generateGraphImage node: " << i << " layout: " << nodeList[i].layoutX << " " << nodeList[i].layoutY << endl;
      }
 
      if (nodeList[i].layoutX < minX)   
         minX = nodeList[i].layoutX;

      if (nodeList[i].layoutX > maxX)   
         maxX = nodeList[i].layoutX;

      if (nodeList[i].layoutY < minY)   
         minY = nodeList[i].layoutY;

      if (nodeList[i].layoutY > maxY)   
         maxY = nodeList[i].layoutY;
   }

   if (debug) {
      // cout << "ProceduralMemoryGraph::generateGraphImage min x,y: " << minX << ", " << minY << "; max x,y: " << maxX << ", " << maxY << endl;
   }

   if ( (( (double) graphImage.width()  - 4*NODE_RADIUS) / (maxX-minX+1)) < 
        (( (double) graphImage.height() - 4*NODE_RADIUS) / (maxY-minY+1)) ) { // only fill 90% of image
      scaleFactor = (((double) graphImage.width()  - 4*NODE_RADIUS) / (maxX-minX+1));
   }
   else {
      scaleFactor = (((double) graphImage.height() - 4*NODE_RADIUS) / (maxY-minY+1));
   }

   // cout << "ProceduralMemoryGraph::generateGraphImage scale factor: " << scaleFactor << endl;
         
   for (int i = 0; i< numberOfNodes; i++ ) {

      nodeList[i].layoutX =  (int) ((nodeList[i].layoutX - (maxX+minX)/2) * scaleFactor + graphImage.width()/2); 
      nodeList[i].layoutY =  (int) ((nodeList[i].layoutY - (maxY+minY)/2) * scaleFactor + graphImage.height()/2); 

      nodeList[i].layoutY =  graphImage.height() - nodeList[i].layoutY; // invert plot to show nodes with large Y values at the top of the image

      if (debug) {
         //cout << "ProceduralMemoryGraph::generateGraphImage node: " << i << " layout: " << nodeList[i].layoutX << " " << nodeList[i].layoutY << endl;
      }
   }


   /* 3. draw the graph                                                                                  */
   /*                                                                                                    */
   /*    - scan the list of nodes to draw the perception nodes (green) and action nodes (red)            */
   /*    - scan the adjacency matrix to draw the arcs (colour coding the connection weight)              */
   /*                                                                                                    */
   /*    No need to traverse the graph again                                                             */
 
   
   graphImage.zero();

   // draw nodes

   redPixel.r = (unsigned char) 255;
   redPixel.g = 0;
   redPixel.b = 0;
            
   greenPixel.r = 0;
   greenPixel.g = (unsigned char) 255;
   greenPixel.b = 0;

   for (int i = 0; i< numberOfNodes; i++ ) {
      if (nodeList[i].nodeType == PERCEPTION_NODE) {      
         addCircleOutline(graphImage,greenPixel,nodeList[i].layoutX,nodeList[i].layoutY,NODE_RADIUS);  
      }     
      else { 
         addCircleOutline(graphImage,redPixel,nodeList[i].layoutX,nodeList[i].layoutY,NODE_RADIUS);  
      }
   }

   // draw edges, with grey value between 128 and 255, depending on edge weight

   maxWeight = 0;
   minWeight = 1e6;

   for (int i = 0; i< numberOfNodes; i++ ) {
      for (int j = 0; j < numberOfNodes; j++) {
         if (adjacencyMatrix[i][j] > maxWeight) {
            maxWeight = adjacencyMatrix[i][j];
         }
         if (adjacencyMatrix[i][j] < minWeight) {
            minWeight = adjacencyMatrix[i][j];
         }
      }
   }

   for (int i = 0; i< numberOfNodes; i++ ) {
      for (int j = 0; j < numberOfNodes; j++) {
         if (adjacencyMatrix[i][j] > 0) {

            rgbPixel.r = (unsigned char) ((adjacencyMatrix[i][j] - minWeight) * (255-64) / (maxWeight-minWeight) + 64);
            rgbPixel.g = rgbPixel.r;
            rgbPixel.b = rgbPixel.r;

            drawEdge(graphImage, rgbPixel, nodeList[i].layoutX,nodeList[i].layoutY, nodeList[j].layoutX,nodeList[j].layoutY, NODE_RADIUS);
         }
      }
   }
}

// ProceduralMemoryGraph layoutDepthFirstTraveral
//
// traverse the procedural memory graph to determine the initial layout

void   ProceduralMemoryGraph::layoutDepthFirstTraveral(int nodeNumber) {

   /* traverse the graph to determine the layout                                                         */
   /*                                                                                                    */
   /*    - the first perception node is assigned coordinates (0, 0)                                      */
   /*    - subsequent nodes are positioned relative to the parent node, based on the saccade coordinates */
   /*      with saccade coordinates extracted either from the parent or the current node, as appropriate */
   /*    - node positions are recorded in the node attribute fields                                      */

   int i;
   int xOffset;
   int yOffset;

   discovered[nodeNumber] = true;
  
   // process this node by assigning layout coordinates
   
   if (nodeNumber == 0) {

      // initial call to this recursive function so assign the root node to the origin

      nodeList[nodeNumber].layoutX = 0;
      nodeList[nodeNumber].layoutY = 0;

      // cout << "ProceduralMemoryGraph::layoutDepthFirstTraveral ROOT image " << nodeList[nodeNumber].imageId << " at (" << nodeList[nodeNumber].layoutX << "," << nodeList[nodeNumber].layoutY << ") "  << endl;

   }
   else {
      if (nodeList[nodeNumber].nodeType == PERCEPTION_NODE) {      
         xOffset =  (int) nodeList[parent[nodeNumber]].saccadeCoordinate1;   
         yOffset =  (int) nodeList[parent[nodeNumber]].saccadeCoordinate2;         
      }
      else if (nodeList[nodeNumber].nodeType == ACTION_NODE) {          
         xOffset =  (int) nodeList[nodeNumber].saccadeCoordinate1;          
         yOffset =  (int) nodeList[nodeNumber].saccadeCoordinate2;               
      }

      nodeList[nodeNumber].layoutX = nodeList[parent[nodeNumber]].layoutX + xOffset;
      nodeList[nodeNumber].layoutY = nodeList[parent[nodeNumber]].layoutY + yOffset;

      if (nodeList[nodeNumber].nodeType == PERCEPTION_NODE) {      
         // cout << "ProceduralMemoryGraph::layoutDepthFirstTraveral node " << nodeNumber << " image " << nodeList[nodeNumber].imageId << " at (" << nodeList[nodeNumber].layoutX << "," << nodeList[nodeNumber].layoutY << ")"  << endl;
      }

      // now make sure that this position is not already occupied

      /*
      do {
         bool relocated = false;
         for (i=0; i<numberOfNodes; i++) {
            if (i != nodeNumber) {
               if ((nodeList[nodeNumber].layoutX == nodeList[i].layoutX) && 
                   (nodeList[nodeNumber].layoutY == nodeList[i].layoutY)) {
                      nodeList[nodeNumber].layoutX += NODE_RADIUS;
                      nodeList[nodeNumber].layoutY += NODE_RADIUS;
                      relocated = true;
                        
                      if (debug)
                         cout << "ProceduralMemoryGraph::layoutDepthFirstTraveral RELOCATING: " << nodeNumber << " layout: " << nodeList[nodeNumber].layoutX << " " << nodeList[nodeNumber].layoutY << endl;
                
               }
            }
         }
      } while (relocated == true);
      */
   }

   if (debug) {
      //cout << "ProceduralMemoryGraph::layoutDepthFirstTraveral node: " << nodeNumber << " layout: " << nodeList[nodeNumber].layoutX << " " << nodeList[nodeNumber].layoutY << endl;
   }

   // now visit the nodes to which it is connected

   for (i=0; i<numberOfNodes; i++) {
      if (adjacencyMatrix[nodeNumber][i] > 0) {
         if (discovered[i] == false) {
            parent[i] = nodeNumber;
            layoutDepthFirstTraveral(i); // recursive descent
         } 
      }
   }
   processed[nodeNumber] = true;
};


/*
 * ProceduralMemoryGraph layoutActionNodes
 */

void   ProceduralMemoryGraph::layoutActionNodes(int nodeNumber) {

   /* traverse the graph to determine the layout                                                         */
   /*                                                                                                    */
   /*    - position action nodes mid-way between the perception nodes they connect                       */
   /*    - node positions are recorded in the node attribute fields                                      */

   int i;
   int offspring;
   bool relocated;

   discovered[nodeNumber] = true;
  

   if (nodeList[nodeNumber].nodeType == ACTION_NODE) {      
        
      offspring = -1;
      for (i=0; i<numberOfNodes; i++) {
         if ((adjacencyMatrix[nodeNumber][i] > 0) && (nodeList[i].nodeType == PERCEPTION_NODE)) {
            offspring = i;
            break;
         } 
      }

      nodeList[nodeNumber].layoutX = (int)(nodeList[parent[nodeNumber]].layoutX + nodeList[offspring].layoutX)/2;
      nodeList[nodeNumber].layoutY = (int)(nodeList[parent[nodeNumber]].layoutY + nodeList[offspring].layoutY)/2;
 
      // now make sure that this position is not already occupied by a perception node
      // (i.e. don't let an action node occlude a perception node)

      do {
         relocated = false;
         for (i=0; i<numberOfNodes; i++) {
            if (i != nodeNumber) {
               if ((nodeList[nodeNumber].layoutX == nodeList[i].layoutX) && 
                   (nodeList[nodeNumber].layoutY == nodeList[i].layoutY)  &&
                   (nodeList[i].nodeType == PERCEPTION_NODE)) {
                      nodeList[nodeNumber].layoutX += NODE_RADIUS;
                      nodeList[nodeNumber].layoutY += NODE_RADIUS;

                      if (false) 
						  cout << "ProceduralMemoryGraph::layoutActionNodes relocating node: " << nodeNumber << " layout: " << nodeList[nodeNumber].layoutX << " " << nodeList[nodeNumber].layoutY << endl;
                      relocated = true;
               }
            }
         }
      } while (relocated == true);

      if (debug) {
         //cout << "ProceduralMemoryGraph::layoutActionNodes node: " << nodeNumber << " layout: " << nodeList[nodeNumber].layoutX << " " << nodeList[nodeNumber].layoutY << endl;
      }
   }

   // now visit the nodes to which it is connected

   for (i=0; i<numberOfNodes; i++) {
      if (adjacencyMatrix[nodeNumber][i] > 0) {
         if (discovered[i] == false) {
            parent[i] = nodeNumber;
            layoutActionNodes(i); // recursive descent
         } 
      }
   }
   processed[nodeNumber] = true;
};


/*
 * ProceduralMemoryGraph coupling
 * 
 * Compute coupling between a given (P_x, A_y, P_z) triple.
 *
 */

double ProceduralMemoryGraph::coupling(int x, int y, int z) {
   
   double coupling;

   coupling = 0;

   /* check bounds */

   if (x<0 || x>=numberOfNodes) {
      cout << "ProceduralMemoryGraph::coupling Error: invalid node number " << x << endl; 
      return 0;
   }
   else if (y<0 || y>=numberOfNodes) {
      cout << "ProceduralMemoryGraph::coupling Error: invalid node number " << y << endl; 
      return 0;
   }
   else if (z<0 || z>=numberOfNodes) {
      cout << "ProceduralMemoryGraph::coupling Error: invalid node number " << z << endl; 
      return 0;
   }
   else if (nodeList[x].nodeType != PERCEPTION_NODE) {
      cout << "ProceduralMemoryGraph::coupling Error: node number " << x << " is not a perception node" << endl; 
      return 0;
   }
   else if (nodeList[y].nodeType != ACTION_NODE) {
      cout << "ProceduralMemoryGraph::coupling Error: node number " << y << " is not an action node" << endl;
      return 0;
   }
   else if (nodeList[z].nodeType != PERCEPTION_NODE) {
      cout << "ProceduralMemoryGraph::coupling Error: node number " << z << " is not a perception node" << endl;
      return 0;
   }
   else {

      coupling = associationMatrix[x][y][z];

      /* alternative */

      /* s = weight(P_x, A_y) * weight(A_y, P_z) / [Sum_i Sum_j weight(P_i, A_j) * Sum_i Sum_j weight(A_i, P_j)] */
/*

   double weight1;
   double weight2;
   int i, j;

      weight1 = 0;
      for (i = 0; i< numberOfNodes; i++ ) {
         if (nodeList[i].nodeType == PERCEPTION_NODE) {
            for (j = 0; j < numberOfNodes; j++) {
               if (nodeList[j].nodeType == ACTION_NODE) {
                  weight1 = weight1 + adjacencyMatrix[i][j];
               }
            }
         }
      }

      weight2 = 0;
      for (i = 0; i< numberOfNodes; i++ ) {
         if (nodeList[i].nodeType == ACTION_NODE) {
            for (j = 0; j < numberOfNodes; j++) {
               if (nodeList[j].nodeType == PERCEPTION_NODE) {
                  weight2 = weight2 + adjacencyMatrix[i][j];
               }
            }
         }
      }

      coupling = (adjacencyMatrix[x][y] * adjacencyMatrix[y][z]) / (weight1 * weight2);
           
*/

      if (false) {
         cout << "ProceduralMemoryGraph::coupling (" << x << ", " << y << ", " << z << "): " << coupling << endl;
      }
   }

   return coupling;
};

 

// ProceduralMemoryGraph predict
// Given the first P in a (P, A, P) triple, predict the subsequent associated A-P.

void ProceduralMemoryGraph::predict(int x, int & y, int & z, double & eventCoupling) {
 
   double weight1;
   double weight2;
    
   int j, k;

   int maxj;
   int maxk;

   y = 0;
   z = 0;
   eventCoupling = 0;

   /* check for valid parameters */

   if (x < 0 || x >= numberOfNodes) {
      cout << "ProceduralMemoryGraph::predict Error: invalid node number " << x << endl; 
   }
   else if (nodeList[x].nodeType != PERCEPTION_NODE) {
      cout << "ProceduralMemoryGraph::predict Error: node " << x << " is not a perception node" << endl;
   }
   else {

      /* find action-image pair with greatest weight associated with previousImageId */

      maxj = 0;
      maxk = 0;

      weight1 = 0;
      for (j = 0; j < numberOfNodes; j++ ) {
         if (nodeList[j].nodeType == ACTION_NODE) {
            if (adjacencyMatrix[x][j] > weight1) {
               weight1 = adjacencyMatrix[x][j];
               maxj = j;
            }
         }
      }
      
      weight2 = 0;
      for (k = 0; k < numberOfNodes; k++ ) {
         if (nodeList[k].nodeType == PERCEPTION_NODE) {
            if (associationMatrix[x][maxj][k] > weight2) {
               weight2 = associationMatrix[x][maxj][k];
               maxk = k;
            }
         }
      }

      y = maxj;
      z = maxk;

      eventCoupling = coupling(x, y, z);

   }

   if (false)
	   cout << "ProceduralMemoryGraph::predict     "  << x << " " << y << " " << z <<  ": " << eventCoupling << endl;

}

//ProceduralMemoryGraph given the second P in a (P, A, P) triple, reconstruct the prior associated P-A.

void ProceduralMemoryGraph::reconstruct(int & x, int & y, int z, double & eventCoupling) {
 
   double weight1;
   double weight2;
    
   int i, j;

   int maxi;
   int maxj;

   x = 0; 
   y = 0;
   eventCoupling = 0;

   /* check for valid parameters */

   if (z < 0 || z >= numberOfNodes) {
      cout << "ProceduralMemoryGraph::reconstruct Error: invalid node number " << z << endl; 
   }
   else if (nodeList[z].nodeType != PERCEPTION_NODE) {
      cout << "ProceduralMemoryGraph::reconstruct Error: node " << z << " is not a perception node" << endl;
   }
   else {

      /* find action-image pair with greatest weight associated with currentImageId */

      maxi = 0;
      maxj = 0;

      weight1 = 0;
      for (j = 0; j < numberOfNodes; j++ ) {
         if (nodeList[j].nodeType == ACTION_NODE) {
            if (adjacencyMatrix[j][z] > weight1) {
               weight1 = adjacencyMatrix[j][z];
               maxj = j;
            }
         }
      }

      weight2 = 0;
      for (i = 0; i < numberOfNodes; i++ ) {
         if (nodeList[i].nodeType == PERCEPTION_NODE) {
            if (associationMatrix[i][maxj][z] > weight2) {
               weight2 = associationMatrix[i][maxj][z];
               maxi = i;
            }
         }
      }
      x = maxi;
      y = maxj;

      eventCoupling = coupling(x, y, z);

   }

   if (false)
	   cout << "ProceduralMemoryGraph::reconstruct "  << x << " " << y << " " << z  <<  ": " << eventCoupling << endl;

}

//ProceduralMemoryGraph lineCircleIntersection

void ProceduralMemoryGraph::lineCircleIntersection(int x1, int y1, int x2, int y2, int cx, int cy, int r,
                                                   int & xa, int & ya, int & xb, int & yb) {
 
   double a, b, c, m, xs1, ys1, xs2, ys2;

   xa = 0;
   ya = 0;
   xb = 0;
   yb = 0;

   if (x1 == x2) {

      // slope is infinite

      xa = cx;
      ya = cy - r;
      xb = cx;
      yb = cy + r;
   }
   else {
      m = (double)(y2-y1)/(double)(x2-x1);
      a = 1 + m*m;
      b = -2 * (cx + m*m*x1 - m*y1 + m*cy);
      c = cx*cx + cy*cy + y1*y1 + m*m*x1*x1 - r*r - 2*m*x1*y1 + 2*cy*m*x1 - 2*cy*y1;

      xs1 = (-b + sqrt(b*b - 4*a*c) ) / (2*a);
      ys1 = m * (xs1 - x1) + y1;

      xs2 = (-b - sqrt(b*b - 4*a*c) ) / (2*a);
      ys2 = m * (xs2 - x1) + y1;

      xa = (int) xs1;
      ya = (int) ys1;

      xb = (int) xs2;
      yb = (int) ys2;
   } 
   
   /*
   cout << "ProceduralMemoryGraph::lineCircleIntersection "  << x1 << " " << y1 << " " 
                                                             << x2 << " " << y2 << " " 
                                                             << cx << " " << cy << " " << r << " "
                                                             << xa << " " << ya << " " 
                                                             << xb << " " << yb << " " 
                                                             << endl;
   */ 
}


// ProceduralMemoryGraph shortestSegment
// Given two groups of (colinear) points, select the pair of points that defining the shortest segment between them

void ProceduralMemoryGraph::shortestSegment(int x1, int y1, int x2, int y2, int x3, int y3, int x4, int y4,
                                            int & xa, int & ya, int & xb, int & yb) {
   double d1, d2, d3, d4;

   xa = 0;
   ya = 0;
   xb = 0;
   yb = 0;

   d1 = sqrt( (double)(x1-x3)*(x1-x3) + (double)(y1-y3)*(y1-y3) );
   d2 = sqrt( (double)(x1-x4)*(x1-x4) + (double)(y1-y4)*(y1-y4) );
   d3 = sqrt( (double)(x2-x3)*(x2-x3) + (double)(y2-y3)*(y2-y3) );
   d4 = sqrt( (double)(x2-x4)*(x2-x4) + (double)(y2-y4)*(y2-y4) );

   if      ((d1<=d2) && (d1<=d3) && (d1<=d4)) {
      xa=x1; ya=y1; xb=x3; yb=y3;
   }
   else if ((d2<=d1) && (d2<=d3) && (d2<=d4)) {
      xa=x1; ya=y1; xb=x4; yb=y4;
   }
   else if ((d3<=d1) && (d3<=d2) && (d3<=d4)) {
      xa=x2; ya=y2; xb=x3; yb=y3;
   }
   else if ((d4<=d1) && (d4<=d2) && (d4<=d3)) {
      xa=x2; ya=y2; xb=x4; yb=y4;
   }
 
   /*
   cout << "ProceduralMemoryGraph::shortestSegment "  << x1 << " " << y1 << " " 
                                                      << x2 << " " << y2 << " " 
                                                      << x3 << " " << y3 << " " 
                                                      << x4 << " " << y4 << " " 
                                                      << xa << " " << ya << " " 
                                                      << xb << " " << yb << " " 
                                                      << endl;
   */
}



// ProceduralMemoryGraph drawEdge
// draw an arrow from one node, centred at (x1, y1), to another, centred at (x2, y2); the radius of the nodes is r
// the arrow is drawn between the boundaries of the nodes, not between their centres

void ProceduralMemoryGraph::drawEdge(ImageOf<PixelRgb> &graphImage, PixelRgb rgbPixel, 
                                     int x1, int y1, int x2, int y2, int r) {

   // cout << "ProceduralMemoryGraph::drawEdge "  << x1 << " " << y1 << " " << x2 << " " << y2 << " " << r  << " " << endl;
 
   int xa, ya, xb, yb;
   int xp, yp, xq, yq;
   int i1, j1, i2, j2, i3, j3;
   double i_offset, j_offset, theta;
   double phase_value;

   lineCircleIntersection(x1, y1, x2, y2,   x1, y1, r, xa, ya, xb, yb);
   lineCircleIntersection(x1, y1, x2, y2,   x2, y2, r, xp, yp, xq, yq);
   shortestSegment(xa, ya, xb, yb, xp, yp, xq, yq, i1, j1, i2, j2);

   /* arrow body */

   addSegment(graphImage,rgbPixel,i1, j1, i2, j2);  

   /* arrow head */

   phase_value = atan2((double)(j2-j1),(double)(i2-i1));

   theta = phase_value + 3.14159 + ARROW_HEAD_ANGLE;
   i_offset = ARROW_HEAD_SIZE * cos(theta);
   j_offset = ARROW_HEAD_SIZE * sin(theta);

   i3 = i2 + (int)(i_offset);
   j3 = j2 + (int)(j_offset);

   addSegment(graphImage,rgbPixel,i2, j2, i3, j3);  

   theta = phase_value + 3.14159 - ARROW_HEAD_ANGLE;
   i_offset = ARROW_HEAD_SIZE * cos(theta);
   j_offset = ARROW_HEAD_SIZE * sin(theta);

   i3 = i2 + (int)(i_offset);
   j3 = j2 + (int)(j_offset);

   addSegment(graphImage,rgbPixel,i2, j2, i3, j3);  
 
}



