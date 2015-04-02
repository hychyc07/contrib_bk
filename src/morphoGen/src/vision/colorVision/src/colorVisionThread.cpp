// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file colorVisionThread.cpp
 * @brief Implementation of the eventDriven thread (see colorVisionThread.h).
 */

#include <colorVisionThread.h>
#include <cstring>
#include <string>
#include "fileutil.h"
#include <time.h>
#include <math.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
     
	
colorVisionThread::colorVisionThread() {
    robot = "icub";        
}

colorVisionThread::colorVisionThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

colorVisionThread::~colorVisionThread() {
    // do nothing
}

bool colorVisionThread::threadInit() {
   
    printf("colorVisionThread thread init \n")   ;
    
    // initialization of the variables
    


    // opening ports
    if (!outputPort.open(getName("/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!inputPort.open(getName("/img:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    if (!dataPort.open(getName("/colordata:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }     
    
    if (!imagePortRTL.open(getName("/imagePortRTL").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    //Network::connect("/icub/camcalib/left/out","/v1/imagePortRTL");
	//Network::connect("/v1/imagePortRTL","/v/l"); 

    
    ModelLoad();
  
    // allocating memory
    d_trws = new real[nK];
     	
    return true;
  
}

void colorVisionThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string colorVisionThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void colorVisionThread::setInputPortName(string InpPort) {
    
}

void colorVisionThread::run() {    
    while (isStopping() != true) {
        
        if (imagePortRTL.getInputCount()) {   
            //========================================================================================

            //for ( int i=0; i<52; i++ )  {
                //tdGpmp[i] = 0;
            //}
            
            
            colSegMainL();
            //tdGpmp[0] = numobjectsL;
            
		    //	 imagePortRT.open("/v1/imagePortRT");
            //   Network::connect("/icub/camcalib/right/out","/v1/imagePortRT");
            //	 colSegMainR();
            //	 tdGpmp[1] = numobjectsR;
    
			
            //int k=1;  //modified to account for the left cam only
                        
            //printf("printing the details \n");
            /*
            for (int i=0; i<numobjectsL; i++ )	{
                for (int j=0; j<5; j++ )    {
                    tdGpmp[k] = imageDetailsL[i][j];
                    printf("%f \n", imageDetailsL[i][j]);
                    k=k+1;
                }
			}
            */
            /* 
            for (int i=0; i<numobjectsR; i++ )
                {
                    for (int j=0; j<5; j++ )
                        {
                            tdGpmp[k] = imageDetails[i][j];
                            printf("%f \n", imageDetails[i][j]);
                            k=k+1;
                        }
                }
            */
            
            //cout << "ready to send data"<< endl;

            if(dataPort.getOutputCount()) {
                Bottle& ColOp = dataPort.prepare();
                ColOp.clear();   
                ColOp.addInt(numobjectsL);
                Bottle objectBottle;

                for (int i = 0; i < numobjectsL; i++) {
                    Bottle& objectBottle = ColOp.addList();
                    objectBottle.clear();
                    objectBottle.addInt(imageDetailsL[i][0]);
                    objectBottle.addInt(imageDetailsL[i][2]);
                    objectBottle.addInt(imageDetailsL[i][1]-imageDetailsL[i][0]);
                    objectBottle.addInt(imageDetailsL[i][3]-imageDetailsL[i][2]);
                    objectBottle.addInt(imageDetailsL[i][4]);
                    // cout << " object Ids"<< tdGpmp[i] << endl;
                }
                
                dataPort.write();
                //Time::delay(5);
            }    
            
            
            
            //========================================================================================     
        }
        
        //if(outputPort.getOutputCount()) {
            
        //    outputPort.prepare() = *inputImage;
        //    outputPort.write();   
            
        //}

        
        
        
        //Time::delay(0.1);

    } // end if while isStopping
    
}

void colorVisionThread::threadRelease() {
    //nothing here

    printf("\n freeing memory \n");
    
    free(K);
    free(q);
    free(E);
    free(f);
    free(W);
    free(colormap);


   

    printf("after freeing memory");

}

void colorVisionThread::onStop() {
    outputPort.interrupt();
    inputPort.interrupt();
    dataPort.interrupt();
    imagePortRTL.interrupt();


    outputPort.close();
    inputPort.close();
    dataPort.close();
    imagePortRTL.close(); 
}

double* colorVisionThread::Vision(int ObjectType) {
	 //loads model and colormap
    int colorcall= colSegMainR();// segments right cam
    
    for ( int i=0; i<numobjectsR; i++ ) {
         //  printf("\n %f \t %f\t %f\t %f\t %f\t", imageDetails[i][0], imageDetails[i][1], imageDetails[i][2],imageDetails[i][3],imageDetails[i][4]);
        ObjectIdentityR[i][0]= imageDetails[i][0];
        ObjectIdentityR[i][1]= imageDetails[i][1];
        ObjectIdentityR[i][2]= imageDetails[i][2];
        ObjectIdentityR[i][3]= imageDetails[i][3];
        ObjectIdentityR[i][4]= imageDetails[i][4];
    }
     
	colSegMainL(); // segments left cam
    for ( int i=0; i<numobjectsL; i++ ) {
         // printf("\n %f \t %f\t %f\t %f\t %f\t", imageDetailsL[i][0], imageDetailsL[i][1], imageDetailsL[i][2],imageDetailsL[i][3],imageDetailsL[i][4]);
        ObjectIdentityL[i][0]= imageDetailsL[i][0];
        ObjectIdentityL[i][1]= imageDetailsL[i][1];
        ObjectIdentityL[i][2]= imageDetailsL[i][2];
        ObjectIdentityL[i][3]= imageDetailsL[i][3];
        ObjectIdentityL[i][4]= imageDetailsL[i][4];
    }
     
    if(numobjectsR == numobjectsL) {// Assumption here that there is correspondance in ordering of images in both cameras     
        for ( int i=0; i<numobjectsL; i++ ) { 
            ImagX=0;
            ImagY=0;
            ImagZ=0;
            double ur=(ObjectIdentityR[i][0]+ObjectIdentityR[i][1])*0.5;
            double vr=(ObjectIdentityR[i][2]+ObjectIdentityR[i][3])*0.5;
            double ul=(ObjectIdentityL[i][0]+ObjectIdentityL[i][1])*0.5;
            double vl=(ObjectIdentityL[i][2]+ObjectIdentityL[i][3])*0.5;
               
            ConvImg(ur,vr,ul,vl);
            ObjectIdentityR[i][5]=ImagX;
            ObjectIdentityR[i][6]=ImagY;
            ObjectIdentityR[i][7]=ImagZ; 
        }
             
    }
     
    for ( int i=0; i<41; i++ )  {
        tdGpmp[i] = 0;
    }
     
    tdGpmp[0] = numobjectsL;
    int k=1;
    for (int i=0; i<numobjectsL; i++ )  {
        for (int j=0; j<8; j++ )    {
            tdGpmp[k] = ObjectIdentityR[i][j];
            k=k+1;
        }
    }
    return tdGpmp;
};


bool colorVisionThread::ModelLoad() {
	// extract parameters from properties (ini-file)
	
	//height = rf.find("height").asInt();
	//width = rf.find("width").asInt();
	
	// string modelFile = rf.find("modelFile").asString();
	//string modelFile = rf.find("model.txt").asString();
    // FILE *model,*colorMaper;
	// model=fopen("w1.txt","r");//input file to be given to the program	
     //colorMaper=fopen("w2.txt","r");

	//char* modelPtr ="model.txt";
    char* modelPtr = (char*) modelPath.c_str();
	int nDim;
	load_numeric_array(modelPtr,&nK,&nDim,&W);
	if ( nDim!=8 ) {
		printf("Wrong size of array in file <%s>.\n",modelPtr);
		return false;
	}
	printf("Segmentation model loaded from file <%s>, with %i labels.\n",modelPtr,nK);

	// Divide some components of SVM weights by 255 (because RGB values in I are in interval [0,255] rather than [0,1]).
	for ( int k=0; k<nK; k++ ) {
		for ( int i=1; i<5; i++ ) {
            W[8*k+i] /= 255.0;
        }
		for ( int i=5; i<8; i++ ) {
            W[8*k+i] /= 255.0*255.0;
        }
	}

	//string colormapFile = rf.find("colormapFile").asString();

	//char* colormapPtr ="colormap.txt";
    char* colormapPtr = (char*) colorMapPath.c_str();
	int n3, nK_;
	load_numeric_array(colormapPtr,&nK_,&n3,&colormap);
	if ( n3!=3 || nK_<nK ) {
		printf("Error: Color map in file <%s> must have 3 columns and at least that many rows as <model>.\n",colormapFile.c_str());
		return false;
	}
	printf("Colormap loaded from file <%s>.\n",colormapPtr);

	smoothness = 3;
	niter =5 ;
	minsize = 200;
	
	q = NULL;

	/* // open thread input and output ports:
	if(    !cam_in.open(("/"+threadName+"/img:i").c_str()) 
		|| !cam_out.open(("/"+threadName+"/img:o").c_str())
		|| !output.open(("/"+threadName+"/out").c_str()) )
	{
		printf("Error: unable to open input or output port\n");
		return false;
	}*/ 

 
	return true;
}


int colorVisionThread::colSegMainR()    {
	ModelLoad(); 
    int greendetect=-1;
	//BufferedPort<Bottle> out1;
    // Name the ports
	int averager=0;       
	 
	double xMean = 0;
    double yMean = 0;
    // read an image from the port
    ImageOf<PixelRgb> *imgRT = imagePortRTL.read();
    int ctR=0; 
    if (imgRT!=NULL) { // check we actually got something
        printf("We got an image of size %dx%d\n", imgRT->width(), imgRT->height());

	    if ( q == NULL ) {
		    printf("Image size known, allocating MRF arrays.\n");
		    init_onsize(imgRT->width(),imgRT->height());
	    }
	    if ( (imgRT->width()!=width) || (imgRT->height()!=height) ) {
		    printf("Error: Image size changed during session.\n");
		    return 100;
	    }
	    ImageOf<PixelRgb> & outP    = outputPort.prepare(); 
        outP.resize(width, height);
        outP.zero();
	    unsigned char *I = imgRT->getRawImage();
	
	    printf("Constructing and optimizing MRF.\n");
	    compute_unary_potentials(I,W,nK,width*height,q);
	    reparameterize_unary_potentials(E,nE,q,width*height,nK,f);
	    for ( int iter=0; iter<niter; iter++ ) {
		    float resid = trws_potts(E,nE,smoothness,q,width*height,nK,f,.5);
		    printf("iter=%i resid=%g\n",iter,resid);
	    }
	    extract_labeling(q,nK,width*height,K);

	    printf("Segmentation done. Label histogram=[");
	    int *hist = new int[nK];
	    for ( int k=0; k<nK; k++ ) hist[k]=0;
	    for ( int i=0; i<width*height; i++ ) hist[K[i]]++;
	    for ( int k=0; k<nK; k++ ) printf("%i ",hist[k]);
	    printf("]\n");
	    delete[] hist;

	    printf("Finding connected components.\n");
	    for ( int i=0; i<width; i++ ) K[i] = 0; // Set column 0 to background
	    for ( int j=0; j<height; j++ ) K[j*width] = 0; // Set row 0 to background
	    unsigned ncomponents = connected_components(K,width,height,minsize,J);
	    printf("No. of components (excl. background) = %i\n",ncomponents);

	    //printf("Computing bounding boxes.\n");
	    int *bbox = new int[5*ncomponents];
	    bounding_boxes(J,K,width,height,ncomponents,bbox);
	    for ( int k=0; k<ncomponents; k++ ) {
		    int *b = bbox + 5*k;
		    //printf("%i %i %i %i %i\n",b[0],b[1],b[2],b[3],b[4]);
		    for ( int i=b[0]; i<=b[1]; i++ ) K[i+width*b[2]] = K[i+width*b[3]] = b[4];
		    for ( int j=b[2]; j<=b[3]; j++ ) K[b[0]+width*j] = K[b[1]+width*j] = b[4];
	    }

	    // Apply color map and send image to output.
	    for ( int t=0; t<width*height; t++ ) {
		    unsigned char Kt = K[t], *It = I + 3*t;
		    for ( int c=0; c<3; c++ ) if ( Kt>0 ) It[c] = colormap[3*Kt+c];
	    }
	    outP = *imgRT;
        outputPort.write();
	    //imagePortRT.prepare() = *imgRT;
        //imagePortRT.write();
	
	    printf("\n number of objects %d ", ncomponents);
	    numobjectsR= ncomponents;
	    int tempV;
        countee=0;
	    int counteei=0;
	  
	    for ( int i=0; i<5*ncomponents; i++ )   {
	        bbvals[i]=bbox[i];
            tempV=bbvals[i];
            imageDetails[counteei][countee]=tempV;
	        if(countee==4)  {
		        if(imageDetails[counteei][countee]==3)  {
	                greendetect=1;
	            }
	            countee=0;
            	counteei=counteei+1;
	        }
	        else {
		        countee=countee+1;
	        }
	        printf("\n bbval %d ", bbvals[i]);
	    }
        //delete bbox;
	    printf("\n");
    }
    
    free(K);
    free(q);
    free(E);
    free(f);
    free(W);
    return greendetect;
}


void colorVisionThread::colSegMainL()   {

	// read an image from the port
    ImageOf<PixelRgb> *imgRTL = imagePortRTL.read(true);
    
    
    
    //runSem.wait();
    if (imgRTL!=NULL) { // check we actually got something
        //printf("We got an image of size %dx%d\n", imgRTL->width(), imgRTL->height());
        
        if ( q == NULL ) {
            //	printf("Image size known, allocating MRF arrays.\n");
            init_onsize(imgRTL->width(),imgRTL->height());
        }
        if ( (imgRTL->width()!=width) || (imgRTL->height()!=height) ) {
            //	printf("Error: Image size changed during session.\n");
            return;
        }
        ImageOf<PixelRgb>& outP    = outputPort.prepare(); 
        outP.resize(width, height);
        outP.zero(); 
        
        unsigned char *I = imgRTL->getRawImage();


        //unsigned char *O = outP.getRawImage();
        
        //printf("Constructing and optimizing MRF.\n");
        // takes the colormap and find the object ID
        compute_unary_potentials(I,W,nK,width*height,q); 
        //printf("interval for compute_unary_potentials = %f \n",intervalFunc);
        reparameterize_unary_potentials(E,nE,q,width*height,nK,f);
        for ( int iter=0; iter<niter; iter++ ) {
            float residL = trws_potts(E,nE,smoothness,q,width*height,nK,f,.5);
            // printf("iter=%i resid=%g\n",iter,residL);
        }
        //printf("preparing extract \n");
        extract_labeling(q,nK,width*height,K);
       
               //printf("after extract \n");
         
        /*  Rea: commented out because not useful
        //printf("Segmentation done. Label histogram=[");
        int *histL = new int[nK];
        for ( int k=0; k<nK; k++ ) {
            histL[k]=0;
        }
        for ( int i=0; i<width*height; i++ ) {
            histL[K[i]]++;
        }
        // for ( int k=0; k<nK; k++ ) printf("%i ",histL[k]);
        //printf("]\n");
        delete[] histL;
        */
        
                 
        //printf("Finding connected components.\n");
        for ( int i=0; i<width; i++ ) {
            K[i] = 0; // Set column 0 to background
        }
        for ( int j=0; j<height; j++ ) {
            K[j*width] = 0; // Set row 0 to background
        }
        unsigned ncomponentsL = connected_components(K,width,height,minsize,J);
        // printf("No. of components (excl. background) = %i\n",ncomponentsL);
        
        
        //printf("Computing bounding boxes.\n");
        int *bboxL = new int[5*ncomponentsL];
        bounding_boxes(J,K,width,height,ncomponentsL,bboxL);
                    
        for ( int k=0; k<ncomponentsL; k++ ) {
            int *bL = bboxL + 5*k;
            //printf("%i %i %i %i %i\n",b[0],b[1],b[2],b[3],b[4]);
            for ( int i=bL[0]; i<=bL[1]; i++ ) {
                K[i+width*bL[2]] = K[i+width*bL[3]] = bL[4];
            }
            for ( int j=bL[2]; j<=bL[3]; j++ ) {
                K[bL[0]+width*j] = K[bL[1]+width*j] = bL[4];
            }
        }
        

        // Apply color map and send image to output.
        for ( int t=0; t<width*height; t++ ) {
            unsigned char KtL = K[t], *ItL = I + 3*t;
            for ( int c=0; c<3; c++ ) {
                if ( KtL>0 ) ItL[c] = colormap[3*KtL+c];
            }
       }
       
       //outputPort.prepare()    =   *imgRTL;
        outP = (const ImageOf<PixelRgb>)*imgRTL;
       outputPort.write();
        
        //imagePortRTL.prepare() = *imgRTL;
        //imagePortRTL.write();
        
        /*Bottle bot;
          bot.addInt(ncomponents); // number of bounding boxes
          
          for ( int i=0; i<5*ncomponents; i++ ) bot.addInt(bbox[i]);
          output.write(bot);*/ 
         
        //printf("\n number of objects %d ", ncomponentsL);
        numobjectsL= ncomponentsL;
        int tempVL;
        counteeL=0;
        int counteeiL=0;
        /*   for ( int i=0; i<10; i++ )
             {
             for ( int j=0; j<8; i++ )
             {
             imageDetailsL[i][j]=0;
             }
             
             }*/ 
        
        
        for ( int i=0; i< 5 * ncomponentsL; i++ )   {
                bbvalsL[i]=bboxL[i];
                tempVL=bbvalsL[i];
                imageDetailsL[counteeiL][counteeL]=tempVL;
                if(counteeL==4) {
                        counteeL=0;
                        counteeiL=counteeiL+1;
                }
                else    {
                    counteeL=counteeL+1;
                }
                //printf("\n bbvalL %d ", bbvalsL[i]);
        }
        
         
        delete bboxL;
       
         
    }
    //runSem.post();
}


/*
// MAP inference in the MRF by a message-passing algorithm (TRW-S).
// See [V.Kolmogorov: Convergent Tree-reweighted Message Passing for Energy Minimization].
real colorVisionThread::trws_potts( // returns residual
  const unsigned *E, // edges
  int nE, // nr of edges
  real w, // edge weights (non-negative)
  real *q, // array (nK,nT), unary potentials
  int nT, // nr of pixels
  int nK, // nr of labels
  real *f, // array (nK,2,nE), messages
  real gamma // TRW-S edge occurence probability (for grid graph, set gamma=1/2)
)
{

    printf("trws_potts class function \n");
    
    real resid   = 0;
    //d_trws = new real[nK]; 
    real INF     = 1E10;
    
    // forward pass
    for ( int e=0; e<nE; e++ ) {
        real *qe = q + nK*E[0+e*2],
            *qee = q + nK*E[1+e*2],
            *fe = f + nK*(0+e*2),
            *fee = fe + nK;
        
        real a = -INF;
        for ( int k=0; k<nK; k++ ) {
            d_trws[k] = gamma*qe[k] + fe[k];
            if ( d_trws[k]>a ) a=d_trws[k];
        }
        a -= w;
        
        real maxd = -INF;
        for ( int k=0; k<nK; k++ ) {
            if ( a>d_trws[k] ) d_trws[k]=a;
            d_trws[k] += fee[k];
            if ( d_trws[k]>maxd ) maxd=d_trws[k];
        }
        
        for ( int k=0; k<nK; k++ ) {
            d_trws[k] -= maxd;
            qee[k]    += d_trws[k];
            fee[k]    -= d_trws[k];
            resid     += fabs(d_trws[k]);
        }
    }
    
    // backward pass
    for ( int e=nE-1; e>=0; e-- ) {
        real *qe = q + nK*E[1+e*2],
            *qee = q + nK*E[0+e*2],
            *fe = f + nK*(1+e*2),
            *fee = fe - nK;
        
        real a = -INF;
        for ( int k=0; k<nK; k++ ) {
            d_trws[k] = gamma*qe[k] + fe[k];
            if ( d_trws[k]>a ) a=d_trws[k];
        }
        a -= w;
        
        real maxd = -INF;
        for ( int k=0; k<nK; k++ ) {
            if ( a>d_trws[k] ) d_trws[k]=a;
            d_trws[k] += fee[k];
            if ( d_trws[k]>maxd ) maxd=d_trws[k];
        }
        
        for ( int k=0; k<nK; k++ ) {
            d_trws[k] -= maxd;
            qee[k]    += d_trws[k];
            fee[k]    -= d_trws[k];
            resid     += fabs(d_trws[k]);
        }
        
    }
    
    //delete[] d_trws;
    return resid/(nK*2*nE); // normalize residual to one label*pixel
}
*/


void colorVisionThread::init_onsize(int width_, int height_)    {
	width = width_;
	height = height_;

	q = new real[nK*width*height]; // allocate unary potentials
	E = grid_graph(width,height,&nE); // construct image graph
	
	f = new real[nK*2*nE]; // allocate and initialize messages
	for ( int i=0; i<nK*2*nE; i++ ) f[i]=0;
	
	K = new unsigned char[width*height];
	J = new unsigned[width*height];
}


double colorVisionThread::Determinant(double **a,int n) {
	int i,j,j1,j2 ;                    // general loop and matrix subscripts
    double det = 0 ;                   // init determinant
    double **m = NULL ;                // pointer to pointers to implement 2d
                                       // square array

    if (n < 1)    {   }                // error condition, should never get here

    else if (n == 1) {                 // should not get here
        det = a[0][0] ;
    }

    else if (n == 2)  {                // basic 2X2 sub-matrix determinate
                                       // definition. When n==2, this ends the
        det = a[0][0] * a[1][1] - a[1][0] * a[0][1] ;// the recursion series
    }


                                       // recursion continues, solve next sub-matrix
    else {                             // solve the next minor by building a
                                       // sub matrix
        det = 0 ;                      // initialize determinant of sub-matrix

                                           // for each column in sub-matrix
        for (j1 = 0 ; j1 < n ; j1++) {
                                           // get space for the pointer list
            m = (double **) malloc((n-1)* sizeof(double *)) ;

            for (i = 0 ; i < n-1 ; i++) {
                m[i] = (double *) malloc((n-1)* sizeof(double)) ;
            }
            //     i[0][1][2][3]  first malloc
            //  m -> +  +  +  +   space for 4 pointers
            //       |  |  |  |          j  second malloc
            //       |  |  |  +-> _ _ _ [0] pointers to
            //       |  |  +----> _ _ _ [1] and memory for
            //       |  +-------> _ a _ [2] 4 doubles
            //       +----------> _ _ _ [3]
            //
            //                   a[1][2]
            // build sub-matrix with minor elements excluded
            for (i = 1 ; i < n ; i++) {
                j2 = 0 ;               // start at first sum-matrix column position
                                       // loop to copy source matrix less one column
                for (j = 0 ; j < n ; j++) {
                    if (j == j1) continue ; // don't copy the minor column element

                    m[i-1][j2] = a[i][j] ;  // copy source element into new sub-matrix
                                            // i-1 because new sub-matrix is one row
                                            // (and column) smaller with excluded minors
                    j2++ ;                  // move to next sub-matrix column position
                }
            }

            det += pow(-1.0,1.0 + j1 + 1.0) * a[0][j1] * Determinant(m,n-1) ;
                                            // sum x raised to y power
                                            // recursively get determinant of next
                                            // sub-matrix which is now one
                                            // row & column smaller

            for (i = 0 ; i < n-1 ; i++) free(m[i]) ;// free the storage allocated to
                                            // to this minor's set of pointers
            free(m) ;                       // free the storage for the original
                                            // pointer to pointer
        }
    }
    return(det) ;
};
		
 void colorVisionThread::CoFactor(double **a,int n,double **b)
 {
	int i,j,ii,jj,i1,j1;
    double det;
    double **c;

    for (j = 0 ; j < n-1 ; j++) {
                                    
        c =  new double *[n-1] ;
			
        for (i = 0 ; i < n-1 ; i++) {
	        c[i]=new double[n-1];
		}
    }


    for (j = 0 ; j < n-1 ; j++) {			
        for (i = 0 ; i < n-1 ; i++) {
				c[i][j]=0;
		}
    }


    for (j=0;j<n;j++) {
        for(i=0;i<n;i++) {

            /* Form the adjoint a_ij */
            i1 = 0;
            for (ii=0;ii<n;ii++) {
                if (ii == i) {
                    continue;
                }
                j1 = 0;
                for (jj=0;jj<n;jj++) {
                    if (jj == j) {
                    continue;
                    }
                    c[i1][j1] = a[ii][jj];
                    j1++;
                }
                i1++;
            }

            /* Calculate the determinate */
            det = Determinant(c,n-1);

            /* Fill in the elements of the cofactor */
            b[i][j] = pow(-1.0,i+j+2.0) * det;
        }
    }
    for (i=0;i<n-1;i++) {
        free(c[i]);
    }
    free(c);
};
		
 void colorVisionThread::Transpose(double **a,int n)    {
	int i,j;
    double tmp;
    
    for (i=1;i<n;i++) {
        for (j=0;j<i;j++) {
            tmp = a[i][j];
            a[i][j] = a[j][i];
            a[j][i] = tmp;
        }
    }
};


 void colorVisionThread::RBF(double UUC1,double UUV1,double UUC2,double UUV2)   {
    ifstream we1("w11.txt");
	ifstream we2("w22.txt"); 
	//ifstream bi1("b1.txt");
	//ifstream bi2("b2.txt");

	printf("\n inputs %f  %f  %f  %f  \t ", UUC1,UUV1,UUC2,UUV2);

	int u,i,j;
	/*if(	bi1==NULL)
				{
					printf("Cant read the input file\n");
					exit(0);
				}
	for(u=0;u<84;u++)  // load caliberation matrix for camera 1
		    {
				bi1 >> s[u];
			    bias1[u]=s[u];  
	   		}

			if(	bi2==NULL)
				{
					printf("Cant read the input file\n");
					exit(0);
				}
	for(u=0;u<3;u++)  // load caliberation matrix for camera 1
		    {
				bi2 >> s[u];
			    bias2[u]=s[u];  
	   		}*/ 

    if(	we1==NULL)  {
        printf("Cant read the input file\n");
        //exit(0);
    }
    
    i=0;
	for(u=0;u<77;u++)   {  // load caliberation matrix for camera 1
        for(j=0;j<2;j++) { // load caliberation matrix for camera 1
            we1 >> s[i];
            weight1[u][j]=s[i];  
            i=i+1;
        }
    }
    
    if(	we2==NULL)  {
        printf("Cant read the input file\n");
        //exit(0);
    }
    
    i=0;
	for(u=0;u<2;u++) { // load caliberation matrix for camera 1
	    for(j=0;j<77;j++){  // load caliberation matrix for camera 1
	    	    
		    we2 >> s[i];
		    weight2[u][j]=s[i];  
			i=i+1;
	    }
	}

	printf("\n BIAS AND WEIGHT %f  %f  %f %f \t ", weight1[76][0],weight1[76][1],weight2[0][76],weight2[1][76]);
	

	double mid[77], midrad[77], pintee, tempj;
    
    for(u=0;u<77;u++) {  // load caliberation matrix for camera 1
        mid[u]=(sqrt(pow((171-weight1[u][0]),2)+ pow((201-weight1[u][1]),2)))*0.8326;
        pintee=pow(mid[u],2);
        midrad[u]=(exp((-1*pintee)/50)); 
		printf("\n Midrad %f  %f \t ", pintee,midrad[u]);
    }
    
    
	for(u=0;u<2;u++) {
        tempj=0;
        for(j=0;j<77;j++) {
            tempj=tempj+weight2[u][j]*midrad[j];
        }
        FinXpos[u]=tempj;
    }
    
	printf("\n FINALPOSITION %f  %f  %f  \t ", FinXpos[0],FinXpos[1]);
    
    FinXpos[0]=FinXpos[0]+45-226;
    FinXpos[1]=FinXpos[1]-246;
    //FinXpos[2]=FinXpos[2]+5;
    printf("\n FINALPOSITION %f  %f  %f  \t ", FinXpos[0],FinXpos[1]);
    //Sleep(45000);		
}
     


 void colorVisionThread::ConvImg(double UUC1,double UUV1,double UUC2,double UUV2)   {
	
	ofstream write3("Sconf.txt");
	ofstream write4("SconfT.txt");
	ofstream write5("Amat.txt");
	ofstream write6("CartCo.txt");
	ofstream write("nugam.txt");
	ofstream write1("cofac.txt");
	ofstream write2("inverse.txt");
	ifstream Weight3("w3.txt");
	ifstream calib1("calib1.txt");
	ifstream calib2("calib2.txt");
	

	int n=3,i,u,j1,j;
	double inter1,inter2,inter3;
	double inv[3][3];

	double u1,v1,u2,v2;

    //	    point[0][0]=94; point[0][1]=104; point[0][2]=78; point[0][3]=104;
    //  point[1][0]=139; point[1][1]=91;point[1][2]=110; point[1][3]=83;
    // point[3][0]=46; point[3][1]=106;point[3][2]=47; point[3][3]=117;

	u1=UUC1;
    v1=UUV1;
	u2=UUC2;
    v2=UUV2;


    //====read calibration martixes of camera 1 and camera 2 ==============
    
	if(	calib1==NULL)
        {
            printf("Cant read the input file\n");
            //exit(0);
        }
	for(u=0;u<11;u++)  // load caliberation matrix for camera 1
        {
            calib1 >> s[u];
			//fscanf(calib1,"%f",&s[u]);
	        //printf("\n\ndata testing  %f",s[u]);
	        l1[u]=s[u];  
	   	}
    
    if(	calib2==NULL)
        {
            printf("Cant read the input file\n");
            //exit(0);
        }
	for(u=0;u<11;u++)   // load caliberation matrix for camera 2
        {
            calib2 >> s[u];
	        //printf("\n\ndata testing  %f",s[u]);
	        l2[u]=s[u];  
	   	}
    //=============================RHS Ycal====================================
    Y1=u1-l1[3];
    Y2=v1-l1[7];
    Y3=u2-l2[3];
    Y4=v2-l2[7];
    double a1=l1[8],a2=l1[9],a3=l1[10],b1=l2[8],b2=l2[9],b3=l2[10];
    
    //=============================Get LHS Matrix using U,V and Calbration Matrixes ====================================
    
    Sconf[0][0]=l1[0]-u1*a1;
    Sconf[0][1]=l1[1]-u1*a2;
    Sconf[0][2]=l1[2]-u1*a3;
    Sconf[1][0]=l1[4]-v1*a1;
    Sconf[1][1]=l1[5]-v1*a2;
    Sconf[1][2]=l1[6]-v1*a3;
    Sconf[2][0]=l2[0]-u2*b1;
    Sconf[2][1]=l2[1]-u2*b2;
    Sconf[2][2]=l2[2]-u2*b3;
    Sconf[3][0]=l2[4]-v2*b1;
    Sconf[3][1]=l2[5]-v2*b2;
    Sconf[3][2]=l2[6]-v2*b3;
    SconfT[0][0]=l1[0]-u1*a1;
    SconfT[0][1]=l1[4]-v1*a1;
    SconfT[0][2]=l2[0]-u2*b1;
    SconfT[0][3]=l2[4]-v2*b1;
    SconfT[1][0]=l1[1]-u1*a2;
    SconfT[1][1]=l1[5]-v1*a2;
    SconfT[1][2]=l2[1]-u2*b2;
    SconfT[1][3]=l2[5]-v2*b2;
    SconfT[2][0]=l1[2]-u1*a3;
    SconfT[2][1]=l1[6]-v1*a3;
    SconfT[2][2]=l2[2]-u2*b3;
    SconfT[2][3]=l2[6]-v2*b3;
    
    int count;
    for (count = 0 ; count < 4 ; count++) 
        {			
            write3 << Sconf[count][0]<< Sconf[count][1] << Sconf[count][2]<< endl ;
        } 
    
    for (count = 0 ; count < 3 ; count++) 
        {			
            write4 << SconfT[count][0]<< SconfT[count][1] << SconfT[count][2]<< SconfT[count][3]<< endl ;      
        } 
    
    
    
    // printf("\n\n %f %f", Sconf[3][0],SconfT[0][3]);
    
    int g, h, q;
    double sum, C[3][3];
    
    for (g = 0; g < 3; g++) {
        for (h = 0; h < 3; h++) {
            sum = 0;
            for (q = 0; q < 4; q++) {
                rw=SconfT[g][q];
                rx=Sconf[q][h];
                sum += rw * rx;
            }
            C[g][h] = sum;
        }
    }
    
    for (count = 0 ; count < 3 ; count++) 
        {			
            write5 << C[count][0]<< C[count][1] << C[count][2]<< endl ;      
        } 
    
    //===============================================================
    
    
    double **p;
    double **b;
    
    for (j1 = 0 ; j1 < n ; j1++) 
        {
            
            p =  new double *[n] ;
			
            for (i = 0 ; i < n ; i++)
                {
                    p[i]=new double[n];
                }
        }
    
    
    for (j = 0 ; j < n ; j++) 
        {			
            for (i = 0 ; i < n ; i++)
                {
                    p[i][j]=C[i][j];
                }
        }
    
    double dete = Determinant(p,n);
    //printf("\n\n %f", dete);
    
    for (j1 = 0 ; j1 < n ; j1++) 
        {
            
            b =  new double *[n] ;
			
            for (i = 0 ; i < n ; i++)
                {
                    b[i]=new double[n];
                }
        }
    
    
    for (j = 0 ; j < n ; j++) 
        {			
            for (i = 0 ; i < n ; i++)
                {
                    b[i][j]=0;
                }
        }
    
    CoFactor(p,n,b);
    for (j = 0 ; j < n ; j++) 
        {			
            write1 << b[j][0]<< b[j][1]<< b[j][2]<< endl ;        
        } 
    Transpose(b,n);
    
    for (j = 0 ; j < n ; j++) 
        {
            for (i = 0 ; i < n ; i++)
                {
                    inv[j][i]=(1/dete)*b[j][i];	
                }
        }
    for (j = 0 ; j < n ; j++) 
        {			
            write2 << inv[j][0]<< inv[j][1]<< inv[j][2]<< endl ;         
        } 
    
    //======================= Inverse(AA')============================================
    inter1=SconfT[0][0]*Y1 + SconfT[0][1]*Y2 +SconfT[0][2]*Y3+SconfT[0][3]*Y4;
    inter2=SconfT[1][0]*Y1 + SconfT[1][1]*Y2 +SconfT[1][2]*Y3+SconfT[1][3]*Y4;
    inter3=SconfT[2][0]*Y1 + SconfT[2][1]*Y2 +SconfT[2][2]*Y3+SconfT[2][3]*Y4;
    
    ImagX=inv[0][0]*inter1+inv[0][1]*inter2+inv[0][2]*inter3;
    ImagY=inv[1][0]*inter1+inv[1][1]*inter2+inv[1][2]*inter3;
    ImagZ=inv[2][0]*inter1+inv[2][1]*inter2+inv[2][2]*inter3;
    printf("\n  %f  %f   %f \t ",ImagX,ImagY,ImagZ);
    /*for (i=0;i<n-1;i++) {
      free(p[i]);
      free(b[i]);
      }
      free(p);
      free(b);*/
    ////Sleep(5000);
 };
		
 
