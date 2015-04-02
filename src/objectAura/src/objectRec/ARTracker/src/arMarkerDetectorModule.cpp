/*
 * Copyright (C) 2008 Alexandre Bernardino, Manuel LopesVislab, IST/ISR.
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */



#include <iostream>
#include "arMarkerDetectorModule.h"

#include <highgui.h>

ARMarkerDetectorModule::ARMarkerDetectorModule()
{

}


ARMarkerDetectorModule::~ARMarkerDetectorModule()
{

}

bool ARMarkerDetectorModule::open(Searchable& config)
{
    if (config.check("help","if present, display usage message")) {
        printf("Call with --file configFile.ini\n");

        return false;
    }

    ConstString input_port = config.check("inputPort", Value("img:i"), "Camera intrinsics definition file.").asString().c_str();
    
    
    ConstString output_port = config.check("outputPort", Value("data:o"), "Camera intrinsics definition file.").asString().c_str();
    
    ConstString conf_port = config.check("confPort", Value("conf"), "Camera intrinsics definition file.").asString().c_str();

    ConstString display_name = config.check("displayName", Value("ARTrackerUH"), "Camera intrinsics definition file.").asString().c_str();
    
    ConstString camera_file = config.check("cameraparameters", Value("camera_para.dat"), "Camera intrinsics definition file.").asString().c_str();

    ConstString objects_file = config.check("objects", Value("object_data.txt"), "Markers definition file.").asString().c_str();

	tracker.loadCameraParameters( camera_file.c_str(), 640, 480);

	//tracker.loadObjectData( objects_file.c_str() );

	int numberofobjects = config.check("numberofpatterns", 0, "number of patterns").asInt();
	tracker.initobjectdata( numberofobjects );

	ConstString directory = config.check("directory", "/", "directory where patterns are located").asString();

	for(int cnt=0;cnt<numberofobjects;cnt++)
	{
		char name[256];
		char filename[256];
		char pattern[16];

		sprintf( pattern,"pattern%d",cnt+1);

		Bottle& K = config.findGroup( pattern, "object");
		cout << K.toString() << endl;

		strcpy( name, (const char*)K.get(1).asString());

		
		strcpy( filename, (const char*)directory);
		strcat( filename, (const char*)K.get(2).asString()  );
		

		double size = K.get(3).asDouble();
		printf("reading conf bootle s0%s s1%s s2%s s3%g\n",
		       (const char*)K.get(0).asString(),
		       (const char*)K.get(1).asString(),
		       (const char*)K.get(2).asString(),
		       size);

		tracker.addobjectdata( cnt, name, filename, size );
	}

	//tracker.loadObjectData( config );

	//check visualization

	int visualize = config.check("Visualization", 0, "Show the tracker image").asInt();
	
	if(visualize)
		tracker.openVisualization(display_name);

    //load in the object data - trained markers and associated bitmap files 



    _imgPort.open(getName(input_port));
    _configPort.open(getName(conf_port));
    _outPort.open(getName(output_port));

    attach(_configPort, true);
//    imageOut.open("/testOut");

    return true;

}



bool ARMarkerDetectorModule::close()
{
    _imgPort.close();
    _configPort.close();
    _outPort.close();

    return true;
}



bool ARMarkerDetectorModule::interruptModule()
{

    _imgPort.interrupt();
    _configPort.interrupt();
    _outPort.interrupt();

    return true;
}



bool ARMarkerDetectorModule::updateModule()
{
    int i, j;

    ImageOf<PixelRgb> *yrpImgIn;   


    ImageOf<PixelRgb>* yrpCopyImgIn;   
    yrpImgIn = _imgPort.read();
    yrpCopyImgIn= _imgPort.read();

	if (yrpImgIn == NULL)   // this is the case if module is requested to quit while waiting for image
        return true;

	double auxtime, aux;

	auxtime = yarp::os::Time::now();

	aux = auxtime - timelastround;

	printf( "%g ms %g fps \n", aux*1000, 1/aux );

	timelastround = auxtime;



	IplImage *iplimg = (IplImage*)yrpImgIn->getIplImage();

	tracker.detectMarkers( iplimg );

	cvWaitKey( 5 );

	Bottle &b = _outPort.prepare();
	b.clear();
	bool detected = false;

	for(int cnt=0; cnt<tracker.nobjects(); cnt++)
	{
		if( tracker.isObjectDetected(cnt) )
		{    
				double T[3][4];
				tracker.getMatrix(cnt,T);
				char name[256];
				tracker.getObjectName(cnt, name);
				printf("%s\nT\n",name);
				b.addString( name );
				b.addString( "T");
				for(i = 0; i < 3; i++ )
				{
					b.addDouble( T[i][0]);
					b.addDouble( T[i][1]);
					b.addDouble( T[i][2]);
					b.addDouble( T[i][3]);
					char str[64];
					printf("%g %g %g %g\n",T[i][0],T[i][1],T[i][2],T[i][3]);
				}

				int cx, cy;
				tracker.getCenter( cnt, &cx, &cy);
				b.addString("imgcoord");
				b.addDouble( cx);
				b.addDouble( cy);
                               
                                
				b.addString("size");
				b.addDouble( tracker.getSize(cnt));
				b.addString("range");
				b.addDouble(sqrt(T[0][3]*T[0][3] + T[1][3]*T[1][3] + T[2][3]*T[2][3]));
				
				b.addString("imgcoordnorm");
				b.addDouble( 2.0 * cx / (double)iplimg->width  - 1.0 );
				b.addDouble( - (2.0 * cy / (double)iplimg->height - 1.0) );

				printf("img coord\n %d %d\t size %d\n", cx, cy,  tracker.getSize(cnt));
				printf("range: %f\n",sqrt(T[0][3]*T[0][3] + T[1][3]*T[1][3] + T[2][3]*T[2][3]));
				printf("imgcoordnorm\n %1.3g %1.3g\n\n",
						2.0 * cx / (double)iplimg->width  - 1.0 ,
						- (2.0 * cy / (double)iplimg->height - 1.0));
                                // edges
                                int x0,y0,x1,y1,x2,y2,x3,y3;
                                tracker.getEdge(cnt,0,&x0,&y0);
                                tracker.getEdge(cnt,1,&x1,&y1);
                                tracker.getEdge(cnt,2,&x2,&y2);
                                tracker.getEdge(cnt,3,&x3,&y3);
                                printf("Edges:\n %d %d - %d %d - %d %d - %d %d\n\n",x0,y0,x1,y1,x2,y2,x3,y3);
                                
                                // compute the simple colour using  chroma comparison - yields Red, green or Blue only!
                                
                                float chromaTable[3];   // in this program we simply normalise for the 3 colours
     
                                for (int i=0;i<3;i++) 
                                {
                                   chromaTable[i] = 0;
                                }
                                
                                int l = sqrt((double)tracker.getSize(cnt))/2.0;        // this gives a slightly wider area than the ART central image - allows the bounding ART square to be coloured   

                                int X1,Y1,X2,Y2;
                                
                                X1 = cx - l;
                                Y1 = cy - l;
                                X2   = cx + l;
                                Y2   = cy + l;
                                             
                                
                     //           printf("x1 y1: %d %d   x2 y2: %d %d\n",X1,Y1,X2,Y2);
                    //            printf("=====================================\n");                                
                                for (int x = X1; x < X2; x++) 
                                {
                                   for (int y = Y1; y < Y2;  y++) 
                                   {
                                      PixelRgb& pixel = yrpCopyImgIn->pixel(x,y);
                                      
                                    // printf("r: %d g: %d   b:  %d  x: %d  y: %d\n",pixel.r,pixel.g,pixel.b,x,y);
         
                                      chromaTable[0] += (float)pixel.r / ((float)pixel.r  + (float)pixel.g  + (float)pixel.b);
                                      chromaTable[1] += (float)pixel.g / ((float)pixel.r  + (float)pixel.g  + (float)pixel.b);
                                      chromaTable[2] += (float)pixel.b / ((float)pixel.r  + (float)pixel.g  + (float)pixel.b);                           

                                   }
                                }
                      //          for (int i=0; i < 3; i++)
                      //          {
                      //             printf("%f ",chromaTable[i]); 
                      //          }
                      //          printf( " \n"); 
                      //          printf("=====================================\n");  
                                if ((chromaTable[0] > chromaTable[1]) && (chromaTable[0] > chromaTable[2]))
                                {
                                   printf("CHROMA: Red\n");
                                   b.addString("red");
                                   b.addInt(4);    // the numbers represent the position of red in a 3 bit colour space
                                                   // in case we expand the selection later
                                }
      
                                if ((chromaTable[1] > chromaTable[0]) && (chromaTable[1] > chromaTable[2]))
                                {
                                   printf("CHROMA: Green\n");
                                   b.addString("green");
                                   b.addInt(2);    // the numbers represent the position of green in a 3 bit colour space
                                                   // in case we expand the selection later
                                }      
      
                                if ((chromaTable[2] > chromaTable[0]) && (chromaTable[2] > chromaTable[1]))
                                {
                                   printf("CHROMA: Blue\n");
                                   b.addString("blue");
                                   b.addInt(1);    // the numbers represent the position of blue in 3 bit colour space
                                                   // in case we expand the selection later
                                }  
			Bottle time;
			_imgPort.getEnvelope(time);
			b.addDouble(time.get(0).asDouble());                                       
                                
			detected = true;
		}
	}
	
	if (detected) {
		 
	}

	_outPort.write();

//        ImageOf<PixelRgb> *imageOutput;
        
//        imageOutput = yrpCopyImgIn;
        
//        imageOut.prepare() = *imageOutput;
      
//        imageOut.write();	
         
    return true;
}
