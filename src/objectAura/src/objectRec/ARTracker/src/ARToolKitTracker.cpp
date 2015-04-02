

#include "ARToolKitTracker.h"

#ifdef _WIN32
#include <windows.h>
#endif
#include <stdio.h>
#ifndef __APPLE__
#include <malloc.h>
#endif
#include <stdlib.h>


#include </home/bishop/iCubDev/ARToolKit/include/AR/ar.h>
#include </home/bishop/iCubDev/ARToolKit/include/AR/param.h>


ARToolKitTracker::ARToolKitTracker()
{

	_visualize = false;
	_font = new CvFont();
	cvInitFont( _font, CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0, 1, 8 );
	_thresh = 120;
	_frame = 0;
	_sizex = -1;
	_sizey = -1;
}



ARToolKitTracker::~ARToolKitTracker()
{

	if(_visualize)
		cvDestroyWindow( visualisationWindowName );

}


bool ARToolKitTracker::open()
{

	return true;

}



bool ARToolKitTracker::close()
{

	return true;

}





bool ARToolKitTracker::loadCameraParameters(const char* filename, int xsize, int ysize)

{

    

    /* set the initial camera parameters */

    if( arParamLoad( filename, 1, &_oparam) < 0 ) {

       cout << "Could not open file " << filename << endl;

        return false;

    }



	_sizex = xsize;

	_sizey = ysize;



	createAuxImage();    



	return true;

}



int ARToolKitTracker::createAuxImage(void)

{

	if(_frame)
		cvReleaseImage( &_frame );
	
	CvSize sz = cvSize( _sizex, _sizey);

		
	//WINDOWS
#ifdef WIN32
	_frame = cvCreateImage(sz, 8, 4 );
#else
	//LINUX
	_frame = cvCreateImage(sz, 8, 3 );
#endif

	arParamChangeSize( &_oparam, _sizex, _sizey, &_cparam );
	arInitCparam( &_cparam );

    	printf("*** Camera Parameter ***\n");
    	arParamDisp( &_cparam );

	return 0;
}



bool ARToolKitTracker::loadObjectData(const char* filename)

{

 /* load in the object data - trained markers and associated bitmap files */

    if( (_object=read_objectdata( filename,&_object_num)) == NULL )     {

        cout << "Could not open object file " << filename << endl;

        return false;

    }

	return true;

}




static char *get_buff( char *buf, int n, FILE *fp );

int ARToolKitTracker::initobjectdata( int numberofobjects )
{
	_object = (ObjectData_T *)malloc( sizeof(ObjectData_T) * numberofobjects );

	_object_num = numberofobjects;

	return _object == NULL;
}

int ARToolKitTracker::addobjectdata( const int objnumber, const char *name, const char *filename, const double size )
{

		strcpy ( _object[objnumber].name, name );

		_object[objnumber].id = arLoadPatt( filename );

		_object[objnumber].visible = 0;

		_object[objnumber].marker_width = size;

		int ret = arActivatePatt( _object[objnumber].id );
		if(ret==1) 
		  printf("pattern activated\n");

		return 0;
}

ObjectData_T *ARToolKitTracker::read_objectdata( const char *name, int *objectnum )
{
    FILE          *fp;
    ObjectData_T  *object;
    char           buf[256], buf1[256];
    int            i;

    if( (fp=fopen(name, "r")) == NULL ) return(0);

    get_buff(buf, 256, fp);
    if( sscanf(buf, "%d", objectnum) != 1 ) {fclose(fp); return(0);}

    object = (ObjectData_T *)malloc( sizeof(ObjectData_T) * *objectnum );
    if( object == NULL ) return(0);

    for( i = 0; i < *objectnum; i++ ) {
        get_buff(buf, 256, fp);
        if( sscanf(buf, "%s", object[i].name) != 1 ) {
          fclose(fp); free(object); return(0);}

        get_buff(buf, 256, fp);
        if( sscanf(buf, "%s", buf1) != 1 ) {
          fclose(fp); free(object); return(0);}
        
        if( (object[i].id = arLoadPatt(buf1)) < 0 )
            {fclose(fp); free(object); return(0);}

        object[i].visible = 0;

        get_buff(buf, 256, fp);
        if( sscanf(buf, "%lf", &object[i].marker_width) != 1 ) {
            fclose(fp); free(object); return(0);
        }

        printf("No.%d: %20s\n", i+1, &(object[i].name[0]) );
    }

    fclose(fp);

    return( object );
}

char *ARToolKitTracker::get_buff( char *buf, int n, FILE *fp )
{
    char *ret;

    for(;;) {
        ret = fgets( buf, n, fp );
        if( ret == NULL ) return(NULL);
        if( buf[0] != '\n' && buf[0] != '#' ) return(ret);
    }
}



int ARToolKitTracker::openVisualization(ConstString display_name)

{



	cvNamedWindow( display_name, CV_WINDOW_AUTOSIZE );

	_visualize = true;
	
	visualisationWindowName = display_name;




	return 0;

}



bool ARToolKitTracker::detectMarkers( IplImage *iplimg)
{
ARUint8         *dataPtr;
int             marker_num;
int i, j, k;
double _object_center[] = {0 , 0};



	if ( (iplimg->width != _sizex) || (iplimg->height != _sizey) )
	{
		_sizex = iplimg->width;
		_sizey = iplimg->height;

		createAuxImage();
	}

    //MUST CONVERT TO BGRA

#ifdef WIN32
	//WINDOWS
    cvCvtColor(iplimg,_frame,CV_RGB2BGRA);
#else
	//LINUX
	//cvCopy(iplimg,_frame,NULL);
        cvCvtColor(iplimg,_frame,CV_RGB2BGR);
#endif

	dataPtr = (ARUint8*)_frame->imageData;

    /* detect the markers in the video frame */
        
    printf("trying to detect markers thres %d\n",_thresh);    
    if( arDetectMarker(dataPtr, _thresh, &_marker_info, &marker_num) < 0 ) 
    {
        cout << "Unknown error in arDetectMarker" << endl;    
        return false;
    }
    
    bool tryAdaptive = false; 
    
    if(marker_num==0)      // try adaptive threshold if default threshold not found anything
    { 
       tryAdaptive = true; 
    }
    else
    {
       for (int n=0;n<marker_num;n++)
       {
          if (_marker_info[n].id == -1)  // got a marker but don't know what it is
          {
             tryAdaptive = true; 
          }
       }       
       
    }
    
    if(tryAdaptive)      // try adaptive threshold if default threshold not found anything
    {
 //       printf("Doing adaptive threshold\n");
        
	for(int ttt=10;ttt<250;ttt=ttt+10)
	{
	  
//         printf("Trying to detect at threshold: %d\n",ttt);
         
         arDetectMarker(dataPtr, ttt, &_marker_info, &marker_num);
         
	  if(marker_num>0)
          {
  //            printf("Found marker at adaptive threshold: %d\n",ttt);
              for (int n=0;n<marker_num;n++)
              {
                 if (_marker_info[n].id != -1)  // we know what at least what one of them are
                 {
                    goto nextbit;
                 }
              }
              
          }
                          
        }
    }
    
nextbit:    
    for (int n=0;n<marker_num;n++)
    {
       printf("Detected n: %d  cf: %f id: %d\n",n,_marker_info[n].cf,_marker_info[n].id); 
    }
    
    /* check for object visibility */

    for( i = 0; i < _object_num; i++ ) 
    {   
       // cout << "Checking object " << i << endl;
        _object[i].visible = 0;
        k = -1;
        for( j = 0; j < marker_num; j++ ) 
        {
	   if( _object[i].id == _marker_info[j].id ) 
           {
              if( k == -1 ) 
              {
                 k = j;
              }
              else 
              {
                  if( _marker_info[k].cf < _marker_info[j].cf ) 
                  {   
                     k = j;
                  }
              }
           }
        }
//k=i;
	if( k == -1 ) continue;

	if(_visualize)
	{
		cvLine( iplimg, cvPoint( (int)_marker_info[k].vertex[0][0], (int)_marker_info[k].vertex[0][1] ),
                                cvPoint( (int)_marker_info[k].vertex[1][0], (int)_marker_info[k].vertex[1][1] ),
							CV_RGB( 255, 0, 0 ), 2, 8, 0 );

		cvLine( iplimg, cvPoint( (int)_marker_info[k].vertex[1][0], (int)_marker_info[k].vertex[1][1] ),
							cvPoint( (int)_marker_info[k].vertex[2][0], (int)_marker_info[k].vertex[2][1] ),
							CV_RGB( 255, 0, 0 ), 2, 8, 0 );

		cvLine( iplimg, cvPoint( (int)_marker_info[k].vertex[2][0], (int)_marker_info[k].vertex[2][1] ),
							cvPoint( (int)_marker_info[k].vertex[3][0], (int)_marker_info[k].vertex[3][1] ),
							CV_RGB( 255, 0, 0 ), 2, 8, 0 );

                cvLine( iplimg, cvPoint( (int)_marker_info[k].vertex[3][0], (int)_marker_info[k].vertex[3][1] ),
							cvPoint( (int)_marker_info[k].vertex[0][0], (int)_marker_info[k].vertex[0][1] ),
							CV_RGB( 255, 0, 0 ), 2, 8, 0 );

		int radius = (int) (sqrt((double)_marker_info[k].area)/1.77);
                

		cvCircle( iplimg, cvPoint( (int)_marker_info[k].pos[0], (int)_marker_info[k].pos[1] ),
						radius, CV_RGB( 255, 0, 0 ), 2, 8, 0 );

                // this outlines the area being used for colour detetction it's slightly wider than the pattern areas
                
                
                int l = sqrt((double)_marker_info[k].area)/2.0;  // this gives a slightly wider area than the ART central image - allows the bounding ART square to be coloured   
                
                
                int X1, Y1, X2, Y2;
                                
                X1 = (int)_marker_info[k].pos[0] - l;
                Y1 = (int)_marker_info[k].pos[1] - l;
                X2   = (int)_marker_info[k].pos[0] + l;
                Y2   = (int)_marker_info[k].pos[1] + l;
               
               
               cvLine( iplimg, cvPoint( X1, Y1) ,
                               cvPoint( X2, Y1),  CV_RGB( 0, 255, 0 ), 2, 8, 0 );    
                    
               cvLine( iplimg, cvPoint( X2, Y1) ,
                               cvPoint( X2, Y2),  CV_RGB( 0, 255, 0 ), 2, 8, 0 );
                   
               cvLine( iplimg, cvPoint( X1, Y1) ,
                               cvPoint( X1, Y2),  CV_RGB( 0, 255, 0 ), 2, 8, 0 );   
                                         
               cvLine( iplimg, cvPoint( X1, Y2) ,
                              cvPoint( X2, Y2),  CV_RGB( 0, 255, 0 ), 2, 8, 0 );   
               
               printf("x1 y1: %d %d   x2 y2: %d %d\n",X1,Y1,X2,Y2);
                          
         	//	cvPutText( iplimg, _object[i].name, cvPoint( (int)_marker_info[k].pos[0], (int)_marker_info[k].pos[1] ), _font,  CV_RGB( 255, 0, 0 ) );
	}

        // get the transformation between the marker and the real camera
        arGetTransMat(&_marker_info[k], _object_center, _object[i].marker_width, _object[i].trans);
        printf("marker size %g\n", _object[i].marker_width);

		//the markers are not in the same order as the objects
	_object[i].markerindex = k;

        _object[i].visible = 1;
    }



    if(_visualize)
    {
    //   cvShowImage( visualisationWindowName, _frame );        
                cvCvtColor(iplimg,iplimg,CV_RGB2BGR);  // needed as opencv needs bgr
		cvShowImage( visualisationWindowName, iplimg );
		cvWaitKey(1);
    }



	return true;

}





bool ARToolKitTracker::isObjectDetected(int objnum)
{

	return _object[objnum].visible;

}



int ARToolKitTracker::getMatrix(int objnum, double T[3][4])
{

	copytrans( _object[objnum].trans, T);

	return 0;
}



void ARToolKitTracker::copytrans(double src[3][4], double dst[3][4])

{

    int i, j;

    for(i=0; i < 3; i++)

        for(j=0; j<4; j++)

            dst[i][j] = src[i][j];

    return;

}



int ARToolKitTracker::getCenter(int objnum, int* x, int* y)
{

	*x = (int)_marker_info[_object[objnum].markerindex].pos[0];

	*y = (int)_marker_info[_object[objnum].markerindex].pos[1];

	return 0;
}

int ARToolKitTracker::getEdge(int objnum, int edge, int* x, int* y)
{
   *x = (int)_marker_info[_object[objnum].markerindex].vertex[edge][0];

   *y = (int)_marker_info[_object[objnum].markerindex].vertex[edge][1];

   return 0;
}
 


int ARToolKitTracker::getSize(int objnum)
{

	return _marker_info[_object[objnum].markerindex].area;

}



int ARToolKitTracker::nobjects()
{

	return _object_num;

}



int ARToolKitTracker::getObjectName(int objnum, char *name)
{

	strcpy( name,_object[objnum].name);

	return 0;
}

