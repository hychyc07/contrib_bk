#include <string.h>
#include<time.h>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <windows.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include "vision.h"

 //
 using namespace std;

 using namespace yarp::os;
 using namespace yarp::sig;
 using namespace yarp::sig::draw;
 using namespace yarp::sig::file;
 using namespace yarp;

 VisionSystem::VisionSystem()
 {
	iLocX=1;
	iLocY=1;
	iLocXL=1;
	iLocYL=1;
 };
		
 VisionSystem::~VisionSystem()
 {
	
 };

 double* VisionSystem::Vision(int ObjectType)
 {
	initHead(-32);
	Sleep(2000);
	if(ObjectType==0)
	{
		GCamSceneR(14);
		GCamSceneL(14);
		
		printf("\n Salient Points  %d \t  %d \t %d \t  %d \t",iLocX,iLocY,iLocXL,iLocYL);
        ConvImg(iLocX,iLocY,iLocXL,iLocYL);
	    printf("\n  %f  %f   %f \t ",ImagX,ImagY,ImagZ);
	}
	else if(ObjectType==1)
	{
		GCamSceneRred(14);
		GCamSceneLred(14);
		
		printf("\n Salient Points  %d \t  %d \t %d \t  %d \t",iLocX,iLocY,iLocXL,iLocYL);
        ConvImg(iLocX,iLocY,iLocXL,iLocYL);
	    printf("\n  %f  %f   %f \t ",ImagX,ImagY,ImagZ);
	}
	double *tdLoc;
	double tdGpmp[3];
	tdGpmp[0] = ImagX;
	tdGpmp[1] = ImagY;
	tdGpmp[2] = ImagZ;
	tdLoc=tdGpmp;
	return tdLoc;
 };


 double VisionSystem::Determinant(double **a,int n)
 {
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

            for (i = 0 ; i < n-1 ; i++)
                m[i] = (double *) malloc((n-1)* sizeof(double)) ;
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
		
 void VisionSystem::CoFactor(double **a,int n,double **b)
 {
	int i,j,ii,jj,i1,j1;
   double det;
   double **c;

for (j = 0 ; j < n-1 ; j++) 
{
                                    
            c =  new double *[n-1] ;
			
            for (i = 0 ; i < n-1 ; i++)
			{
				c[i]=new double[n-1];
			}
}


for (j = 0 ; j < n-1 ; j++) 
{			
            for (i = 0 ; i < n-1 ; i++)
			{
				c[i][j]=0;
			}
}


   for (j=0;j<n;j++) {
      for (i=0;i<n;i++) {

         /* Form the adjoint a_ij */
         i1 = 0;
         for (ii=0;ii<n;ii++) {
            if (ii == i)
               continue;
            j1 = 0;
            for (jj=0;jj<n;jj++) {
               if (jj == j)
                  continue;
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
   for (i=0;i<n-1;i++)
      free(c[i]);
   free(c);
 };
		
 void VisionSystem::Transpose(double **a,int n)
 {
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
		
 void VisionSystem::ConvImg(double UUC1,double UUV1,double UUC2,double UUV2)
 {
	
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
					exit(0);
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
					exit(0);
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
////Sleep(5000);
 };
		
 void VisionSystem::GCamSceneR(int ActCamsR)
 {
	// Create a port you will use to send commands to the bot
     BufferedPort<ImageOf<PixelRgb> > imagePortR;
     
		//BufferedPort<Bottle> out1;
        // Name the ports
	 int averager=0;       
 imagePortR.open("/v1/imagePortR");
 Network::connect("/icub/cam/right","/v1/imagePortR");
 Network::connect("/v1/imagePortR","/v/r"); 
 int xMean = 0;
         int yMean = 0;
 // read an image from the port
     ImageOf<PixelRgb> *imgR = imagePortR.read();
     int ctR=0; 
       if (imgR!=NULL) { // check we actually got something
         printf("We got an image of size %dx%d\n", imgR->width(), imgR->height());
         
         int ct = 0;
         for (int x=0; x<imgR->width(); x++) {
           for (int y=0; y<imgR->height(); y++) {
             PixelRgb& pixel = imgR->pixel(x,y);
             // very simple test for blueishness
             // make sure blue level exceeds red and green by a factor of 2
			 //  if (pixel.r>pixel.b*1.2+10 && pixel.r>pixel.g*1.2+10)
			 //  if (pixel.b>pixel.r*1.2+10 && pixel.b>pixel.g*1.2+10)
             if (pixel.g>pixel.b*1.2+10 && pixel.g>pixel.r*1.2+10) {
              // there's a blueish pixel at (x,y)!
              // let's find the average location of these pixels
              xMean += x;
              yMean += y;
              ct++;
             }
           }
         }
         if (ct>0) {
           xMean /= ct;
           yMean /= ct;
         }
         if (ct>(imgR->width()/20)*(imgR->height()/20)) {
           printf("Best guess at blue target: %g %g\n", xMean, yMean);
         }
      }
	 
   /*   if (imgR==NULL)
	 {
       printf ("Image is NULL!!!\n");
     }
      for (int y=0; y<imgR->height(); y++) 
	  {
           for  (int x=0; x<imgR->width(); x++)
		   {
                 PixelRgb& pixel = imgR->pixel(x,y);
					 // very simple test for greenishness
					 // make sure green level exceeds red and blue by a factor of 2
              if ( (pixel.g<=2) && (pixel.r>=145)&& (pixel.r<=155)&& (pixel.b<=2)) 
                 {
                   averager=averager+1;
					 // there's a greenish pixel at (x,y)!
				   iLocX=iLocX+x;
				   iLocY=iLocY+y;
				 }
   }
 }
                    
	               iLocX=iLocX/averager;
				   iLocY=iLocY/averager; */
	                iLocX= xMean;
                    iLocY= yMean;
				    printf("\n\n centroid....%d %d ", iLocX,iLocY); 
                   PixelRgb blue(255,255,255);
				   addCircle(*imgR,blue,iLocX,iLocY,5);
				   ctR = (ctR+10)%imgR->width();	    
				   imagePortR.prepare() = *imgR;
                  
				   imagePortR.write();
				   Sleep(3000);
//                   ImageOf<PixelRgb> *img = imagePort.write();
     // write the image to a file
     //write(*imgR, "img5R.ppm");
	 //Sleep(10000);
 };
		
 void VisionSystem::GCamSceneL(int ActCamsL)
 {
	// Create a port you will use to send commands to the bot
     BufferedPort<ImageOf<PixelRgb> > imagePortL;
     
		//BufferedPort<Bottle> out1;
        // Name the ports
	 int averager=0;       
 imagePortL.open("/v1/imagePortL");
 Network::connect("/icub/cam/left","/v1/imagePortL");
 Network::connect("/v1/imagePortL","/v/l"); 
 int xMeanL = 0;
         int yMeanL = 0;
 // read an image from the port
     ImageOf<PixelRgb> *imgL = imagePortL.read();
     int ctLL=0; 
       if (imgL!=NULL) { // check we actually got something
         printf("We got an image of size %dx%d\n", imgL->width(), imgL->height());
         
         int ctL = 0;
         for (int x=0; x<imgL->width(); x++) {
           for (int y=0; y<imgL->height(); y++) {
             PixelRgb& pixel = imgL->pixel(x,y);
             // very simple test for blueishness
             // make sure blue level exceeds red and green by a factor of 2
			 //  if (pixel.r>pixel.b*1.2+10 && pixel.r>pixel.g*1.2+10)
			 //  if (pixel.b>pixel.r*1.2+10 && pixel.b>pixel.g*1.2+10)
             if (pixel.g>pixel.b*1.2+10 && pixel.g>pixel.r*1.2+10) {
              // there's a blueish pixel at (x,y)!
              // let's find the average location of these pixels
              xMeanL += x;
              yMeanL += y;
              ctL++;
             }
           }
         }
         if (ctL>0) {
           xMeanL /= ctL;
           yMeanL /= ctL;
         }
         if (ctL>(imgL->width()/20)*(imgL->height()/20)) {
           printf("Best guess at blue target: %g %g\n", xMeanL, yMeanL);
         }
      }
	 
   /*   if (imgR==NULL)
	 {
       printf ("Image is NULL!!!\n");
     }
      for (int y=0; y<imgR->height(); y++) 
	  {
           for  (int x=0; x<imgR->width(); x++)
		   {
                 PixelRgb& pixel = imgR->pixel(x,y);
					 // very simple test for greenishness
					 // make sure green level exceeds red and blue by a factor of 2
              if ( (pixel.g<=2) && (pixel.r>=145)&& (pixel.r<=155)&& (pixel.b<=2)) 
                 {
                   averager=averager+1;
					 // there's a greenish pixel at (x,y)!
				   iLocX=iLocX+x;
				   iLocY=iLocY+y;
				 }
   }
 }
                    
	               iLocX=iLocX/averager;
				   iLocY=iLocY/averager; */
	                iLocXL= xMeanL;
                    iLocYL= yMeanL;
				    printf("\n\n centroid....%d %d ", iLocXL,iLocYL); 
                   PixelRgb blue(255,255,255);
				   addCircle(*imgL,blue,iLocXL,iLocYL,5);
				   ctLL = (ctLL+10)%imgL->width();	    
				   imagePortL.prepare() = *imgL;
                  
				   imagePortL.write();
				   Sleep(3000);
//                   ImageOf<PixelRgb> *img = imagePort.write();
     // write the image to a file
     //write(*imgR, "img5R.ppm");
	 //Sleep(10000);

 };
		
 void VisionSystem::GCamSceneRred(int ActCamsRred)
 {
	// Create a port you will use to send commands to the bot
     BufferedPort<ImageOf<PixelRgb> > imagePortR;
     
		//BufferedPort<Bottle> out1;
        // Name the ports
	 int averager=0;       
 imagePortR.open("/v1/imagePortR");
 Network::connect("/icub/cam/right","/v1/imagePortR");
 Network::connect("/v1/imagePortR","/v/r"); 
 double xMean = 0;
         double yMean = 0;
 // read an image from the port
     ImageOf<PixelRgb> *imgR = imagePortR.read();
     int ctR=0; 
       if (imgR!=NULL) { // check we actually got something
         printf("We got an image of size %dx%d\n", imgR->width(), imgR->height());
         
         int ct = 0;
         for (int x=0; x<imgR->width(); x++) {
           for (int y=130; y<imgR->height(); y++) {
             PixelRgb& pixel = imgR->pixel(x,y);
             // very simple test for blueishness
             // make sure blue level exceeds red and green by a factor of 2
			 //  if (pixel.r>pixel.b*1.2+10 && pixel.r>pixel.g*1.2+10)
			 //  if (pixel.b>pixel.r*1.2+10 && pixel.b>pixel.g*1.2+10)
             if (pixel.r>pixel.b*1.2+10 && pixel.r>pixel.g*1.2+10) {
              // there's a blueish pixel at (x,y)!
              // let's find the average location of these pixels
              xMean += x;
              yMean += y;
              ct++;
             }
           }
         }
         if (ct>0) {
           xMean /= ct;
           yMean /= ct;
         }
         if (ct>(imgR->width()/20)*(imgR->height()/20)) {
           printf("Best guess at blue target: %g %g\n", xMean, yMean);
         }
      }
	 
   /*   if (imgR==NULL)
	 {
       printf ("Image is NULL!!!\n");
     }
      for (int y=0; y<imgR->height(); y++) 
	  {
           for  (int x=0; x<imgR->width(); x++)
		   {
                 PixelRgb& pixel = imgR->pixel(x,y);
					 // very simple test for greenishness
					 // make sure green level exceeds red and blue by a factor of 2
              if ( (pixel.g<=2) && (pixel.r>=145)&& (pixel.r<=155)&& (pixel.b<=2)) 
                 {
                   averager=averager+1;
					 // there's a greenish pixel at (x,y)!
				   iLocX=iLocX+x;
				   iLocY=iLocY+y;
				 }
   }
 }
                    
	               iLocX=iLocX/averager;
				   iLocY=iLocY/averager; */
	                iLocX= (int)xMean;
                    iLocY= (int)yMean;
				    printf("\n\n centroid....%d %d ", iLocX,iLocY); 
                   PixelRgb blue(255,255,255);
				   addCircle(*imgR,blue,iLocX,iLocY,5);
				   ctR = (ctR+10)%imgR->width();	    
				   imagePortR.prepare() = *imgR;
                  
				   imagePortR.write();
				   Sleep(2000);
//                   ImageOf<PixelRgb> *img = imagePort.write();
     // write the image to a file
     //write(*imgR, "img5R.ppm");
	 //Sleep(10000);
 };

 void VisionSystem::GCamSceneLred(int ActCamsLred)
 {
	// Create a port you will use to send commands to the bot
     BufferedPort<ImageOf<PixelRgb> > imagePortL;
     
		//BufferedPort<Bottle> out1;
        // Name the ports
	 int averager=0;       
 imagePortL.open("/v1/imagePortL");
 Network::connect("/icub/cam/left","/v1/imagePortL");
 Network::connect("/v1/imagePortL","/v/l"); 
 double xMeanL = 0;
         double yMeanL = 0;
 // read an image from the port
     ImageOf<PixelRgb> *imgL = imagePortL.read();
     int ctLL=0; 
       if (imgL!=NULL) { // check we actually got something
         printf("We got an image of size %dx%d\n", imgL->width(), imgL->height());
         
         int ctL = 0;
         for (int x=0; x<imgL->width(); x++) {
           for (int y=130; y<imgL->height(); y++) {
             PixelRgb& pixel = imgL->pixel(x,y);
             // very simple test for blueishness
             // make sure blue level exceeds red and green by a factor of 2
			 //  if (pixel.r>pixel.b*1.2+10 && pixel.r>pixel.g*1.2+10)
			 //  if (pixel.b>pixel.r*1.2+10 && pixel.b>pixel.g*1.2+10)
             if (pixel.r>pixel.b*1.2+10 && pixel.r>pixel.g*1.2+10) {
              // there's a blueish pixel at (x,y)!
              // let's find the average location of these pixels
              xMeanL += x;
              yMeanL += y;
              ctL++;
             }
           }
         }
         if (ctL>0) {
           xMeanL /= ctL;
           yMeanL /= ctL;
         }
         if (ctL>(imgL->width()/20)*(imgL->height()/20)) {
           printf("Best guess at blue target: %g %g\n", xMeanL, yMeanL);
         }
      }
	 
   /*   if (imgR==NULL)
	 {
       printf ("Image is NULL!!!\n");
     }
      for (int y=0; y<imgR->height(); y++) 
	  {
           for  (int x=0; x<imgR->width(); x++)
		   {
                 PixelRgb& pixel = imgR->pixel(x,y);
					 // very simple test for greenishness
					 // make sure green level exceeds red and blue by a factor of 2
              if ( (pixel.g<=2) && (pixel.r>=145)&& (pixel.r<=155)&& (pixel.b<=2)) 
                 {
                   averager=averager+1;
					 // there's a greenish pixel at (x,y)!
				   iLocX=iLocX+x;
				   iLocY=iLocY+y;
				 }
   }
 }
                    
	               iLocX=iLocX/averager;
				   iLocY=iLocY/averager; */
	                iLocXL= (int)xMeanL;
                    iLocYL= (int)yMeanL;
				    printf("\n\n centroid....%d %d ", iLocXL,iLocYL); 
                   PixelRgb blue(255,255,255);
				   addCircle(*imgL,blue,iLocXL,iLocYL,5);
				   ctLL = (ctLL+10)%imgL->width();	    
				   imagePortL.prepare() = *imgL;
                  
				   imagePortL.write();
				   Sleep(3000);
//                   ImageOf<PixelRgb> *img = imagePort.write();
     // write the image to a file
     //write(*imgR, "img5R.ppm");
	 //Sleep(10000);
 };
		
 void VisionSystem::initHead(double headzero)
 {
	  BufferedPort<Bottle> outHead;
		
        outHead.open("/nat/outHead");
		        //
        // Connect the ports so that anything written from /out arrives to /in
        Network::connect("/nat/outHead","/icub/head/rpc:i");
		
     Bottle& outBotH1 = outHead.prepare();   // Get the object
     outBotH1.clear();
	 outBotH1.addString("set"); // put "set" command in the bottle
     outBotH1.addString("pos");
	 outBotH1.addInt(0);
	 outBotH1.addDouble(headzero);

	 outHead.write();                       // Now send i  
	 Time::delay(5);
 };