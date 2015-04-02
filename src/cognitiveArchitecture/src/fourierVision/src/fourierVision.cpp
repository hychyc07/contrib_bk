/************************************************************************************************

fourierVision

A library of computer vision utilities for use with articulated stereo heads 
  
Most code is based on Fourier segmentation, stereo, and optical flow techniques. 
The theory underlying this implementation can be found in:
 
  D. Vernon, Fourier Vision � Segmentation and Velocity Measurement using the Fourier Transform,
     Kluwer Academic Publishers, Norwell, Mass., 195 pages, 2001, (ISBN: 0-7923-7413-4), and
     The Springer International Series in Engineering and Computer Science, Vol. 623 (ISBN: 978-0-7923-7413-8) 

There is also some other code for blob analysis, colour segmentation, vergence control.

This package was implemented by David Vernon
 

Audit Trail
-----------

22/10/10  Fixed bug in rectify() to force the computation of transformation matrix upon first call  DV
25/03/11  Added conditional compilation flag DVR to all the library to be compiled in isolation from the iCub repository
10/04/11  Several updates to optical_flow (region of interest processing, extended padding, non-significant maxima suppression)
04/08/11  Added selective_tuning() 
02/02/12  Added threshold(t,v) to DVimage class 
02/02/12  Added distance_from_background_transform() 
03/08/12  Added subtract_from_constant(value) to DVimage class
*************************************************************************************************/
 
// WARNING ... comment out when compiling in the iCub repository; the default is for standalone compilation
// #define DVR 
 
// System includes
// ------------------------------------------------------------------------

#include <assert.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>

 

// fourierVision includes
// ------------------------------------------------------------------------

#ifdef DVR
#include <fourierVision.h>
#else
#include <iCub/fourierVision.h>
#endif


// Log-Polar includes
// ------------------------------------------------------------------------

#ifdef DVR
#include <RC_DIST_FB_logpolar_mapper.h>
#endif

// Migrate _ftime function and _timeb structure to portable equivalents
// --paulfitz
// ------------------------------------------------------------------------


#ifndef DVR

#include <yarp/os/Time.h>

struct portable_timeb_ {
  int time;
  int millitm;
};

static void portable_ftime(struct portable_timeb_ *data) {
  if (data!=NULL) {
    double now = yarp::os::Time::now();
    //double start = now;
    //now -= start;
    data->time = (int)now;
    data->millitm = (int)((now-data->time)*1000);
  }
}

#else

// use native window version
#define portable_timeb_ _timeb
#define portable_ftime _ftime

#endif

 
/************************************************************************************************

DV classes method definitions
 
	   DVimage
       DVhs_histogram



*************************************************************************************************/
 

/*---------------------------------------------------------------------------------------------*

class DVimage methods

*----------------------------------------------------------------------------------------------*/

	// constructor

	DVimage::DVimage(int w, int h, int mode, unsigned char *image, char *description, int type) {

		  width = w;
        height = h;
        colour_mode = mode; // 1 => GREYSCALE_IMAGE  3 => COLOUR_IMAGE 
        image_type = type;  // DVFLOAT or DVINT 

		  if (w>0 && h>0)
			  if (type == DVINT)
			    idata = new unsigned char[w*h*mode];
			  else
			    fdata = new float[w*h*mode];
			  
		  if (image == NULL) {
			  
			  // no image passed as argument; initialize the image
			  
			  //printf("DVimage constructor: creating image data %d %d %d\n",h,w,mode);

			  if (type == DVINT) {
				  for (int i=0; i<w*h*mode; i++)
					 *(idata+i) = 0;
			  }
			  else {
				  for (int i=0; i<w*h*mode; i++)
					 *(fdata+i) = 0;
           }
		  }
        else {
		  
		     // the image data has been passed as an argument so copy the data

			  if (type == DVINT) {		// image argument and type of DVimage are the same
				  for (int i=0; i<w*h*mode; i++)
					  *(idata+i) = *(image+i);       
			  }
			  else {                     // image argument is unsigned char but type is DVFLOAT so need to convert
				  for (int i=0; i<w*h*mode; i++)
					  *(fdata+i) = (float)(*(image+i));
			  }
		  }


		  if (description == NULL) {

			  // no image description so just create a null string

			  annotation = new char[1];
           annotation = '\0';
		  }
		  else {

			  // do a deep copy on the description

			  annotation = new char[strlen(description)+1];
			  strcpy(annotation, description);
        }
	}
	 


	DVimage::~DVimage() {
		delete [] annotation;
		if (image_type == DVINT) {
			if (width>0 && height>0) delete [] idata;
		}
		else {
			if (width>0 && height>0) delete [] fdata;
		}
	}


	void DVimage::get_size(int *w, int *h) {
		*w = width;
		*h = height;
	}


	int DVimage::get_image_mode() {
		return(colour_mode);
	}


	void DVimage::set_image_mode(int mode) {
		colour_mode = mode;   // needs to be completed
		                 // if we change the colour mode, need to either computer
		                 // greyscale or create other colour bands
	}


   int DVimage::get_image_type() {
		return(image_type);
	}


	// read data from entire image

	void DVimage::read(unsigned char *image) {

		if (image != NULL) {
			if (image_type == DVINT) {
				int j;
				j=width*height*colour_mode;
				for (int i=0; i<j; i++)
					*(image+i) = *(idata+i);
			}
			else { // image_type == DVFLOAT  // may lose data due to casting
				int j;
				j=width*height*colour_mode;
				for (int i=0; i<j; i++)
					*(image+i) = (unsigned char)*(fdata+i);
			}
		}
	}

	void DVimage::read(float *image) {

		if (image != NULL) {
			if (image_type == DVINT) {
				int j;
				j=width*height*colour_mode;
				for (int i=0; i<j; i++)
					*(image+i) = (float) *(idata+i);
			}
			else { // image_type == DVFLOAT
				int j;
				j=width*height*colour_mode;
				for (int i=0; i<j; i++)
					*(image+i) = *(fdata+i);
			}
		}
	}

	  


	  // write data to entire image

	  void DVimage::write(unsigned char *image) {
		  //printf("DVimage::write - %d %d %d\n",width, height, colour_mode);

		  if (image != NULL) {
			  if (image_type == DVINT) {  // unsigned char to unsigned char ... straight copy
			      int j;
				   j=width*height*colour_mode;
				   for (int i=0; i<j; i++)
				      *(idata+i) = *(image+i);
			  }
			  else { // image_type == DVFLOAT; unsigned char to float ... simple cast

			      int j;
				   j=width*height*colour_mode;
				   for (int i=0; i<j; i++)
				      *(fdata+i) = (float)(*(image+i));
			  }
		  }
	  }


	  void DVimage::write(float *image) {
		  //printf("DVimage::write - %d %d %d\n",width, height, colour_mode);

		  if (image != NULL) {
			  if (image_type == DVINT) {  // float to unsigned char ...  dangerous as the cast wmay cause loss of data

				  int j;
				   j=width*height*colour_mode;
				   for (int i=0; i<j; i++)
				      *(idata+i) = (unsigned char) *(image+i);
			  }
			  else { // image_type == DVFLOAT; float to float ... straight copy

			      int j;
				   j=width*height*colour_mode;
				   for (int i=0; i<j; i++)
				      *(fdata+i) = (*(image+i));
			  }
		  }
	  }
	  	  

	  // read and return a single pixel value from the image
	  // note that the byte number needs to be specified if the mode is COLOUR_IMAGE
	  // (it defaults to byte 0, i.e. the first byte)

	  void DVimage::get_pixel(int x, int y, unsigned char *value, int byte_number) {
		  if (image_type == DVINT) { 
			*value = (*(idata+(y*width*colour_mode + x*colour_mode + byte_number)));
		  }
		  else {
			*value = (unsigned char) (*(fdata+(y*width*colour_mode + x*colour_mode + byte_number)));
		  }
	  }

	  void DVimage::get_pixel(int x, int y, float *value, int byte_number) {
		  if (image_type == DVINT) { 
			*value = (float)(*(idata+(y*width*colour_mode + x*colour_mode + byte_number)));
		  }
		  else {
			*value = (*(fdata+(y*width*colour_mode + x*colour_mode + byte_number)));
		  }
	  }

	  
	  // write a single pixel value to the image
	  // note that the byte number needs to be specified if the mode is COLOUR_IMAGE
	  // (it defaults to byte 0, i.e. the first byte)

	  void DVimage::put_pixel(int x, int y, unsigned char value, int byte_number) {

		  //printf("DVimage::put_pixel(%d, %d, %d, %d) colour mode %d\n", x, y, value, byte_number, colour_mode);
		  if (image_type == DVINT) { 
			*(idata+(y*width*colour_mode + x*colour_mode + byte_number))=value;
		  }
		  else {
			*(fdata+(y*width*colour_mode + x*colour_mode + byte_number))=(float)value;
		  }
	  }

	  void DVimage::put_pixel(int x, int y, float value, int byte_number) {

		  //printf("DVimage::put_pixel(%d, %d, %f, %d) colour mode %d\n", x, y, value, byte_number, colour_mode);
		  if (image_type == DVINT) { 
			*(idata+(y*width*colour_mode + x*colour_mode + byte_number))=(unsigned char)value;
		  }
		  else {
			*(fdata+(y*width*colour_mode + x*colour_mode + byte_number))=value;
		  }
	  }

	  // return pointer to the image annotation string
 
	  char *DVimage::read_annotation() {
        return(annotation);
	  }

	  
	  // write a string to the image annotation

	  void DVimage::write_annotation(char *description) {
        if (annotation != NULL)
			  delete [] annotation;
		   annotation = new char[strlen(description)+1];
		   strcpy(annotation, description);
	  }


	  // initialize entire image to zero

	  void DVimage::initialize() {
		  //printf("DVimage::initialize\n");

			  if (image_type == DVINT) {  // unsigned char to unsigned char ... straight copy
			      int j;
				   j=width*height*colour_mode;
				   for (int i=0; i<j; i++)
				      *(idata+i) = 0;
			  }
			  else { // image_type == DVFLOAT; unsigned char to float ... simple cast

			      int j;
				   j=width*height*colour_mode;
				   for (int i=0; i<j; i++)
				      *(fdata+i) = (float)0.0;
			  }
	  }


	  	  
	  // constrast stretch an image in place; minimum and maximum values will be 0 and 255 after invoking this method


	  void DVimage::contrast_stretch() {
		  //printf("DVimage::contrast_stretch\n");

		  float min, max;
		  int i, j;

 
	     // find range of values so that we can contrast stretch

 	     min = 1E10;
	     max = -1E10;

		  j=width*height*colour_mode;
 
		  if (image_type == DVINT) {  //  
	       for (i=0; i<j; i++){
		      if ((float) *(idata+i) > max) max = (float) *(idata+i);
	         if ((float) *(idata+i) < min) min = (float) *(idata+i);
			 }
		  }
		  else { // image_type == DVFLOAT 
	       for (i=0; i<j; i++){
		      if (*(fdata+i) > max) max = *(fdata+i);
	         if (*(fdata+i) < min) min = *(fdata+i);
			 }
		  }

		  		  
		  if (image_type == DVINT) {  //  
	       for (i=0; i<j; i++){
		      *(idata+i)  = (unsigned char) (255 * (( (float)(*(idata+i)) - min)/(max-min)));
			 }
		  }
		  else { // image_type == DVFLOAT 
	       for (i=0; i<j; i++){
		      *(fdata+i)  = (255 * (( (*(fdata+i)) - min)/(max-min)));
			 }
		  } 
	  }
 

   // threshold an image in place; any value > t are assigned value v NB: > t, not >= t, so t == 0 leaves zero values unchanged.


	  void DVimage::threshold(float t, float v) {
		  //printf("DVimage::threshold\n");

 		  int i, j;

		  j=width*height*colour_mode;
 
		  if (image_type == DVINT) {  //  
	       for (i=0; i<j; i++){
		      if ((float) *(idata+i) > t) *(idata+i) = (unsigned char) v;
			 }
		  }
		  else { // image_type == DVFLOAT 
	       for (i=0; i<j; i++){
		      if (*(fdata+i) > t)  *(fdata+i) = v;
			 }
		  }

	  }
     	  	  
	  // subtract image from a constant value  

	  void DVimage::subtract_from_constant(float constant) {

		  int i, j;
 
		  j=width*height*colour_mode;
		  		  
		  if (image_type == DVINT) {  //  
	       for (i=0; i<j; i++){
		      *(idata+i)  = (unsigned char) (constant -(float)(*(idata+i)));
			 }
		  }
		  else { // image_type == DVFLOAT 
	       for (i=0; i<j; i++){
		      *(fdata+i)  = (constant - (*(fdata+i)));
			 }
		  } 

        //printf("DVimage::subtract_from_constant %f\n", constant);

	  }

/*---------------------------------------------------------------------------------------------*

class DVhs_histogram methods

*----------------------------------------------------------------------------------------------*/
 
// constructor

DVhs_histogram::DVhs_histogram(int h, int s) {

   hue_dimension = h;
   saturation_dimension = s;
 
   if (h>0 && s>0) {
      data = new int[h*s];
 			  
      for (int i=0; i<h*s; i++)
		   *(data+i) = 0;
 
 	}
}
	 

// destructor

DVhs_histogram::~DVhs_histogram() {

   if (hue_dimension>0 && saturation_dimension>0) delete [] data;
}


void DVhs_histogram::get_dimensions(int *h, int *s) {
   *h = hue_dimension;
   *s = saturation_dimension;
}
 	  	  


void DVhs_histogram::get_bin(int h, int s, int *value) {

   *value = (*(data+(s*hue_dimension + h)));
 
}

 
void DVhs_histogram::put_bin(int h, int s, int value) {

   *(data+(s*hue_dimension + h))=value;
}

 
void DVhs_histogram::increment_bin(int h, int s) {

   *(data+(s*hue_dimension + h)) += 1;
}

 
 // initialize histogram to zero

void DVhs_histogram::initialize() {
 
   int j;
   j=hue_dimension*saturation_dimension;
   for (int i=0; i<j; i++)
      *(data+i) = 0;
}


 // compute the hue and saturation values corresponding to the mode of the histogram
 // these values are in the range [0,360] and [0,1] respectively
 // NB they are NOT the bin numbers of the histogram mode

void DVhs_histogram::hsMode(float *hue, float *saturation) {

   int i, j;
   int max_value;
   int max_i;
   int max_h;
   int max_s;

   max_value = 0;
   max_i = 0;

   j=hue_dimension*saturation_dimension;
   for (i=0; i<j; i++) {
      if (*(data+i) >= max_value) {
         max_value = *(data+i);
         max_i = i;
      }
   }

   max_s = max_i / hue_dimension;
   max_h = max_i - (max_s * hue_dimension);

   if (saturation_dimension <= 1) {
      *saturation = 0.5;
   }
   else {
      *saturation = (float) max_s / (float) (saturation_dimension-1);
   }

   if (hue_dimension <= 1) {
      *hue = 180;
   }
   else {
      *hue = (float) (360 * max_h) / (float) (hue_dimension-1);
   }
}



/****************************************************************
* 
*  Routine Name: selective_tuning
* 
*      Purpose: Implementation of the Selective Tuning Model (STM) of visual attention,
*      a model developed by John Tsotsos, York University, Canada. 
*      The model is described in detail in a paper published in Artificial Intelligence:
*
*      J. K. Tsotsos, S. Culhane, W. Wai, Y. Lai, N. David, and F. Nuflo.
*      Modeling visual attention via selective tuning. Artificial Intelligence, 78:507�547, 1995.
*
*      Additional background material may be found on the STM website at
*      http://web.me.com/john.tsotsos/Visual_Attention/Selective_Tuning.html
*
*      Inputs
*      ------
*
*      number_of_levels        - int parameter; number of levels in the STM pyramid
*
*      bias_image              - pointer to DVimage with the bias unit data for level 1 of the pyramid. 
*                                The width and height of this data-structure should match the resolution 
*                                of level 1 of the pyramid. The depth should have the value 1 (GREYSCALE_IMAGE).
*                                The type of the image must be floating point; this is set by using DVFLOAT as an argument 
*                                when instantiating the image.
*
*      input_image             - pointer to DVimage with the image data for level 1 of the pyramid. 
*                                The width and height of this data-structure should match the resolution of level 1 
*                                of the pyramid. The depth should have the value 1 (GREYSCALE_IMAGE). 
*                                The type of the image can be either 8-bit integer or floating point; 
*                                this is set by using DVINT and DVFLOAT, respectively, as an argument when instantiating 
*                                the image.
*
*      minRFsize               - int; minimum length of the rectangular receptive field
*
*      maxRFsize               - int; maximum length of the rectangular receptive field
*
*      rectangularRF           - int; flag to invoke use of rectangular receptive fields (when value = 1)
*
*      localMax                - int; flag to invoke local maximum instead of local average when computing 
*                                the image pyramid (when value = 1)
*
*      Outputs
*      -------
*
*      number_of_winning_units - pointer to int; the number of winning units in level 1 of the pyramid. 
*                                This is an integer value, of type int, passed (i.e. returned) by reference.
* 
*      winning_units           - The x and y coordinates of the winning units in level 1 of the pyramid, 
*                                in units of pixels, and in the range of the resolution of input image data 
*                                for level 1 of the pyramid. These are to be returned as an array of structures, 
*                                by maxima_data_type defined in fourierVision.h.
* 
*      The function return an int value; 0 for failure; 1 for success
*
*
* Written By:    David Vernon
* Date:          August 4, 2011
*
* Modifications: None to date                               
*
****************************************************************/
 
int selective_tuning(int number_of_levels, 
                     DVimage *bias_image, DVimage *input_image, int minRFSize, int maxRFSize, int rectangularRF, int localMax,
                     int *number_of_winning_units, maxima_data_type winning_units[])

{

   int selective_tuning_forward_mapping(int p, int q, int r, int s, int pyramid_level_reduction, int number_of_receptive_fields, int number_of_RF_sides, int minRFSize, int sminRFSize);
   void selective_tuning_inverse_mapping(int k, int *p_prime, int *q_prime, int *r_prime, int *s_prime, int pyramid_level_reduction, int number_of_receptive_fields, int number_of_RF_sides, int minRFSize, int sminRFSize);

   int i, j, k, l, p, q, r, s;
   int p_prime, q_prime, r_prime, s_prime;
   int ii, jj, kk;
   int max_l, max_i, max_j, max_p;
   int max_RF_area;
   int width, height, depth; 
   float alpha, beta;
   float aspect_ratio;
   int debug;
   static int local_number_of_levels = 0;
   static int local_resolution[MAX_PYRAMID_LEVELS];
   static float **image_values[MAX_PYRAMID_LEVELS];
   static float **new_image_values[MAX_PYRAMID_LEVELS];
   static float **bias_image_values[MAX_PYRAMID_LEVELS];
   static float ***interpretive_units[MAX_PYRAMID_LEVELS];
   static float ***new_interpretive_units[MAX_PYRAMID_LEVELS];
   static float ***gating_units[MAX_PYRAMID_LEVELS];
   static float ***new_gating_units[MAX_PYRAMID_LEVELS];
   static float ***gating_control_units[MAX_PYRAMID_LEVELS];
   static float ***bias_units[MAX_PYRAMID_LEVELS];
   static float *interpretive_unit_weights[MAX_PYRAMID_LEVELS];

   int resolution[MAX_PYRAMID_LEVELS];
   double scale_normalization_factor;
   bool rebuild = false;
   int number_of_maxima = 2;
   int non_maxima_suppression_radius = 20;
   float float_pixel_value;	 
   double delta;
   double sum_delta;
   double sum_RF;
   double maximum_value;
   double theta;
   int iteration;
   bool finished;
   double gating_unit_difference;
   double image_value_difference;
   int   number_of_gating_units;
   int   number_of_receptive_fields;
   int   pyramid_level_reduction;
   int   number_of_RF_sides;
   int   size_RF;
   int   r_lower_limit;
   int   r_upper_limit;  
   int   s_lower_limit;
   int   s_upper_limit;  
   int   centre_x;
   int   centre_y;
   int   gamma;
   bool  uniform_image; 
   bool  found;
   int sminRFSize;
   int smaxRFSize;

   /* set debug flags */

   debug = FALSE;

   if (false && debug) {
      printf("selective_tuning: number of levels = %d\n",number_of_levels); 
      bias_image->get_size(&width,&height);
      depth = bias_image->get_image_mode();
      printf("selective_tuning: bias image width, height, depth = %d %d %d \n", width, height, depth);
      input_image->get_size(&width,&height);
      depth = input_image->get_image_mode();
      printf("selective_tuning: input image width, height, depth = %d %d %d \n", width, height, depth);
      printf("selective_tuning: minRFSize = %d\n",minRFSize);  
      printf("selective_tuning: maxRFSize = %d\n",maxRFSize);  
      printf("selective_tuning: rectangularRF = %d\n",rectangularRF);  
      printf("selective_tuning: localMax = %d\n",localMax);  
 }

   // set the maximum number of iterations on the WTA competition
   // as well as the normalization factor constants
   // ===========================================================

   gamma = 10;  
   //gamma = 6;  
   
   alpha = (float) 10.0;
   beta  = (float) 1.03;
   //alpha = (float) 1.0;
   //beta  = (float) 2;

   // build resolution array
   // ======================
 
   input_image->get_size(&width,&height);
   aspect_ratio = (float)width / (float)height;

   for (i=0; i<number_of_levels; i++) {
      resolution[i] = (int) (width * pow( (float)0.5, i ));  // reduce resolution by a factor of two on each level
      //if (debug) printf("selective_tuning: pyramid resolution level %d is %d\n", i, resolution[i]);
   }


   // Build data structures if necessary
   // ==================================

   rebuild = false;
   if (number_of_levels != local_number_of_levels) {
      rebuild = true;
   }
   else {
      for (i=0; i<number_of_levels; i++) {
         if ((resolution[i] != local_resolution[i]) ) {
            rebuild = true;
         }
      }
   }


   // resolution halves from level to level so receptive fields (i.e. interpretive unit values) 
   // from four assemblies at level l feed gating units at level l+1
   
   if (rectangularRF) 
      number_of_receptive_fields = (maxRFSize - minRFSize +1) * (maxRFSize - minRFSize +1); 
   else
      number_of_receptive_fields = (maxRFSize - minRFSize +1); 

   number_of_gating_units =  number_of_receptive_fields * 4; 

   if (debug) {
      //printf("selective_tuning: number of receptive fields %d; number of gating units %d \n",number_of_receptive_fields,number_of_gating_units);  
   }

   if (rebuild == true) {
 
      if (debug) {
         printf("selective_tuning: rebuilding \n");  
      }
 
      // retain the pyramid parameters
      // -----------------------------

      local_number_of_levels = number_of_levels;
      for (i=0; i<number_of_levels; i++) {
         local_resolution[i]       = resolution[i];
      }


      // generate the pyramid of image values
      // ------------------------------------

      if (number_of_levels > MAX_PYRAMID_LEVELS) {
         printf("selective_tuning: ERROR cannot generate a pyramid with %d levels\n",number_of_levels);  
         return 0;
      }

      for (l=0; l<number_of_levels; l++) {
         width = resolution[l];
         height = (int) (((float) resolution[l] / aspect_ratio)+0.5); // round
         if (image_values[l] != NULL) 
            free(image_values[l]);
         image_values[l] = (float **) calloc( width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (image_values[l][i] != NULL) 
               free(image_values[l][i]);
            image_values[l][i] = (float *) calloc(height, sizeof(float));  
         }
      }

      for (l=0; l<number_of_levels; l++) {
         width = resolution[l];
         height = (int) (((float) resolution[l] / aspect_ratio)+0.5); // round
         if (new_image_values[l] != NULL) 
            free(new_image_values[l]);
         new_image_values[l] = (float **) calloc( width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (new_image_values[l][i] != NULL) 
               free(new_image_values[l][i]);
            new_image_values[l][i] = (float *) calloc(height, sizeof(float));  
         }
      }



      // generate the pyramid of bias image values
      // -----------------------------------------

      if (number_of_levels > MAX_PYRAMID_LEVELS) {
         printf("selective_tuning: ERROR cannot generate a pyramid with %d levels\n",number_of_levels);  
         return 0;
      }
 

      for (l=0; l<number_of_levels; l++) {
         width = resolution[l];
         height = (int) (((float) resolution[l] / aspect_ratio)+0.5); // round
         if (bias_image_values[l] != NULL) 
            free(bias_image_values[l]);
         bias_image_values[l] = (float **) calloc( width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (bias_image_values[l][i] != NULL) 
               free(bias_image_values[l][i]);
            bias_image_values[l][i] = (float *) calloc(height, sizeof(float));  
         }
      }

      
      // generate the pyramid of interpretive units:
      // one interpretive unit for every receptive field  
      // multiplied by the number of image locations at a given level that is part of 
      // the square receptive field of the global winning unit at the top level
      // -----------------------------------------------------------------------------

      for (l=0; l<number_of_levels; l++) {

         // restrict the pyramid to have a single unit at the top
         // this is the unit that corresponds to the global winning unit
         
         width = resolution[l]  / resolution[number_of_levels-1];  
         height = width;  

         if (interpretive_units[l] != NULL) 
            free(interpretive_units[l]);
         interpretive_units[l] = (float ***) calloc( width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (interpretive_units[l][i] != NULL) 
               free(interpretive_units[l][i]);
            interpretive_units[l][i] = (float **) calloc(height, sizeof(float));  

            for (j=0; j<height; j++) {
               if (interpretive_units[l][i][j] != NULL) 
                  free(interpretive_units[l][i][j]);
               interpretive_units[l][i][j] = (float *) calloc(number_of_receptive_fields, sizeof(float));
            }
         }
      }

            
      for (l=0; l<=0; l++) {

         // just need the new interpretive units at the bottom level for the final WTA
         
         width = resolution[l]  / resolution[number_of_levels-1];  
         height = width;  

         if (new_interpretive_units[l] != NULL) 
            free(new_interpretive_units[l]);
         new_interpretive_units[l] = (float ***) calloc( width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (new_interpretive_units[l][i] != NULL) 
               free(new_interpretive_units[l][i]);
            new_interpretive_units[l][i] = (float **) calloc(height, sizeof(float));  

            for (j=0; j<height; j++) {
               if (new_interpretive_units[l][i][j] != NULL) 
                  free(new_interpretive_units[l][i][j]);
               new_interpretive_units[l][i][j] = (float *) calloc(number_of_receptive_fields, sizeof(float));
            }
         }
      }

      // generate the pyramid of gating units
      // ------------------------------------

      for (l=1; l<number_of_levels; l++) {  // NB WE DONT NEED GATING UNITS AT THE FIRST LEVEL 

         width = resolution[l]  / resolution[number_of_levels-1];  
         height = width;  

         if (gating_units[l] != NULL) 
            free(gating_units[l]);
         gating_units[l] = (float ***) calloc(width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (gating_units[l][i] != NULL) 
               free(gating_units[l][i]);
            gating_units[l][i] = (float **) calloc(height, sizeof(float *));

            for (j=0; j<height; j++) {
               if (gating_units[l][i][j] != NULL) 
                  free(gating_units[l][i][j]);
               gating_units[l][i][j] = (float *) calloc(number_of_gating_units, sizeof(float)); // 4 * RF
            }
         }
      }
   
            
      for (l=1; l<number_of_levels; l++) {

         width = resolution[l]  / resolution[number_of_levels-1];  
         //height = (int) (((float) resolution[l] / aspect_ratio)+0.5) / resolution[number_of_levels-1] ; // round
         height = width;  

         if (new_gating_units[l] != NULL) 
            free(new_gating_units[l]);
         new_gating_units[l] = (float ***) calloc(width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (new_gating_units[l][i] != NULL) 
               free(new_gating_units[l][i]);
            new_gating_units[l][i] = (float **) calloc(height, sizeof(float *));

            for (j=0; j<height; j++) {
               if (new_gating_units[l][i][j] != NULL) 
                  free(new_gating_units[l][i][j]);
               new_gating_units[l][i][j] = (float *) calloc(number_of_gating_units, sizeof(float)); // 4 * RF
            }
         }
      }

      for (l=1; l<number_of_levels; l++) {

         width = resolution[l]  / resolution[number_of_levels-1];  
         height = width;  

         for (i=0; i<width; i++) {
            for (j=0; j<height; j++) {
               for (p=0; p<number_of_gating_units; p++) {
                  gating_units[l][i][j][p] = 0.0;  
                  new_gating_units[l][i][j][p] = 0.0;  
               }
            }
         }
      }


            
      // generate the pyramid of gating control units
      // --------------------------------------------

      for (l=0; l<number_of_levels; l++) {

         width = resolution[l]  / resolution[number_of_levels-1];  
         height = width;  

         if (gating_control_units[l] != NULL) 
            free(gating_control_units[l]);
         gating_control_units[l] = (float ***) calloc( width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (gating_control_units[l][i] != NULL) 
               free(gating_control_units[l][i]);
            gating_control_units[l][i] = (float **) calloc(height, sizeof(float *));  
            
            for (j=0; j<height; j++) {
               if (gating_control_units[l][i][j] != NULL) 
                  free(gating_control_units[l][i][j]);
               gating_control_units[l][i][j] = (float *) calloc(number_of_receptive_fields, sizeof(float));
            }
         }
      }
   
                        
      // generate the pyramid of bias units
      // ----------------------------------

      for (l=0; l<number_of_levels; l++) {

         width = resolution[l]  / resolution[number_of_levels-1];  
         height = width;  

         if (bias_units[l] != NULL) 
            free(bias_units[l]);
         bias_units[l] = (float ***) calloc( width, sizeof(float *));
       
         for (i=0; i<width; i++) {
            if (bias_units[l][i] != NULL) 
               free(bias_units[l][i]);
            bias_units[l][i] = (float **) calloc(height, sizeof(float));  
                       
            for (j=0; j<height; j++) {
               if (bias_units[l][i][j] != NULL) 
                  free(bias_units[l][i][j]);
               bias_units[l][i][j] = (float *) calloc(number_of_receptive_fields, sizeof(float));
            }

         }
      }

    
          
      // generate the set of interpretive unit weights, one set for each level of the pyramid,
      // with the same number of weights as there are gating units
      // -------------------------------------------------------------------------------------

      for (l=0; l<number_of_levels; l++) {
         if (interpretive_unit_weights[l] != NULL) 
            free(interpretive_unit_weights[l]);
         interpretive_unit_weights[l] = (float *) calloc(number_of_gating_units, sizeof(float *));
      }
   
      for (l=0; l<number_of_levels; l++) {
         for (i=0; i<number_of_gating_units; i++) {
            //interpretive_unit_weights[l][i] = (float)(1.0)/(float)(number_of_gating_units);   // just use a simple average for now
            interpretive_unit_weights[l][i] = (float)(1.0);
         }
      }
   }
   
   
   // Pass 1: propagate the input image values up the pyramid from level 1 to level L 
   //        (indexed by 0 and number_of_levels, respectively)
   //        
   //        Then perform a WTA on the top level to find the globally most salent unit
   // ===============================================================================

   // first, transfer the feature values from the image to level 1 of the pyramid
   // at the same time, identify the maximum value

   if (false && debug) {
       printf("selective_tuning: Pass 1 ... initializing level 1 to input image values\n");  
   }

   input_image->get_size(&width,&height);
   for (i=0; i<width; i++) {
      for (j=0; j<height; j++) {
         input_image->get_pixel(i,j,&float_pixel_value); 
         image_values[0][i][j] = float_pixel_value;  
       } 
   }
   
   // next, for each level above level 1, compute the local average

   for (l=1; l<number_of_levels; l++) {   

      width = resolution[l];
      height = (int) (((float) resolution[l] / aspect_ratio)+0.5); // round

      if (debug) {
         //printf("selective_tuning:            propagating image values to level %d\n",l);  
      }

      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {

            image_values[l][i][j] = 0.0;

            ii =  i * 2;  // coordinates at level l-1
            jj =  j * 2;

            if (localMax) {  
                           
               // propagate image value as maximum rather than mean; 
               // the prevents small objects from being averaged out
               // as we ascend the pyramid
 
               if (image_values[l][i][j] < image_values[l-1][ii][jj]) 
                   image_values[l][i][j] = image_values[l-1][ii][jj];
               if (image_values[l][i][j] < image_values[l-1][ii][jj+1]) 
                   image_values[l][i][j] = image_values[l-1][ii][jj+1];
               if (image_values[l][i][j] < image_values[l-1][ii+1][jj]) 
                   image_values[l][i][j] = image_values[l-1][ii+1][jj];
               if (image_values[l][i][j] < image_values[l-1][ii+1][jj+1]) 
                   image_values[l][i][j] = image_values[l-1][ii+1][jj+1];
 
            }
            else {
               image_values[l][i][j] =  (image_values[l-1][ii][jj] + 
                                         image_values[l-1][ii][jj+1] + 
                                         image_values[l-1][ii+1][jj] + 
                                         image_values[l-1][ii+1][jj+1]) / 4; 
            }

         }
      }
   }
   

   // apply bias values before doing the global WTA

   bias_image->get_size(&width,&height);

   if ((width == resolution[0]) && (height == (int) (((float) resolution[0] / aspect_ratio)+0.5)))  {
      if (false  && debug) {
         printf("selective_tuning: Pass 1 ... applying bias to image values at top level of the pyramid \n");  
      }

      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            bias_image->get_pixel(i,j,&float_pixel_value); 
            bias_image_values[0][i][j] = float_pixel_value;  
         } 
      }

      // next, for each level above level 1, compute the local average

      for (l=1; l<number_of_levels; l++) {

         width = resolution[l];
         height = (int) (((float) resolution[l] / aspect_ratio)+0.5); // round

         if (false && debug) {
            printf("selective_tuning:            propagating bias image values to level %d\n",l);  
         }

         for (i=0; i<width; i++) {
            for (j=0; j<height; j++) {

               bias_image_values[l][i][j] = 0.0;

               ii = (i * resolution[l-1]) / resolution[l];  // coordinates at level l-1
               jj = (j * resolution[l-1]) / resolution[l];

               bias_image_values[l][i][j] += (bias_image_values[l-1][ii][jj] + 
                                              bias_image_values[l-1][ii][jj+1] + 
                                              bias_image_values[l-1][ii+1][jj] + 
                                              bias_image_values[l-1][ii+1][jj+1]) / 4; 
            }
         }
      }
   
      // now bias the image pyramid

      l = number_of_levels-1;
      width = resolution[l];
      height = (int) (((float) resolution[l] / aspect_ratio)+0.5); // round
      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            image_values[l][i][j] = image_values[l][i][j] * bias_image_values[l][i][j];
         } 
      }
   }


   // compute the maximum value and determine whether the image is uniform, 
   // in which case it's not possible to run a WTA

   maximum_value = 0;
   uniform_image = true;

   l = number_of_levels-1;
   width = resolution[l];
   height = (int) (((float) resolution[l] / aspect_ratio)+0.5); // round

   for (i=0; i<width; i++) {
      for (j=0; j<height; j++) {
         if (image_values[l][i][j] > maximum_value) {
            maximum_value = image_values[l][i][j];
         }
      } 
   }
 
   if (maximum_value == 0) {
       if (debug) printf("selective_tuning: NO GLOBAL WINNING UNIT\n");
       *number_of_winning_units = 0;
       return 0;
   };
 

   // Perform the global WTA competition 
  
   l = number_of_levels-1;
   width = resolution[l];
   height = (int) (((float) resolution[l] / aspect_ratio)+0.5); // round
 
   // compute threshold theta

   theta = (maximum_value * interpretive_unit_weights[l][0]) / (pow((float)2,(float)gamma) + 1);
 
   if (debug) {
      printf("selective_tuning: Pass 1 ... maximum %4.1f, gamma %d, theta %f \n",maximum_value, gamma, theta);  
   }
 
   // now, start the WTA competition
  
   iteration = 0;
 
   do {                         
      iteration++;
       
      for (p = 0; p < width; p++) {
         for (q = 0; q < height; q++) {

            sum_delta = 0.0;

            for (r = 0; r < width; r++) {
               for (s = 0; s < height; s++) {
                 if ((p!=r) || (q!=s)) {
                     delta = image_values[l][r][s] - image_values[l][p][q];
                              
                     if (delta > theta) {
                        sum_delta += delta;
                     }
                     if (debug) {
                        //printf("selective_tuning: Pass 1 ... maximum %f, gamma %f, theta %f, delta %f\n",maximum_value, gamma, theta, delta);  
                     }
                  }
               }
            }

            image_value_difference = image_values[l][p][q] - sum_delta;
                  
            if (image_value_difference < 0) {
               image_value_difference = 0;
            }

            new_image_values[l][p][q] = (float) image_value_difference; 

            if (debug) {
               //printf("selective_tuning: Pass 1 ... %d %d %d iteration %d I old %4.1f I new %4.1f\n",l,i,j,iteration,image_values[l][p][q],new_image_values[l][p][q]);  
            }
         }
      }

      // now copy the new image values back to the originals for the next iteration

      for (p = 0; p < width; p++) {
         for (q = 0; q < height; q++) {
            if (debug) {  
               //printf("selective_tuning:            WTA  %d %d %d %d %d iteration %d \n",l,i,j,p,q,iteration);  
               //printf("selective_tuning:            old image %4.1f, new image %4.1f\n",image_values[l][p][q],new_image_values[l][p][q]);  
            }
            image_values[l][p][q] = new_image_values[l][p][q];
         }
      }

      // now check to see if convergence has been reached
      // check that all image values are either zero or
      // greater than theta and the difference between these is less than theta

      finished = true;

      for (p = 0; p < width; p++) {
         for (q = 0; q < height; q++) {

            if (image_values[l][p][q] > 0) {
               if (image_values[l][p][q] < theta) {
                  finished = false;
               }
               else {
                  for (r = 0; r < width; r++) {
                     for (s = 0; s < height; s++) {
                        if (image_values[l][r][s] > 0) {
                           if (fabs(image_values[l][r][s] - image_values[l][p][q]) > theta) {
                              finished = false;
                           }
                        }
                     }
                  }
               }
            }
         }
      }

   } while (!finished);

   // pick up the first winning unit

   *number_of_winning_units = 0;

   for (p = 0; (p < width) && (*number_of_winning_units == 0); p++) {
     for (q = 0; (q < height) && (*number_of_winning_units == 0); q++) {
        if (image_values[l][p][q] != 0.0) {
             winning_units[*number_of_winning_units].x = p * (int) pow((float)2,number_of_levels-1);
             winning_units[*number_of_winning_units].y = q * (int) pow((float)2,number_of_levels-1);
             winning_units[*number_of_winning_units].value = image_values[0][p][q];
             winning_units[*number_of_winning_units].rf_x =  0;  // show receptive field with size zero
             winning_units[*number_of_winning_units].rf_y =  0;  // to distinguish the result from the selective tuning
             if (debug) {
                //printf("selective_tuning: global winning unit at (%d, %d) = (%d, %d): %6.4f\n",p,q, winning_units[*number_of_winning_units].x,winning_units[*number_of_winning_units].y,image_values[l][p][q]);
             
             }
             *number_of_winning_units =  *number_of_winning_units + 1;
        }
     }
   }
 
   if (*number_of_winning_units == 0) {
       printf("selective_tuning: NO GLOBAL WINNING UNIT\n");
       return 0;
   };
 
   // Pass 2: propagate unity bias values down the pyramid from level L to level 1 
   //        Then set gating control values to zero
   // ===============================================================================

   if (false && debug) {
      printf("selective_tuning: Pass 2 ... propagating bias units (aspect ratio %f)\n",aspect_ratio);  
   }
 
   for (l=number_of_levels-1; l>=0; l--) {

      width = resolution[l]  / resolution[number_of_levels-1];  
      height = width;  

      if (false && debug) {
         printf("selective_tuning: Pass 2 ... propagating bias units to level %d (%d %d)\n",l,width,height);  
      }

      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            for (p=0; p<number_of_receptive_fields; p++) {
               bias_units[l][i][j][p] = 1.0;  
            }
         }
      }
   }



   // now set gating control units to zero
         
   for (l=0; l<number_of_levels; l++) {

      width = resolution[l]  / resolution[number_of_levels-1];  
      height = width;  

      if (false && debug) {
         printf("selective_tuning: Pass 2 ... initializing GCU to zero on level %d (%d %d)\n",l,width,height);  
      }
      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) { 
            for (p=0; p<number_of_receptive_fields; p++) {
               gating_control_units[l][i][j][p] = 0.0;  
            }
         }
      }
   }


   // Pass 3: compute the interpretive unit values and the gating unit values on every level
   // ================================================================================================

           
   pyramid_level_reduction = resolution[0] / resolution[1];

   for (l=0; l<number_of_levels-1; l++) {  // only compute interpretive units on levels 1 to L-1 (0-number_of_levels-2)

      width = resolution[l]  / resolution[number_of_levels-1];  
      height = width;  

 

      if (false && debug) {
         printf("selective_tuning: Pass 3 ... computing interpretive units on level %d (%d %d)\n",l,width,height);  
      }

      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {

            centre_x = (winning_units[0].x / (int) pow((float)2,l)) + i + 1;  // adjust the coordinates as we move up the pyramid
            centre_y = (winning_units[0].y / (int) pow((float)2,l)) + j + 1;

            for (r = minRFSize; r <= maxRFSize; r++) {

               if (rectangularRF) {
                   sminRFSize = minRFSize;
                   smaxRFSize = maxRFSize;
                   number_of_RF_sides = maxRFSize - minRFSize + 1;
               }
               else {
                  sminRFSize = r;
                  smaxRFSize = r;
                  number_of_RF_sides = 1;
               }

    
               for (s = sminRFSize; s <= smaxRFSize; s++) {

                  k =   (s-sminRFSize) * (number_of_receptive_fields / number_of_RF_sides) + (r-minRFSize);

            
                  // calculate the mean value of the receptive field centred at the location of the current
                  // winning point, but in the current level

                  sum_RF=0;
                  size_RF=0;

                  r_lower_limit =  -r/2;              // distribute receptive field symmetrically 
                  r_upper_limit =   r/2;              // around the current point
                  if ((r % 2)==0)                     // if the receptive field size is even
                  r_upper_limit = r_upper_limit - 1;  // need to reduce the range by one

                  s_lower_limit =  -s/2;              // distribute receptive field symmetrically 
                  s_upper_limit =   s/2;              // around the current point
                  if ((s % 2)==0)                     // if the receptive field size is even
                  s_upper_limit = s_upper_limit - 1;  // need to reduce the range by one

                  for (ii = r_lower_limit; ii <= r_upper_limit; ii++) {
                     for (jj = s_lower_limit; jj <= s_upper_limit; jj++) {

                        // check bounds

                        if ((centre_x+ii >= 0) && (centre_x+ii < resolution[l]) &&
                            (centre_y+jj >= 0) && (centre_y+jj < (int) (((float) resolution[l] / aspect_ratio)+0.5))) {

                           sum_RF += image_values[l][centre_x+ii][centre_y+jj];
                           size_RF++;
                        }
                     }
                  }

                  interpretive_units[l][i][j][k] = (float) (sum_RF / size_RF); 
                  //printf("selective_tuning:level %d: %d %d size %d I value: %4.1f\n",l, centre_x,centre_y,size_RF,interpretive_units[l][i][j][k]);  

               }
            }
         }
      }
   }
 

   for (l=1; l<number_of_levels; l++) {  // only compute gating units on levels 2 to L

      width = resolution[l]  / resolution[number_of_levels-1];  
      height = width;  

      if (false && debug) {
         printf("selective_tuning: Pass 3 ... computing gating units on level %d (%d %d)\n",l,width,height);  
      }

      maximum_value = 0;

      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {

            for (p = 0; p < pyramid_level_reduction; p++) {
               for (q = 0; q < pyramid_level_reduction; q++) {

                  for (r = minRFSize; r <= maxRFSize; r++) {

                     if (rectangularRF) {
                        sminRFSize = minRFSize;
                        smaxRFSize = maxRFSize;
                        number_of_RF_sides = maxRFSize - minRFSize + 1;
                     }
                     else {
                        sminRFSize = r;
                        smaxRFSize = r;
                        number_of_RF_sides = 1;
                     }

                     for (s = sminRFSize; s <= smaxRFSize; s++) {

                        k = selective_tuning_forward_mapping(p,q,r,s,pyramid_level_reduction,number_of_receptive_fields,number_of_RF_sides,minRFSize,sminRFSize);

                        scale_normalization_factor = (alpha + (float) 1.0) / (alpha + (float) pow(beta, (float)(-(sqrt((float)(r*s))))));  
 
                        ii = ((i * resolution[l-1]) / resolution[l]) + p;
                        jj = ((j * resolution[l-1]) / resolution[l]) + q;
                        kk =   (s-sminRFSize) * (number_of_receptive_fields / number_of_RF_sides) + (r-minRFSize);

                        gating_units[l][i][j][k] =  bias_units[l-1][ii][jj][kk] * 
                                                    (float) scale_normalization_factor * 
                                                    interpretive_units[l-1][ii][jj][kk];
                        if (debug) {
                           if (gating_units[l][i][j][k] > maximum_value) {
                              maximum_value = gating_units[l][i][j][k];
                              //printf("selective_tuning: RF (%d, %d): SN %6.4f\n",r,s,scale_normalization_factor);  
                              //printf("l %d i %d, j %d, k %d RF (%d, %d); GU %4.1f SN %4.1f IU %4.1f\n",l, i, j, k, r, s, gating_units[l][i][j][k],  scale_normalization_factor, interpretive_units[l-1][ii][jj][kk]);  
                     
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }
   
 
     
   // Pass 4: Perform the WTA competitions and propagate the gating control units down the pyramid from level L to level 1 
   // ===================================================================================================================

   // first, set the gating control values to 1 on level L

   if (false && debug) {
      printf("selective_tuning: Pass 4 ... init gating control units to 1 on level %d \n",number_of_levels-1);  
   }

   // this is restricted to just the one assembly which corresponds to the global WTA 

       
   for (p = 0; p < number_of_receptive_fields; p++) {
      gating_control_units[number_of_levels-1][0][0][p] = (float) 1.0;  
   }

    
   for (l=number_of_levels-1; l>0; l--) {

      // now, start the WTA competition

      width = resolution[l]  / resolution[number_of_levels-1];  
      height = width;  

      if (rectangularRF) {
         number_of_RF_sides = maxRFSize - minRFSize + 1;
      }
      else {
         number_of_RF_sides = 1;
      }           

      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {

            iteration = 0;

            // compute the maximum value and determine whether the image is uniform, 
            // in which case it's not possible to run a WTA

            max_p = 0;
            uniform_image = true;
               
            maximum_value = 0;
 
            for (p = 0; p < number_of_gating_units; p++) {

               if (gating_units[l][i][j][p] > maximum_value) {
                  maximum_value = gating_units[l][i][j][p];
               }
               if (gating_units[l][i][j][p] != gating_units[l][i][j][0]) {
                  uniform_image = false;
               }
            } 


            if (uniform_image) {
               printf("selective_tuning: ALL GATING UNITS ARE THE SAME\n");
            }

            // compute threshold theta

            theta = (float) ((maximum_value * interpretive_unit_weights[l][0]) / (pow((double)2,(double)gamma) + 1));
 
            if (debug) {
               //printf("selective_tuning: Pass 4 ... Level %d; max %4.1f, gamma %d theta %f\n", l,maximum_value,gamma, theta);  
            }
 

            // for each assembly [i][j]

            if (debug) {
               for (p = 0; p < number_of_gating_units; p++) {
                  if (interpretive_unit_weights[l][p] * gating_units[l][i][j][p] != 0.0) {
                     //printf("selective_tuning: Pass 4 ... WTA  %d %d %d %d theta %6.4f iteration %d \n",l,i,j,p,theta,iteration);  
                     //printf("selective_tuning:            old weighted gating %6.4f\n",interpretive_unit_weights[l][p] * gating_units[l][i][j][p]);  
                  }
               }
            }

            do {                         
               iteration++;

               for (p = 0; p < number_of_gating_units; p++) {

                  // for each gating unit

                  sum_delta = 0.0;
                  for (r = 0; r < number_of_gating_units; r++) {
                     if (p!=r) {
                        delta = (interpretive_unit_weights[l][r] * gating_units[l][i][j][r]) - 
                                (interpretive_unit_weights[l][p] * gating_units[l][i][j][p]) ;
                        if (delta > theta) {
                           sum_delta += delta;
                        }
                        if (debug) {
                           //printf("selective_tuning: Pass 4 ... maximum %f,  gamma %d, theta %f, delta %f\n",maximum_value,  gamma, theta, delta);  
                        }
                     }
                  }

                  gating_unit_difference = gating_units[l][i][j][p] - sum_delta;
                  
                  if (gating_unit_difference < 0) {
                     gating_unit_difference = 0;
                  }

                  // find out which RF we are working on

                  selective_tuning_inverse_mapping(p,&p_prime,&q_prime,&r_prime,&s_prime,pyramid_level_reduction,number_of_receptive_fields,number_of_RF_sides,minRFSize,sminRFSize);

                  new_gating_units[l][i][j][p] = gating_control_units[l][i][j][(s_prime * (number_of_receptive_fields / number_of_RF_sides) ) + r_prime] * (float) gating_unit_difference; 

                  if (debug) {
                     if (new_gating_units[l][i][j][p] != 0) {
                        //printf("selective_tuning: Pass 4 ... %d %d %d %d iteration %d\n",l,i,j,p,iteration);  
                        //printf("selective_tuning: Pass 4 ... sum %6.2f, GCU %3.1f GU diff %6.2f, GU new %6.2f\n",sum_delta,gating_control_units[l][i][j][r_prime*s_prime],gating_unit_difference,new_gating_units[l][i][j][p]);  

                     }
                  }
               }

               // now copy the new gating unit values back to the originals for the next iteration

               for (p = 0; p < number_of_gating_units; p++) {
                  if (debug) { 
                     if (new_gating_units[l][i][j][p] > 0) {
                        //printf("selective_tuning:            WTA  %d %d %d %d iteration %d \n",l,i,j,p,iteration);  
                        //printf("selective_tuning:            %4.1f -> %4.1f\n",gating_units[l][i][j][p],new_gating_units[l][i][j][p]);  
                  
                     }
                  }
                  gating_units[l][i][j][p] = new_gating_units[l][i][j][p];
               }

               // now check to see if convergence has been reached
               // check that all gating units are either zero or
               // greater than theta and the difference between these is less than theta

               finished = true;
              
               for (p = 0; p < number_of_gating_units; p++) {
                  if (gating_units[l][i][j][p] > 0) {
                     if (gating_units[l][i][j][p] < theta) {
                        finished = false;
                     }
                     else {
                        for (r = 0; r < number_of_gating_units; r++) {
                           if (gating_units[l][i][j][r] > 0) {
                              if (fabs(interpretive_unit_weights[l][r]*gating_units[l][i][j][r] - 
                                       interpretive_unit_weights[l][p]*gating_units[l][i][j][p]) > theta) {
                                 finished = false;
                              }
                           }
                        }
                     }
                  }
               }
            } while (!finished);
 

            if (debug) {
               found = false;
               for (p = 0; p < number_of_gating_units; p++) {
                  if (interpretive_unit_weights[l][p] * gating_units[l][i][j][p] > 0.0) {
                     //printf("selective_tuning: WU l %d i %d j %d p %d new GU %6.4f\n",l,i,j,p,interpretive_unit_weights[l][p] * gating_units[l][i][j][p]);  
                     found = true;
                  }
               }
               if (!found) {
                  //printf("selective_tuning: NO WINNING UNIT %d %d %d \n",l,i,j);  
               }
            }
         } // j
      } // i

      // now propagate the gating unit values to determine the gating control values at the next level down

      width = resolution[l]  / resolution[number_of_levels-1];  
      height = width;  


      if (debug) {
         //printf("selective_tuning: Pass 4 ... propagating GCU from level %d (%d %d) to level %d\n",l,width,height,l-1);  
      }

      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {

            ii = (i * resolution[l-1]) / resolution[l];
            jj = (j * resolution[l-1]) / resolution[l];

            for (p = 0; p < number_of_gating_units; p++) {

               selective_tuning_inverse_mapping(p,&p_prime,&q_prime,&r_prime,&s_prime,pyramid_level_reduction,number_of_receptive_fields,number_of_RF_sides,minRFSize,sminRFSize);

               // set the gating control unit at level l-1
               
               /* we inhibit the restrictive propagation of control units */
               /* to allow the RF size to be tuned at the next level      */
               
               //if (gating_units[l][i][j][p] > 0) {
                  
                  gating_control_units[l-1][ii+p_prime][jj+q_prime][(s_prime * (number_of_receptive_fields / number_of_RF_sides) ) + r_prime] = (float) 1.0;
                  
                  /* if we are propagating the winning receptive field, then we must double the RF size for the next level */
                  /* however, this doesn't allow any tuning of the RF from level to level and you end up with a winning RF */
                  /* that is a power of two of the one that won at the top level; hardly satisfactory                      */
                  
                  //gating_control_units[l-1][ii+p_prime][jj+q_prime][((2*s_prime+minRFSize) * (number_of_receptive_fields / number_of_RF_sides) ) + (2*r_prime+minRFSize)] = (float) 1.0;
                  
                  if (debug) {
                     // printf("selective_tuning: Pass 4 ...  GU[%d][%d][%d][%d] RF (%d, %d)) = %f\n",l,i,j,p,r_prime+minRFSize,s_prime+minRFSize,gating_units[l][i][j][p]);  
                  }
               //}

               if (true && debug) {
                  if (gating_control_units[l-1][ii+p_prime][jj+q_prime][(s_prime * (number_of_receptive_fields / number_of_RF_sides) ) + r_prime] > 0) {
                     //printf("selective_tuning: Pass 4 ... p, q, r, s prime %d %d %d %d\n",p_prime,q_prime,r_prime,s_prime);  
                     //printf("selective_tuning: Pass 4 ... GCU[%d][%d][%d][%d]) = %f\n",l-1,ii+p_prime,jj+q_prime,(s_prime * (number_of_receptive_fields / number_of_RF_sides) ) + r_prime,gating_control_units[l-1][ii+p_prime][jj+q_prime][(s_prime * (number_of_receptive_fields / number_of_RF_sides) ) + r_prime]);  
                  }
               }  
            
            }
         }
      }
   }
  

   // There may be multiple winners at the end in distinct receptive fields that were propagated by
   // multiple levels on the descent ... we now find the one with the maximum response
   // in case of a tie, we choose the largest RF
 
   l = 1; 
   width = resolution[l]  / resolution[number_of_levels-1];  
   height = width;

   if (rectangularRF) {
      number_of_RF_sides = maxRFSize - minRFSize + 1;
   }
   else {
      number_of_RF_sides = 1;
   }

   if (rectangularRF) {
      number_of_RF_sides = maxRFSize - minRFSize + 1;
   }
   else {
      number_of_RF_sides = 1;
   }
   
   maximum_value = 0.0;
   max_l=0;
   max_i=0;
   max_j=0;
   max_p=0;
   max_RF_area = 0;

   for (i=0; i<width; i++) {
      for (j=0; j<height; j++) {
         for (p = 0; p < number_of_gating_units; p++) {
            if ((interpretive_unit_weights[l][p] * gating_units[l][i][j][p]) >= maximum_value) { // >= get biggest RF

               selective_tuning_inverse_mapping(p,&p_prime,&q_prime,&r_prime,&s_prime,pyramid_level_reduction,number_of_receptive_fields,number_of_RF_sides,minRFSize,sminRFSize);

               if (rectangularRF) {
                  sminRFSize = minRFSize;
               }
               else {
                  sminRFSize = r_prime+minRFSize;
               }

               if (((interpretive_unit_weights[l][p] * gating_units[l][i][j][p]) > maximum_value) ||
                   (((r_prime+minRFSize) * (s_prime+sminRFSize)) >= max_RF_area)) {

                  maximum_value = interpretive_unit_weights[l][p] * gating_units[l][i][j][p];
                  max_l=l;
                  max_i=i;
                  max_j=j;
                  max_p=p;
                  max_RF_area = (r_prime+minRFSize) * (s_prime+sminRFSize);
                      
                  //printf("selective_tuning: Pass 5 ...  GU[%d][%d][%d][%d]) = %f; RF = %d\n",max_l,max_i,max_j,max_p,gating_units[max_l][max_i][max_j][max_p], max_RF_area);  

               }
            }
         }
      }
   }
            
   if (maximum_value == 0) {
      *number_of_winning_units = 0;
	        
	  //if (debug) {
         printf("selective_tuning: NO WINNING UNIT\n");
     // }

   }
   else {
      l = 1;

      if (rectangularRF) {
         number_of_RF_sides = maxRFSize - minRFSize + 1;
      }
      else {
         number_of_RF_sides = 1;
      }

      ii = (max_i * resolution[l-1]) / resolution[l];
      jj = (max_j * resolution[l-1]) / resolution[l];
            
      selective_tuning_inverse_mapping(max_p,&p_prime,&q_prime,&r_prime,&s_prime,pyramid_level_reduction,number_of_receptive_fields,number_of_RF_sides,minRFSize,sminRFSize);

      if (rectangularRF) {
         sminRFSize = minRFSize;
      }
      else {
         sminRFSize = r_prime+minRFSize;
      }

      winning_units[0].x += ii+p_prime;
      winning_units[0].y += jj+q_prime;
      winning_units[0].value = image_values[0][winning_units[0].x][winning_units[0].y];
      winning_units[0].rf_x =  r_prime+minRFSize;
      winning_units[0].rf_y =  s_prime+sminRFSize;
 
      if (debug) {
         //printf("selective_tuning: Selectively Tuned unit i, j %d %d \n",max_i, max_j);  
         printf("selective_tuning: Final winning unit at (%d, %d): %6.4f - RF = %d %d\n",winning_units[0].x, winning_units[0].y,winning_units[0].value,r_prime+minRFSize,s_prime+sminRFSize);
      }
   }

   return 1;

}

int selective_tuning_forward_mapping(int p, int q, int r, int s, int pyramid_level_reduction, int number_of_receptive_fields, int number_of_RF_sides, int minRFSize, int sminRFSize) {

   int k;
   int temp;
   int s_prime, r_prime, q_prime, p_prime;
   bool debug;

   debug = false;
   
   k =   (s-sminRFSize) * pyramid_level_reduction * pyramid_level_reduction * (number_of_receptive_fields / number_of_RF_sides) 
       + (r-minRFSize) * pyramid_level_reduction * pyramid_level_reduction 
       + q * pyramid_level_reduction 
       + p;


   if (debug) {

      // check mapping 
                       
      temp = k;
      s_prime = temp / (pyramid_level_reduction * pyramid_level_reduction * (number_of_receptive_fields / number_of_RF_sides));
      temp    = temp % (pyramid_level_reduction * pyramid_level_reduction * (number_of_receptive_fields / number_of_RF_sides));
      r_prime = temp / (pyramid_level_reduction * pyramid_level_reduction);
      temp    = temp % (pyramid_level_reduction * pyramid_level_reduction);
      q_prime = temp / pyramid_level_reduction;
      p_prime = temp % pyramid_level_reduction;
              
      printf("selective_tuning_forward_mapping: p, q, r, s %d %d %d %d -> k %d -> %d %d %d %d\n",p,q,r,s,k,p_prime, q_prime, r_prime+minRFSize, s_prime+sminRFSize);  
                        
   }

   return k;
}

void selective_tuning_inverse_mapping(int k, int *p_prime, int *q_prime, int *r_prime, int *s_prime, int pyramid_level_reduction, int number_of_receptive_fields, int number_of_RF_sides, int minRFSize, int sminRFSize) {

   int temp;
   bool debug;

   debug = false;
   
   temp = k;
   *s_prime = temp / (pyramid_level_reduction * pyramid_level_reduction * (number_of_receptive_fields / number_of_RF_sides));
   temp    = temp % (pyramid_level_reduction * pyramid_level_reduction * (number_of_receptive_fields / number_of_RF_sides));
   *r_prime = temp / (pyramid_level_reduction * pyramid_level_reduction);
   temp    = temp % (pyramid_level_reduction * pyramid_level_reduction);
   *q_prime = temp / pyramid_level_reduction;
   *p_prime = temp % pyramid_level_reduction;

   if (debug) {

      // check mapping 
                                     
      printf("selective_tuning_inverse_mapping: k %d -> %d %d %d %d\n",k,*p_prime,*q_prime,*r_prime+minRFSize,*s_prime+sminRFSize);  
                        
   }

}


/****************************************************************
* 
*  Routine Name: optical_flow
* 
*      Purpose: Compute the instantaneous optical flow between two images using the cross power spectrum in 
*            an apodized window at a sample of points.
*            If images are fused (e.g. using vergence control) this routine can also be used to compute the stereo disparity
*                        
*      Input: image1           - pointer to DVimage (INT); either image at t0 or left image 
*             image2           - pointer to DVimage (INT); either image at t1 or right image
*             window_size      - int parameter  
*             sampling_period  - int parameter      
*             sigma            - float parameter; standard_deviation of Gaussian      
*             x1ROI, y1ROI, 
*             x2ROI, y2ROI     - coordinates of top left and bottom right of region of interest
*
*      Output: flow_magnitude  - pointer to DVimage (FLOAT) with flow magnitude values
*              flow_phase      - pointer to DVimage (FLOAT) with flow phase values
*                                NB it is assumed that these image exists and is of the same dimensions
*                                as the inputs
*
* Written By: David Vernon
* Date: July 10, 2006
*
* Modifications: Forced non-zero results to facilitate subsequent interpolation                               
*                DV 30/03/2011
*
*                Added region of interest processing 
*                DV 05/04/2011
*
*                Extended padding from window_size/2 to window_size/2; flow is now computed right to the edges of the image
*                DV 07/04/2011
*
*                Only register significant maxima corresponding to non-zero velocity.                             
*                This suppresses flow around occluding boundaries and in regions which are relatively homogeneous 
*                (even in the presence of noise)   
*                DV 23/04/2011
*
*                Only register flow values greater than 1 pixel ... this avoids occasional detection of noisy flows in static scenes 
*                DV 05/01/2011 (Munich)
* 
****************************************************************/
 
void optical_flow (DVimage *image1, DVimage *image2, int window_size, int sampling_period, float sigma, 
                   int x1ROI, int y1ROI, int x2ROI, int y2ROI,
                   DVimage *flow_magnitude, DVimage *flow_phase)
{

   int i, j;
   int width, height, depth; 

   float temp;
   unsigned char pixel_value;
   float gaussian_value;
   int p, q, r, s;
   char debug, dump_debug_image;
    
   DVimage *padded_image1 = NULL;
   DVimage *padded_image2 = NULL;
   DVimage *window1       = NULL;
   DVimage *window2       = NULL;
   DVimage *cps           = NULL;
   DVimage *enhanced_cps  = NULL;
   DVimage *gaussian      = NULL;
  
   int filter_radius = 1;
   int number_of_maxima = 2;
   int non_maxima_suppression_radius = 1;
   maxima_data_type maxima[10]; 
	     

   /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("optical_flow: debug on \n");  
   if (debug) printf("window_size = %d; sampling_period = %d; standard_deviation = %f\n",window_size, sampling_period, sigma);  
 
   if (image1 != NULL && image2 != NULL) {

      /* create working images: windows and processed windows */

      image1->get_size(&width,&height);
      depth = image1->get_image_mode();

      padded_image1 = new DVimage(width+window_size,height+window_size,GREYSCALE_IMAGE,NULL,NULL,DVINT);
      padded_image2 = new DVimage(width+window_size,height+window_size,GREYSCALE_IMAGE,NULL,NULL,DVINT);
      window1       = new DVimage(window_size,window_size,GREYSCALE_IMAGE,NULL,NULL,DVINT);
      window2       = new DVimage(window_size,window_size,GREYSCALE_IMAGE,NULL,NULL,DVINT);
      cps           = new DVimage(window_size,window_size,GREYSCALE_IMAGE,NULL,NULL,DVFLOAT);
      enhanced_cps  = new DVimage(window_size,window_size,GREYSCALE_IMAGE,NULL,NULL,DVFLOAT);
      gaussian      = new DVimage(window_size,window_size,GREYSCALE_IMAGE,NULL,NULL,DVFLOAT);
 
      /* Generate a normalized 2D circular Gaussian image                   */
      /* with size n pixels, centred at pixel n/2, and centre value of 1        */
      /* sigma is the standard deviation of the Gaussian function             */

      for (i=0; i<window_size; i++) {
         for (j=0; j<window_size; j++) {

            // gaussian_value = ( exp(-( (i-window_size/2)*(i-window_size/2) )/(2*sigma*sigma)) / (sigma*sqrt(2*3.14159))  ) *  
            //                  ( exp(-( (j-window_size/2)*(j-window_size/2) )/(2*sigma*sigma)) / (sigma*sqrt(2*3.14159))  ); 

            gaussian_value = (float)(( exp(-( (i-window_size/2)*(i-window_size/2) )/(2*sigma*sigma))  ) *  
                                     ( exp(-( (j-window_size/2)*(j-window_size/2) )/(2*sigma*sigma))  ) );   // maximum value = 1

	        gaussian->put_pixel(i, j, gaussian_value); 
         }
      }

      //dump_float_image(gaussian->fdata,window_size,window_size);

 
      /* initialize the flow field to zero */

      flow_magnitude->initialize();
      flow_phase->initialize();

     
      // printf("width, height, depth = %d %d %d \n", width, height, depth);

      /* copy the input image to a larger image, padding with zeros                           */
      /* so that we can estimate the flow/disparity at points closer to the edge of the image */
 
      padded_image1->initialize();
      padded_image2->initialize();

      for (i = x1ROI; i < x2ROI; i++) {
         for (j = y1ROI; j < y2ROI; j++) {

            image1->get_pixel(i,j,&pixel_value, 0);

			if (depth == COLOUR_IMAGE) {              // convert to grey-scale if necessary
                temp = pixel_value;
  			    image1->get_pixel(i,j,&pixel_value, 1);
			    temp += pixel_value;
			    image1->get_pixel(i,j,&pixel_value, 2);
			    temp += pixel_value;
			    temp = temp / 3;
			    pixel_value = (unsigned char) temp;	
			}
            //padded_image1->put_pixel(i+window_size/4,j+window_size/4, (unsigned char) pixel_value);
            padded_image1->put_pixel(i+window_size/2,j+window_size/2, (unsigned char) pixel_value);
				      
            image2->get_pixel(i,j,&pixel_value, 0);

			if (depth == COLOUR_IMAGE) {
			   temp = pixel_value;
               image2->get_pixel(i,j,&pixel_value, 1);
			   temp += pixel_value;
			   image2->get_pixel(i,j,&pixel_value, 2);
			   temp += pixel_value;
			   temp = temp / 3;
			   pixel_value = (unsigned char) temp;	
			}	
            //padded_image2->put_pixel(i+window_size/4,j+window_size/4, (unsigned char) pixel_value);
            padded_image2->put_pixel(i+window_size/2,j+window_size/2, (unsigned char) pixel_value);
	
         }
      }

      //for (i = x1ROI; i < x2ROI - window_size/2; i+=sampling_period) {
      for (i = x1ROI; i < x2ROI; i+=sampling_period) {

      if (debug) 
         printf("optical flow: processing row %d of %d\n", i,width-window_size);

         //for (j = y1ROI; j < y2ROI - window_size/2; j+=sampling_period) {
         for (j = y1ROI; j < y2ROI; j+=sampling_period) {

            for (p=0; p<window_size; p++) {
               for (q=0; q<window_size; q++) {

			      gaussian->get_pixel(p,q,&gaussian_value);

			      padded_image1->get_pixel(i+p,j+q,&pixel_value, 0);
			      temp = (float) pixel_value * gaussian_value;
                  window1->put_pixel(p,q, (unsigned char) temp);

				  				  
			      padded_image2->get_pixel(i+p,j+q,&pixel_value, 0);		  
			      temp = pixel_value * gaussian_value;
                  window2->put_pixel(p,q, (unsigned char) temp);
               }
		    }
       
            cross_power_spectrum (window2, window1, cps); // cps must exist, type FLOAT

            /* Enhancing the maxima does not significantly improve the quality of the optical flow computation */
            /* so we skip it                                                                                   */
            /*
            filter_radius = 1;	 
  		    enhance_local_maxima (cps, filter_radius, enhanced_cps); 	  
            */
            
            number_of_maxima = 2;  // we only need to find the principal shift in the window
            non_maxima_suppression_radius = 3;
            //find_maxima (enhanced_cps, number_of_maxima, non_maxima_suppression_radius, maxima);  
		    find_maxima (cps, number_of_maxima, non_maxima_suppression_radius, maxima);  
		    
            // printf("%d %d = %f, %d %d = %f\n",maxima[0].x ,maxima[0].y, maxima[0].value, maxima[1].x ,maxima[1].y, maxima[1].value);
  			 
            /*** only register significant maximimum for non-zero velocity   ***/
            
            p = maxima[0].x  - window_size/2;          
            q = maxima[0].y  - window_size/2; 
            r = maxima[1].x  - window_size/2;          
            s = maxima[1].y  - window_size/2; 

            if ((p==0) && (q==0)) {

               /* global maximum corresponds to zero velocity; this is either a static background     */
               /* or a region which is relatively uniform and so may have multiple maxima,            */
               /* one which corresponds to zero velocity.                                             */
               /* Such cases are very unreliable so we filter them out here                           */ 

               
            }
            else if ((r==0) && (s==0)) {

               /* the second maximum corresponds to zero velocity; this is probably a static background   */
               /* or a region which is relatively uniform and so may have multiple maxima,                */
               /* one which corresponds to zero velocity.                                                 */
               /* so we accept the first maximum as significant if it is 2 times greater than the second  */

               if (maxima[0].value > (2 * maxima[1].value)) {

			      /* NB register flow values greater than 1 pixel ... this avoids occasional detection of noisy flows in static scenes */ 

		          temp = (float) sqrt((float)( p*p + q*q ));   
                  if (temp > 1.0) flow_magnitude->put_pixel(i,j,(float) temp); 
 
                  temp = (float) atan2((float)q, (float)p);
                  if (temp == 0)  temp = (float) 0.000001; // ensure non-zero result to facilitate possible subsequent interpolation
                  flow_phase->put_pixel(i,j,(float) temp); // put value in centre of window, offset by padding

                  if (debug) 
                     printf("optical flow: static background corresponding to second maximum\n");
               }
            }
            else {

               /* the remaining possibility is that both maxima correspond to non-zero velocities         */
               /* (the fourth permutation that both maxima correspond to zero velocity is not possible)   */
               /* so we accept the first maximum as significant if it is 2 times greater than the second  */

               if (maxima[0].value > 2 * maxima[1].value) {
         
				  /* NB register flow values greater than 1 pixel ... this avoids occasional detection of noisy flows in static scenes */ 

		          temp = (float) sqrt((float)( p*p + q*q ));   
                  if (temp > 1.0) flow_magnitude->put_pixel(i,j,(float) temp); 

		          temp = (float) atan2((float)q, (float)p);
                  if (temp == 0)  temp = (float) 0.000001; // ensure non-zero result to facilitate possible subsequent interpolation
                  flow_phase->put_pixel(i,j,(float) temp); // put value in centre of window, offset by padding

                  if (debug) 
                     printf("optical flow: moving foreground and background; significant first maximum\n");

               }
            }
         }
      }
   }

     
   /* free up all allocated space, i.e., release local image space */

   delete padded_image1;
   delete padded_image2;
   delete window1;
   delete window2;
   delete cps;
   delete enhanced_cps;
   delete gaussian;

   if (debug) printf("Leaving optical_flow\n");

   return;
   
}






/************************************************************************************************

Motion Segmentation Code
 
*************************************************************************************************/


/****************************************************************
* 
*  Routine Name: image_difference
* 
*      Purpose: Computer the instantaneous motion (difference) between two images  
*            and threshold the results
*            If the input images are COLOUR_IMAGES then we only use the RED channel
*
*            NB we don't perform any data validation (e.g. check that the images are the same size
*            
*                        
*       Input: image1           - pointer to DVimage; either image at t0 or left image 
*            image2           - pointer to DVimage; either image at t1 or right image
*            threshold         - int parameter  
*
*      Output: output_image      - pointer to DVimage with thresholded image difference
*
*   Written By: David Vernon
*        Date: July 22, 2006
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications:
*
*
****************************************************************/
 
void image_difference (DVimage *image1, DVimage *image2, int threshold,
               DVimage *output_image)
{
   int i, j;
   int width, height, depth; 

   unsigned char pixel_value1;
   unsigned char pixel_value2;
   char debug, dump_debug_image;
    
   DVimage *window1     = NULL;
   DVimage *window2     = NULL;
   
 
 
   /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("image_difference: debug on \n");  
   if (debug) printf("threshold = %d\n",threshold);  
   
  
  
   if (image1 != NULL && image2 != NULL && output_image != NULL) {

     /* input data is intensity image */

     image1->get_size(&width,&height);
     depth = output_image->get_image_mode();
     
     if (debug) printf("width, height, depth = %d %d %d \n", width, height, depth);

     for (i=0; i< width; i++) {
       for (j=0; j< height; j++) {
 
			image1->get_pixel(i,j,&pixel_value1, 0);
			image2->get_pixel(i,j,&pixel_value2, 0);
			
			if(abs(pixel_value1 - pixel_value2) > threshold) {
            output_image->put_pixel(i,j, (unsigned char) 255,0);
			   
			   if (depth == COLOUR_IMAGE)  {
			     output_image->put_pixel(i,j, (unsigned char) 255,1);
			     output_image->put_pixel(i,j, (unsigned char) 255,2);
			   }
			}
			else {
            output_image->put_pixel(i,j, (unsigned char) 0,0);
			   
			   if (depth == COLOUR_IMAGE)  {
			     output_image->put_pixel(i,j, (unsigned char) 0,1);
			     output_image->put_pixel(i,j, (unsigned char) 0,2);
			   }
			}
		 }
	  }
   }

 
   if (debug) printf("Leaving image_difference\n");

   return;
   
}



/************************************************************************************************

Colour Segmentation Code
 
*************************************************************************************************/




// -----------------------------------------------------------------------------------------------
// rgb2hsi
//
// convert an RGB triple to a HSI triple
// -----------------------------------------------------------------------------------------------


void rgb2hsi_old(unsigned char r, unsigned char g, unsigned char b, float *hue, float *saturation, float *intensity){

	int min =256;

   //  0 <= hue <= 2 pi
   //  0 <= saturation <= 1

	*intensity = (float) (r+g+b)/3;

	if ( r==g && g==b)
		*hue=-1; // degenerate case: no hue
   else {
		*hue= (float) acos ( (double) (( ((r-g)+(r-b))/2 ) / sqrt( (double) ((r-g)*(r-g) + (r-b)*(g-b)))  ));

		if ( g <= b)
			*hue= (float)(2*3.14159) - *hue;
   }
 


	if ( r < min )
		min = r;

	if ( g < min )
		min = g;

	if( b < min )
		min = b;

	if (*intensity != 0)
		*saturation = 1 - ( ( 3 * min ) / (float)(r+g+b) );
   else
     *saturation = 0;

   //printf("rgb2hsi: (%d, %d, %d) -> (%f, %f, %f)\n", r, g, b, *hue, *saturation, *intensity);

}


// -----------------------------------------------------------------------------------------------
// rgb2hsi
//
// convert an RGB triple to a HSI triple
//
// The transform is based on "The Taming of the Hue, Saturation and Brightness Colour Space", Allan Hanbury, Proc. CVWW, [Hanbury02]
// 
// -----------------------------------------------------------------------------------------------


void rgb2hsi(unsigned char red, unsigned char green, unsigned char blue, float *hue, float *saturation, float *intensity){

	double y, h, h_star, c, c1, c2,  s, r, g, b; 

   
   int min =256;

   //  0 <= hue <= 2 pi
   //  0 <= saturation <= 1

   r = (float) red   / 256;
   g = (float) green / 256;
   b = (float) blue  / 256;

   y  = 0.2125 * r + 0.7154 * g + 0.0721 * b;
   c1 =          r - 0.5    * g - 0.5    * b;
   c2 =            - 0.8660 * g + 0.8660 * b;


   // chroma c: [0,1]

   c = sqrt(c1*c1 + c2*c2);


   // hue h: [0,360]

   if (c == 0) { // h and s are undefined
      *hue        = (float) -1;
      *saturation = (float) -1;
   }
   else {
      if(c2 <= 0) {
         h = acos (c1/c);
      }
      else {
         h = 2*3.14159  - acos (c1/c);
      }

      h = 360 * (h / (2 * 3.14159)); // convert to degrees


      // saturation: [0,1]

      h_star =  (int) h - (int) (60 * (  ((int) h) / 60));  // convert to interval 0,60


      s = (2 * c * sin( 2 * 3.14159 * ((120 - h_star) / 360.0))) / 1.73205;


      //*hue        = (float)  ((h / 360) * 2 * 3.14159); // convert to radians ... for the moment anyway
      *hue        = (float)  h;  
      *saturation = (float)  s;
   }

 	*intensity  = (float)  (r+g+b)/3;

  // printf("rgb2hsi: (%d, %d, %d) -> (%3.1f, %3.1f, %3.1f)\n", red, green, blue, *hue, *saturation, *intensity);

}



/****************************************************************
* 
*  Routine Name: colour_segmentation  
* 
*      Purpose: Segment an image based on the hue and saturation values of each pixel
*                        
*       Input: input_image     - pointer to DVimage (RGB) 
*            hue           - float parameters specifying the required hue and saturation value
*            saturation      
*            hue_range      - float parameters specifying tolerance on hue
*            saturation_range   and saturation value
*                           
*
*      Output: output_image   - pointer to DV colour image with the segmented data
*                          NB it is assumed that this image exists and is of the same dimensions
*                          as the inputs
*
*   Written By: David Vernon
*        Date: December 18, 2005
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications:
*
*
****************************************************************/
 

void colour_segmentation (DVimage *input_image, float hue, float saturation, float hue_range, float saturation_range,
                    DVimage *output_image)
{
   int width, height, depth; 
   unsigned char r, g, b;
   float h = 0;
   float s = 0;
   float i = 0;
   int p, q;
   char debug, dump_debug_image;
  

   /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("colour_segmentation: debug on \n");  
   if (debug) printf("hue & saturation = (%f, %f); range = (%f, %f) \n", hue, saturation,hue_range,saturation_range);  
   
 
   if (input_image != NULL) {

	   /* input data is intensity image */

      input_image->get_size(&width,&height);
      depth = input_image->get_image_mode();
     
      // printf("width, height, depth = %d %d %d \n", width, height, depth);

 
      if (depth == COLOUR_IMAGE)  {
 
        for (p=0; p<width; p++) {
          for (q=0; q<height;q++) {


            //input_image->get_pixel(p,q,&r,0);
            //input_image->get_pixel(p,q,&g,1);
            //input_image->get_pixel(p,q,&b,2);
 
            // using the pixel macro version takes approx. 70% of the time of the get_pixel method call

            r = pixel(input_image->idata, width, p, q, 0);
            g = pixel(input_image->idata, width, p, q, 1);
            b = pixel(input_image->idata, width, p, q, 2);
         
            rgb2hsi(r,g,b,&h, &s, &i);

            if (   
                  (
                     (
                        (h >= hue - hue_range      ) && (h <= hue + hue_range      )
                     ) 
                     
                     ||

                     (
                        (h >= hue+360 - hue_range) && (h <= hue+360 + hue_range)
                     )

                     ||

                     (
                        (h >= hue-360 - hue_range) && (h <= hue-360 + hue_range)
                     )
                     
                  ) 
                  &&
                  (
                     (s >= (saturation - saturation_range)) && (s <= (saturation + saturation_range))
                  )
               ) 
               {

               //output_image->put_pixel(p,q,r,0);  
               //output_image->put_pixel(p,q,g,1);  
               //output_image->put_pixel(p,q,b,2);  

               pixel(output_image->idata, width, p, q, 0) = r;
               pixel(output_image->idata, width, p, q, 1) = g;
               pixel(output_image->idata, width, p, q, 2) = b;

            }
            else {
               //output_image->put_pixel(p,q,(unsigned char)0,0);  
               //output_image->put_pixel(p,q,(unsigned char)0,1);  
               //output_image->put_pixel(p,q,(unsigned char)0,2);  
                             
               pixel(output_image->idata, width, p, q, 0) = 0;
               pixel(output_image->idata, width, p, q, 1) = 0;
               pixel(output_image->idata, width, p, q, 2) = 0;
            }
          }  
        }
      }
   }
     

   if (debug) printf("Leaving colour_segmentation\n");

   return;
   
}



/****************************************************************
* 
*  Routine Name: colour_histogram  
* 
*  Compute a 2-D hue-saturation historgram
*                        
*  Input: input_image     - pointer to DVimage (RGB) 
*
*  Output: hs_histogram   - pointer to a hue-saturation object
*                           NB it is assumed that object exists 
*
*  Written By: David Vernon
*  Date:       17 September 2006
*
*
****************************************************************/
 

void colour_histogram (DVimage *input_image, DVhs_histogram *hs)
{
   int width, height, depth; 
   unsigned char r, g, b;
   float h = 0;
   float s = 0;
   float i = 0;
   int hue_dimension;
   int saturation_dimension;
   int hue;
   int saturation;
   int p, q;
   char debug, dump_debug_image;
  

   /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("colour_histogram: debug on \n");  
   
   if (hs != NULL)
      hs->initialize();
 
   if (input_image != NULL) {

	   /* input data is intensity image */

      input_image->get_size(&width,&height);
      depth = input_image->get_image_mode();

      hs->get_dimensions(&hue_dimension, &saturation_dimension);

     
      // printf("width, height, depth = %d %d %d \n", width, height, depth);

 
      if (depth == COLOUR_IMAGE)  {
 
         for (p=0; p<width; p++) {
            for (q=0; q<height;q++) {


               //input_image->get_pixel(p,q,&r,0);
               //input_image->get_pixel(p,q,&g,1);
               //input_image->get_pixel(p,q,&b,2);
 
               // using the pixel macro version takes approx. 70% of the time of the get_pixel method call

               r = pixel(input_image->idata, width, p, q, 0);
               g = pixel(input_image->idata, width, p, q, 1);
               b = pixel(input_image->idata, width, p, q, 2);

               if (!(r==0 && g==0 && b==0)) {
         
                  rgb2hsi(r,g,b,&h, &s, &i);  // h: [0,360]; s: [0,1]

                  hue = (int) ((float) hue_dimension * (h / 360));
                  saturation = (int) ((float) saturation_dimension * s); 

                  hs->increment_bin(hue,saturation);
               }
            }  
         }
      }
   }
     

   if (debug) printf("Leaving colour_histogram\n");

   return;
   
}


/****************************************************************
* 
*  Routine Name: dilation  
* 
*      Purpose: Perform a binary dilation of an intensity image
*            if a pixel value is non-zero then all of it neighbours within the structuring element radius
*            are assigned that pixel value
*                        
*       Input: input_image    - pointer to DVimage (either COLOUR_IMAGE or GREYSCALE_IMAGE) 
*            radius        - int value specifying radius of the structuring element                     
*
*      Output: output_image   - pointer to dilated DVimage 
*                          NB it is assumed that this image exists and is of the same dimensions
*                          as the inputs
*
*   Written By: David Vernon
*        Date: December 20, 2005
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications:
*
*
****************************************************************/
 

void dilation (DVimage *input_image, int radius, DVimage *output_image)
{
   int width, height, depth; 
   unsigned char r, g, b;
   int i, x, y;
   int p, q;
   int p_limit, q_limit;


   char debug, dump_debug_image;


   
//#define PIX(f,width,i,j)   (*((f) + ( (j) * (width) )  + (i) ))

   /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("dilation: debug on \n");  
 
   if (input_image != NULL) {

	   /* input data is intensity image */
      input_image->get_size(&width,&height);
      depth = input_image->get_image_mode();
     
      
     // printf("Depth = %d\n", depth);

      output_image->initialize();

      if (depth == COLOUR_IMAGE)  {

        p_limit = width-radius;
        q_limit = height-radius;
        for (p=radius; p<p_limit; p++) {         // exclude border pixels from processing
          for (q=radius; q<q_limit;q++) {
 
            // using the pixel macro version takes approx. 70% of the time of the get_pixel method call

            r = pixel(input_image->idata, width, p, q, 0);
            g = pixel(input_image->idata, width, p, q, 1);
            b = pixel(input_image->idata, width, p, q, 2);

    
            if (r != 0 || g!=0 || b!=0) { // non-zero pixel

               for (x=-radius; x<=radius; x++) {
                 for (y=-radius; y<=radius; y++) {

                   pixel(output_image->idata, width, p+x, q+y, 0) = r;
                   pixel(output_image->idata, width, p+x, q+y, 1) = g;
                   pixel(output_image->idata, width, p+x, q+y, 2) = b;
                 }
               }
            }
          }
        }
      }
      else  if (depth == GREYSCALE_IMAGE)  {

        p_limit = width-radius;
        q_limit = height-radius;

        for (p=radius; p<p_limit; p++) {         // exclude border pixels from processing
          for (q=radius; q<q_limit;q++) {
 
            // using the pixel macro version takes approx. 70% of the time of the get_pixel method call

            i = PIX(input_image->idata, width, p, q);
         
            if ( i > 0) { // non-zero pixel

               for (x=-radius; x<=radius; x++) {
                 for (y=-radius; y<=radius; y++) {

                   PIX(output_image->idata, width, p+x, q+y) = i;

                 }
               }
            }
          }
        }
      }
   }
     
   if (debug) printf("Leaving dilation\n");

   return;
   
}




/****************************************************************
* 
*  Routine Name: erosion  
* 
*      Purpose: Perform a binary erosion of an intensity image:
*            if a pixel and all its neighbours within the structuring element radius
*            are non-zero, assigned that pixel value to the output
*                        
*       Input: input_image    - pointer to DVimage (either COLOUR_IMAGE or GREYSCALE_IMAGE) 
*            radius        - int value specifying radius of the structuring element                     
*
*      Output: output_image   - pointer to eroded DVimage 
*                          NB it is assumed that this image exists and is of the same dimensions
*                          as the inputs
*
*   Written By: David Vernon
*        Date: December 20, 2005
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications:
*
*
****************************************************************/
 

void erosion (DVimage *input_image, int radius, DVimage *output_image)
{
   int width, height, depth; 
   unsigned char r, g, b, r1, g1, b1;
   int x, y;
   int copy;
   int p, q;
   int i, j;
   int p_limit, q_limit;


   char debug, dump_debug_image;


   /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("erosion: debug on \n");  
 
   if (input_image != NULL) {

	   /* input data is intensity image */
      input_image->get_size(&width,&height);
      depth = input_image->get_image_mode();
     
      
     // printf("Depth = %d\n", depth);

      output_image->initialize();

      if (depth == COLOUR_IMAGE)  {

        p_limit = width-radius;
        q_limit = height-radius;
        for (p=radius; p<p_limit; p++) {         // exclude border pixels from processing
          for (q=radius; q<q_limit;q++) {
 
            // using the pixel macro version takes approx. 70% of the time of the get_pixel method call

            r = pixel(input_image->idata, width, p, q, 0);
            g = pixel(input_image->idata, width, p, q, 1);
            b = pixel(input_image->idata, width, p, q, 2);

            if (r != 0 || g!=0 || b!=0) { // non-zero pixel

               copy = TRUE;
               for (x=-radius; x<=radius && copy; x++) {
                 for (y=-radius; y<=radius && copy; y++) {

                   r1 = pixel(input_image->idata, width, p+x, q+y, 0);
                   g1 = pixel(input_image->idata, width, p+x, q+y, 1);
                   b1 = pixel(input_image->idata, width, p+x, q+y, 2);

                   if (r1==0 && g1==0 && b1==0) { // found a background pixel; stop checking
                     copy = FALSE;
                   }
                 }
               }
               if (copy) {
                 pixel(output_image->idata, width, p, q, 0) = r;
                 pixel(output_image->idata, width, p, q, 1) = g;
                 pixel(output_image->idata, width, p, q, 2) = b;
               }
            }
          }
        }
      }
      else  if (depth == GREYSCALE_IMAGE)  {

        p_limit = width-radius;
        q_limit = height-radius;

        for (p=radius; p<p_limit; p++) {         // exclude border pixels from processing
          for (q=radius; q<q_limit;q++) {
 
            // using the pixel macro version takes approx. 70% of the time of the get_pixel method call

            i = PIX(input_image->idata, width, p, q);
         
            if (i != 0) { // non-zero pixel

               copy = TRUE;
               for (x=-radius; x<=radius && copy; x++) {
                 for (y=-radius; y<=radius && copy; y++) {

                   j = PIX(input_image->idata, width, p+x, q+y);

                   if (j==0) { // found a background pixel; stop checking
                     copy = FALSE;
                   }
                 }
               }
               if (copy) {
                 PIX(output_image->idata, width, p, q) = i;
               }
            }

          }
        }
      }
   }
     
   if (debug) printf("Leaving erosion\n");

   return;
   
}



/****************************************************************
* 
*  Routine Name: distance_from_background_transform
* 
*  Replace the pixel values of all non-zero pixels with the minimum distance of that pixel from a background (i.e. zero) pixel.
*  Border pixels are all set to zero.
*                        
*  Input: input_image    - pointer to DVimage (either COLOUR_IMAGE or GREYSCALE_IMAGE) 
* 
*  Output: output_image   - pointer to transformed DVimage 
*                           NB it is assumed that this image exists and is of the same dimensions
*                           as the inputs
*
*   Written By: David Vernon
*        Date: February 2, 2012
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications:
*
*
****************************************************************/
 

void distance_from_background_transform (DVimage *input_image, DVimage *output_image)
{
   int width, height, depth; 
   unsigned char r,  g,  b,  r1,  g1,  b1;
   float         rf, gf, bf, r1f, g1f, b1f;
   int x, y;
   int p, q;
   int radius;
   bool background_found;
   char debug;
  


   /* set debug flags */

   debug = FALSE;
 
   if (debug) printf("distance_from_background_transform: debug on \n");  
 
   if (input_image != NULL) {

	   /* input data is intensity image */
      input_image->get_size(&width,&height);
      depth = input_image->get_image_mode();
       
      if (debug)  printf("distance_from_background_transform: width, height, depth %d %d %d\n",width,height, depth);
 
      output_image->initialize();
 
      if (depth == COLOUR_IMAGE)  {

         for (p=1; p<width-1; p++) {         // ensure border pixels are zero
            for (q=1; q<height-1;q++) {
			      if (input_image->get_image_type() ==  DVINT) {
			         r1 = pixel(input_image->idata, width, p, q, 0);
                  g1 = pixel(input_image->idata, width, p, q, 1);
                  b1 = pixel(input_image->idata, width, p, q, 2);

                  pixel(output_image->idata, width, p, q, 0) = r1;
                  pixel(output_image->idata, width, p, q, 1) = g1;
                  pixel(output_image->idata, width, p, q, 2) = b1;
			      }
			      else {
			         r1f = pixel(input_image->fdata, width, p, q, 0);
                  g1f = pixel(input_image->fdata, width, p, q, 1);
                  b1f = pixel(input_image->fdata, width, p, q, 2);

                  pixel(output_image->fdata, width, p, q, 0) = r1f;
                  pixel(output_image->fdata, width, p, q, 1) = g1f;
                  pixel(output_image->fdata, width, p, q, 2) = b1f;
			      }
			   }
         }
	   }
      else {  // GREYSCALE
	      for (p=1; p<width-1; p++) {         // ensure border pixels are zero
            for (q=1; q<height-1;q++) {
			      if (input_image->get_image_type() ==  DVINT) {
			         r1 = PIX(input_image->idata, width, p, q);
                  PIX(output_image->idata, width, p, q) = r1;
			      }
			      else {
			         r1f = PIX(input_image->fdata, width, p, q);
                  PIX(output_image->fdata, width, p, q) = r1f;   //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! PIX for GREYSCALE
 			      }
			   }
         }
	   }


      // now compute transform
 
	   if (depth == COLOUR_IMAGE)  {

         for (p=1; p<width-1; p++) {         
            for (q=1; q<height-1;q++) {
               if (input_image->get_image_type() ==  DVINT) {
			         r = pixel(input_image->idata, width, p, q, 0);
                  g = pixel(input_image->idata, width, p, q, 1);
                  b = pixel(input_image->idata, width, p, q, 2);

		            if (r != 0 || g != 0 || b != 0) { // non-zero pixel

				         radius = 1;
				         background_found = false;

                     while (!background_found) {
				            for (x=-radius; x<=radius && !background_found; x++) {
				               for (y=-radius; y<=radius && !background_found; y++) {
                              if  (y*y + x*x <= radius*radius) {

						               r1 = pixel(input_image->idata, width, p+x, q+y, 0);
                                 g1 = pixel(input_image->idata, width, p+x, q+y, 1);
                                 b1 = pixel(input_image->idata, width, p+x, q+y, 2);

                                 if (r1==0 && g1==0 && b1==0) { // found a background pixel; stop checking
                                    background_found = true;
                                 }
                              }
                           }
 					         }
					         if (!background_found) {
						         radius++;
					         }
                     }

                     if (debug) printf("distance_from_background_transform: radius = %d\n",radius);

                     pixel(output_image->idata, width, p, q, 0) = radius;
                     pixel(output_image->idata, width, p, q, 1) = radius;
                     pixel(output_image->idata, width, p, q, 2) = radius;
			         }
			      }
               else {
			         rf = pixel(input_image->fdata, width, p, q, 0);
                  gf = pixel(input_image->fdata, width, p, q, 1);
                  bf = pixel(input_image->fdata, width, p, q, 2);

		            if (rf > 0.0 || gf > 0.0 || bf > 0.0) { // non-zero pixel

				         radius = 1;
				         background_found = false;

				         while (!background_found) {
				            for (x=-radius; x<=radius && !background_found; x++) {
				               for (y=-radius; y<=radius && !background_found; y++) {
                              if  (y*y + x*x <= radius*radius) {

						               r1f = pixel(input_image->fdata, width, p+x, q+y, 0);
                                 g1f = pixel(input_image->fdata, width, p+x, q+y, 1);
                                 b1f = pixel(input_image->fdata, width, p+x, q+y, 2);

                                 if (!(r1f+g1f+b1f > 0.0)) { // found a background pixel; stop checking
                                    background_found = true;
                                 }
                              }
                           }
					         }
					         if (!background_found) {
						         radius++;
					         }
                     }

                     pixel(output_image->fdata, width, p, q, 0) = (float) radius;
                     pixel(output_image->fdata, width, p, q, 1) = (float) radius;
                     pixel(output_image->fdata, width, p, q, 2) = (float) radius;
				                    
                     if (debug) printf("distance_from_background_transform: radius = %d\n",radius);

			         }
			      }
			   }
         }
      }
	   else {  // GREYSCALE
         for (p=1; p<width-1; p++) {         
            for (q=1; q<height-1;q++) {
			      if (input_image->get_image_type() ==  DVINT) {
			         r = PIX(input_image->idata, width, p, q);

		            if (r > 0) { // non-zero pixel

				         radius = 1;
				         background_found = false;

				         while (!background_found) {
				            for (x=-radius; x<=radius && !background_found; x++) {
				               for (y=-radius; y<=radius && !background_found; y++) {
                              if  (y*y + x*x <= radius*radius) {
                                 r1 = PIX(input_image->idata, width, p+x, q+y);
                                 if (r1==0) { // found a background pixel; stop checking
                                    background_found = true;
                                 }
                              }
                           }
					         }
					         if (!background_found) {
						         radius++;
					         }
                     }

                     PIX(output_image->idata, width, p, q) = radius;
			            if (debug) printf("distance_from_background_transform: radius = %d\n",radius);

                  }
			      }
               else {
			         rf = PIX(input_image->fdata, width, p, q);

				      //if (debug) printf("distance_from_background_transform: rf = %d\n",rf);
		         
				      if (rf > 0.0) { // non-zero pixel

				         radius = 1;
				         background_found = false;

				         while (!background_found) {
				            for (x=-radius; x<=radius && !background_found; x++) {
				               for (y=-radius; y<=radius && !background_found; y++) {
                              if  ((y*y + x*x) <= (radius*radius)) {
                                 r1f = PIX(input_image->fdata, width, p+x, q+y);
                                 if (r1f == 0.0) { // found a background pixel; stop checking
                                    background_found = true;
                                 }
                              }
                           }
			               }
					         if (!background_found) {
						         radius++;
					         }
                     }

                     PIX(output_image->fdata, width, p, q) = (float) radius;
					      //printf("distance_from_background_transform: radius = %d\n",radius);

                  }
			      }
            }
         }
      }
   }
     
   if (debug) printf("Leaving distance_from_background_transform\n");

   return;
   
}

#ifdef DVR
/****************************************************************
* 
*  Routine Name: log_polar_transform - compute the Log-Polar transform
*            in both forward (Cartesian to Log-Polar) and backward (Log-Polar to Cartesian) directions
*            
*    Input: input     - pointer to DVimage 
*           direction - int, either CARTESIAN2LOGPOLAR or LOGPOLAR2CARTESIAN
*           overlap   - float, overlap of receptive fields.
*
*    Output: output   - pointer to DVimage  
*                     NB it is assumed that this image exists 
*                     The dimensions of the output log-polar / Cartesian image will be extracted directly from this image
*
*   Written By: David Vernon
*   Date: July 25, 2006
*
*   Amended to pass overlap as a parameter. DV September 20, 2009
*
*   Note: this function is just an interface to the University of Genoa Log-Polar routines
*   and made available by the RobotCub project (www.robotcub.org)
*
****************************************************************/

void log_polar_transform  (DVimage *input_image, DVimage *output_image, int direction, double overlap)
{

   // we make these static as we don't want to recompute the lookup tables for each image
   // if they have already been computed.

   static cart2LpPixel *c2lTable;
   static lp2CartPixel *l2cTable;
   static unsigned char *colorLP;
   static unsigned char *colorBRem;
   static unsigned char *buffer;

   static int    nEcc; 
   static int    nAng;  
   //static double overlap;
   static int    xSize, ySize;
   static double scaleFact;
   static double logIndex;
   static int    cartSize; 
   static char   path[] = "./";


   int   width_in,  height_in, depth_in; 
   int   width_out, height_out, depth_out;
   int   min_i, min_j, max_i, max_j;
   int   i, j;
   int   temp;
   unsigned char pixel_value;
   unsigned char *p;
   char   debug;

   
#define PIX(f,width,i,j)   (*((f) + ( (j) * (width) )  + (i) ))

   /* set debug flags */

   debug = FALSE;
   if (debug) printf("log_polar_transform: debug on \n");  
   
   if (input_image != NULL && output_image != NULL) { 
	   
      //overlap = 0.5;//1.00;

	  input_image->get_size(&width_in,&height_in);
	  depth_in = input_image->get_image_mode();

	  output_image->get_size(&width_out,&height_out);
	  depth_out = output_image->get_image_mode();


	  if (debug) printf("log_polar_transform: input width, height, depth = (%d, %d, %d); output width, height, depth = (%d, %d, %d) \n", width_in, height_in, depth_in, width_out, height_out, depth_out);  

      if (direction == CARTESIAN2LOGPOLAR) {
	 
		 // General procedure: copy square region from input to buffer, perform log-polar transform, and copy to output

		 // only compute the look-up tables once for a given input and output image size

		 if (!(xSize == _min(width_in,height_in) && nEcc == height_out && nAng == width_out && buffer != NULL && colorLP != NULL && c2lTable != NULL)) {
	      
		    if (debug) printf("log_polar_transform: computing lookup tables \n");  

			xSize = _min(width_in, height_in);
		    ySize = xSize;
	        nEcc = height_out;  // dimension of the log-polar image
            nAng = width_out;  
            logIndex = RCgetLogIndex (nAng);
            cartSize = xSize; 
		    scaleFact = RCcomputeScaleFactor (nEcc, nAng, xSize, ySize, overlap);

		    if (buffer   != NULL) free(buffer);
		    if (colorLP  != NULL) free(colorLP);
		    if (c2lTable != NULL) free(c2lTable);
			
			buffer   = (unsigned char *) malloc(sizeof(unsigned char) * xSize * ySize * 3);
            colorLP  =  (unsigned char *) malloc(sizeof(unsigned char) * nEcc * nAng * 3);
            c2lTable  = (cart2LpPixel  *) malloc(sizeof(cart2LpPixel)  * nEcc * nAng);

            RCbuildC2LMap (nEcc, nAng, xSize, ySize, overlap, scaleFact, RADIAL, path);  // Alternative: TANGENTIAL or ELLIPTICAL
            RCallocateC2LTable (c2lTable, nEcc, nAng, 0, path);
		 }


		 // now copy the input image and perform the transform

	     // THIS LOOP NEEDS TO BE OPTIMIZED 

		 if (width_in > height_in) {
	      min_i = (width_in - xSize)/2;
			min_j = 0;
			max_i = xSize + min_i;
			max_j = xSize;
		 }
		 else {
			min_j = (height_in - xSize)/2;
			min_i = 0;
			max_j = xSize + min_j;
			max_i = xSize;
		 }

		 p = buffer;

		 for (i = min_i; i<max_i; i++) {
			for (j = min_j; j<max_j; j++) {

 			   input_image->get_pixel(i,j,&pixel_value, 0);
			   *(p++) = pixel_value; 

			   if (depth_in == 3) {
  				  input_image->get_pixel(i,j,&pixel_value, 1);
				  *(p++) = pixel_value; 

				  input_image->get_pixel(i,j,&pixel_value, 2);
				  *(p++) = pixel_value; 
			   }
			   else {
				  *(p++) = pixel_value; // replicate R value to get a grey RGB image
			     *(p++) = pixel_value; 
			   }
			}
		 }


       RCgetLpImg (colorLP, buffer, c2lTable, nEcc * nAng, 0);


		 /* now copy colour log-polar to output */

       output_image->write(colorLP);

	  }
      else if (direction == LOGPOLAR2CARTESIAN) {

		 // General procedure: copy input to buffer, perform log-polar transform, and copy to square region in output

         // only compute the look-up tables once for a given input and output image size

		 if (!(xSize == _min(width_out,height_out) && nEcc == height_in && nAng == width_in && colorBRem != NULL && l2cTable != NULL)) {
	      
		    if (debug) printf("log_polar_transform: computing lookup tables \n");  

	        nEcc = height_in;   // dimension of the log-polar image
            nAng = width_in;   
	        xSize = _min(width_out, height_out);
		    ySize = xSize;
            logIndex = RCgetLogIndex (nAng);
            cartSize = xSize; 
		    scaleFact = RCcomputeScaleFactor (nEcc, nAng, xSize, ySize, overlap);

		    if (colorBRem  != NULL) free(colorBRem);
		    if (l2cTable != NULL)   free(l2cTable);

		    colorBRem = (unsigned char *) calloc(xSize * ySize * 3, sizeof(unsigned char));
            l2cTable  = (lp2CartPixel  *) malloc(sizeof(lp2CartPixel)  * xSize * ySize);

            if (depth_in != 3) {
			   printf("log_polar_transform: log-polar to Cartesian error - log-polar image must be a colour image\n");
			   return;
			}

            RCbuildL2CMap (nEcc, nAng, xSize, ySize, overlap, scaleFact, 0, 0, RADIAL, path);  // Alternative: TANGENTIAL or ELLIPTICAL
            RCallocateL2CTable (l2cTable, xSize, ySize, path);
		}

		// now perform the transform 

        RCgetCartImg (colorBRem, input_image->idata, l2cTable, cartSize * cartSize);

         // now copy result (i.e. square image) to output image
		 // THIS LOOP NEEDS TO BE OPTIMIZED 

		 output_image->initialize();

		 if (width_out > height_out) {
			min_i = (width_out - xSize)/2;
			min_j = 0;
			max_i = xSize + min_i;
			max_j = xSize;
		 }
		 else {
			min_j = (height_out - xSize)/2;
			min_i = 0;
			max_j = xSize + min_j;
			max_i = xSize;
		 }

		 p = colorBRem;

		 for (i = min_i; i<max_i; i++) {
			for (j = min_j; j<max_j; j++) {


			   if (depth_out == 3) {
			     pixel_value = *(p++); 
 			     output_image->put_pixel(i,j,pixel_value, 0);

			     pixel_value = *(p++); 
 			     output_image->put_pixel(i,j,pixel_value, 1);

			     pixel_value = *(p++); 
 			     output_image->put_pixel(i,j,pixel_value, 2);
			   }
			   else {
				  temp =  *(p++);  // convert RGB to grey-scale and output it
				  temp += *(p++);
				  temp += *(p++);
				  temp = temp/3;
				  pixel_value = (unsigned char) temp;
 			     output_image->put_pixel(i,j,pixel_value, 0);
			   }
			}
		 }
	  }
   }

   if (debug) printf("Leaving log_polar_transform\n\n");

   return;
   
}

#endif

 


/************************************************************************************************

Fourier Vision Code
 
*************************************************************************************************/



/****************************************************************
* 
*  Routine Name: single_sensor_stereo 
* 
*****************************************************************/

void single_sensor_stereo(
                  DVimage *input_image,
                  DVimage **disparity_image, 
                  DVimage **ncc_image,
                  int window_size, 
                  int sampling_period,
					   int calibrate_flag)
					   
{
   DVimage *disparity_magnitude=NULL;
   DVimage *disparity_confidence=NULL;

   int  width, height; 
   int  interference_area;
   int  vertical_tolerance;
   float ncc, ncc_threshold, sum1, sum2, sum3, max_ncc, a, b;

   unsigned char *i_image=NULL;   
   unsigned char *d_image;
                  
   int  i, j, k, l, max_x, max_y, offset, delta_x,min_disparity;
   char debug, debug_image;

   static int vertical_offset = 0;  

   int max_vertical_offset, v, min_v, max_v;
   float cumulative_ncc, max_cumulative_ncc;

#define PIX(f,width,i,j)   (*((f) + ( (j) * (width) )  + (i) ))

  /* set debug flag */

   debug = FALSE;
   debug_image = FALSE;

   if (debug) 
     printf("single sensor stereo: debug on \n");               

	
	
   /* now create working images */

   input_image->get_size(&width,&height);

   disparity_magnitude = new DVimage(width,height,GREYSCALE_IMAGE,NULL,NULL,DVFLOAT);
   disparity_confidence = new DVimage(width,height,GREYSCALE_IMAGE,NULL,NULL,DVFLOAT);
  
   i_image =  (unsigned char *) malloc(sizeof(unsigned char) * width * height);
   d_image =  (unsigned char *) malloc(sizeof(unsigned char) * window_size * window_size *2);



   // ------- set some hard-coded parameters ---------------------------------------------------

   	interference_area = width / 20;      // size of the interference area / 2
	ncc_threshold = (float) 0.98;       // don't accept any match with an ncc less than this value
	vertical_tolerance = 10;           // allow this number of pixels to calibrate vertical
  	                             // offset of prism
  //------------------------------------------------------------------------------------------

   /* read the input image */

   input_image->read(i_image);

	offset = width/2;                



	if (calibrate_flag == TRUE) {

		// need to calibrate vertical offset for non-aligned prism

		max_cumulative_ncc = (float) 0;
		min_v = -vertical_tolerance;
		max_v = vertical_tolerance;

	}
	else {

		min_v = vertical_offset;
		max_v = vertical_offset;
	}


for (v=min_v; v<=max_v; v++) {

   if (calibrate_flag == TRUE) cumulative_ncc = 0;


   min_disparity = width;
   for (j=0; j< height-window_size; j+=sampling_period) {

     if (debug) 
       printf("single_sensor_stereo: processing row %d of %d\n", j,height-window_size);

     for (i=0; i< offset-window_size-interference_area; i+=sampling_period) {
 
		  // search for maximum correlation match on this scanline

		  max_ncc = 0;
		  max_x = 0; max_y = 0;

		  for (delta_x=offset+interference_area; delta_x<width-window_size; delta_x+=2) {

            /* compute the normalized cross correlation in the window region */

		      sum1 = 0; sum2=0; sum3 = 0;
				
            for (k=0; k<window_size; k+=2) {
               for (l=0; l<window_size; l+=2) {
			        a = (float) PIX(i_image,width,i+k,j+l);
			        b = (float) PIX(i_image,width,delta_x+k,j+l+v);
                 sum1 += a*b;
					  sum2 += a*a;
					  sum3 += b*b;
				   }
				}
				
				//ncc = (float) ( sum1 / (sqrt(sum2) * sqrt(sum3)));
				ncc = (float)(sum1 * sum1) / (sum2 * sum3);
			   if (ncc > max_ncc) {
			   	   max_ncc = ncc;
				   max_x = delta_x-i;
				}
		  }

/*

        for (k=0; k<window_size; k++) {
           for (l=0; l<window_size; l++) {
               PIX(d_image,window_size*2,k,l) = PIX(i_image,width,i+k,j+l);
               PIX(d_image,window_size*2,k+window_size,l) = PIX(i_image,width,i+k +max_x,j+l +max_y);
			  }
		  }
		  dump_char_image(d_image,window_size*2, window_size);

*/      

		  // now store the disparity and ncc values

		  if (max_ncc >= ncc_threshold) {
           disparity_magnitude->put_pixel(i+window_size/2,j+window_size/2,(float)max_x);
			  if (max_x < min_disparity) min_disparity = max_x;
 		     disparity_confidence->put_pixel(i+window_size/2,j+window_size/2,(float)(max_ncc-ncc_threshold));
		  }

		  if (calibrate_flag == TRUE)
			  cumulative_ncc += max_ncc;		


		  //printf("max ncc %f at disparity %d %d\n",max_ncc,max_x,max_y);

     }      
   }


	if (calibrate_flag == TRUE) {
		if (cumulative_ncc > max_cumulative_ncc) {
	      max_cumulative_ncc = cumulative_ncc;
	      max_vertical_offset = v;
		}
		printf("computed and maximum cumulative ncc = %f %f, offset %d\n",cumulative_ncc, max_cumulative_ncc, v);
	}
}

if (calibrate_flag == TRUE) {
	vertical_offset = max_vertical_offset;
	printf("Calibrated vertical offset is %d\n", vertical_offset);
}

/*

for (i=0; i<width; i++) {
	for (j=0; j<height; j++) {
		disparity_magnitude->get_pixel(i,j,&a);
		if (a != 0) 
			disparity_magnitude->put_pixel(i,j,a-min_disparity);
	}
}

  */
   interpolate(disparity_magnitude, disparity_image);
   interpolate(disparity_confidence, ncc_image);

 
   /* free up all allocated space, i.e., release local image space */

   delete disparity_magnitude;
   delete disparity_confidence;

   free(i_image);
   free(d_image);
}


/****************************************************************
* 
*  Routine Name: fourier_segmentation - Segment a scene using Fourier analysis
* 
*      Purpose: This routine segments a scene into foreground and background
*            by resolving their respective Fourier components.
*            It is assumed that the foreground and background exhibit
*            different velocities (i.e. that the foreground is moving
*            relative to the background). The background velocity can 
*            be zero but it is assumed that the foreground velocity is
*            greater than the background velocity.
*            The foreground and background velocities may be real, in the
*            case of a sequence of imaged depicting moving objects, 
*            or they may be apparent, in the case that the sequence is 
*            generated by camera motion.
*
*            At present, it is assumed that there is no scaling and 
*            no rotation; i.e. that the motions are fronto-parallel.
*
*            The segmentation is effected by making use of the fact that
*            image translation only gives rise to a regular phase shift in
*            the Fourier domain.  Furthermore, the shift is a linear 
*            function of displacement (real or apparent).  This allows
*            the segmentation problem to be cast as a solution to a 
*            system of non-linear complex equations linking:
*
*            1. the Fourier component of the foreground, 
*            2. its (time or space) displacement-dependent phase change, 
*            3. the Fourier component of the background, 
*            4. its (time or space) displacement-dependent phase change, 
*            5. and the Fourier component of the combined image.
*
*            We solve for the resolved components at each spatial frequency
*            independently, and then produce the segmented images by 
*            computing the inverse Fourier transform
*      
*            Whilst the underlying allow for segmentation of multiple
*            objects (i.e. more than two), the current implementation 
*            allows only figure/ground separation (i.e. it assumes only two
*            objects).
*            
*
*       Input: input_1         - intensity image (1st in sequence)
*            input_2         - intensity image (2nd in sequence)
*            threshold        - double: phase change difference defining
*                            the cut-off frequency of the high-pass 
*                            filter (used for the occlusion model only)
*
*      Output: output_1         - segmented object 1
*            output_2         - segmented object 2
*            phase_output_1    - phase changes of Fourier spectrum
*                            of segmented object 1
*            phase_output_2    - phase changes of Fourier spectrum
*                            of segmented object 2
*          
*      Returns: TRUE (1) on success, FALSE (0) otherwise
*
*  Restrictions: Restrictions on data or input as applicable
*   Written By: David Vernon
*        Date: Nov 23, 2001
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications: *
*
* Modified the way in which the Hough transform accumulator is updated.
* Rather than solving for vy given, inter alia, ky regardless of the value
* of ky, we now solve for vy only if ky < kx and solve for vx if ky >= kx
* This avoids problems with truncation of computed value of vy when
* ky becomes large and the consequent problems with incorrect local
* maxima in the Hough accumulator around values where vy approaches zero
*
* After much experiment with the localization of the maxima in the 
* Hough accumulator, we presently use the original approach with NO
* filtering.  
* Note that the evidence values which are based on the magnitude of the
* maxima are presently computed as the sum of the Hough accumulator in
* a 3x3 region centered on the maxima.
*
* These modifications were made following an extensive set of trials to
* ensure accurate computation of the velocity of the image for use with
* the optical flow field function.
*
* DV 3/3/98
*
*
* Modified the way in which the Hough accumulator is updated (again)!
* This time, the minimum and maximum velocities are computed such that
* they are bounded by the range of the accumulator and, more significantly,
* the increment value is now computed to be equivalent to the interpixel
* distance of the accumulator.  This gives less noisy accumulators and
* more distinct maxima.
*
* DV 21/4/98
*
* 
* Added a new routine - dump_float_image - for debugging purposes.
*
* DV 21/4/98
*
*
* Modified routine so that it now takes the intensity images as input
* and produces the segmented images as output, i.e., all the FFT preprocessing
* is now done in this routine.
*
* DV 22/9/98
*
* 
* Modified sort_fourier_components to increase the mid-maxima value to be 60%
* of the maximum (rather than 50%) as a condition in maxima detection in the
* Hough accumulator
*
* Also added a new check on the maxima; if the second maximum is less than
* 10% of the primary maximum, it is assumed now that it is in fact a spurious
* maximum and that the real second maximum was due to a zero-velocity image
* whose components had been suppressed (as their phase difference was less than
* the allowed tolerance).
*
* DV 27/9/98
*
* Added the option of choosing among three residue types
*
* DV 3/10/08
*
*
* Following much work checking the implementation of the residues, 
* it emerges that the advanced residue models don't work well and, in fact, 
* the simple static residue doesn't actually work particularly well either.  
* The best results are obtained with a zero residue but ensuring that the 
* errors that are produced in using the linear model (with no residue) are 
* removed by attenuating those frequencies for which the phase difference of
* foreground and background are approximately equal.
*
* I have left the static residue in as an option to allow it to be compared to 
* the zero residue model but I may take it out at a later date. 
( (taken out 11/8/99 DV)
*
* In summary, the generalized occlusion model now differs from the linear 
* additive model only in the following ways:
*
* 1. Only a subset of (low) frequencies are used when computing the velocity of
*   the foreground and background (and, hence, sorting the components); 
*   the high frequency estimates tend to be error-prone and distort the 
*   Hough transform considerably.
*
* 2. The velocities are used to compute the an estimate of the true phase 
*   changes and these values values are then used to  replace those 
*   computed using the analytic model.
*   We also compute those for the high frequencies which weren't solved.
*
* 3. The Fourier components at all frequencies then re-computed using the true 
*   phase changes
*
* 4. Since the defining equations are only approximate for the occlusion case
*   (as the describe an additive model), the errors that result when computing 
*   Fourier components at frequencies which exhibit approximately the same 
*   phase changes for foreground and background are removed by attenuating 
*   those frequencies.  Note that these errors become more significant at 
*   these frequencies because the expression for the components involves a 
*   denominator which approaches zero as the phase changes approach each other.
*
* It is worth remarking at this point that the failure of the residue model 
* is due, understandably, to the fact that its formulation required an 
* assumption that the residue remains constant from frame to frame.  
* This is clearly not the case for most backgrounds and the error this 
* introduces is greater than the error involved in assuming that the residue 
* is very small or zero.  The key to the  success of the occlusion approach 
* over the raw application of the linear approach lies in the estimation of 
* the correct phase changes and the effective attenuation
* of error-prone frequencies.
*
* DV 22/11/98
*
*
* Binocular model implemented using additive model with attenuation of 
* ill-conditioned spatial frequencies. 
*
* DV 25/11/98
*
*
* Scale model implemented as an optional refinement on the solution
* We do it this way as the scaling is solved by Newton-Raphson using
* the translationary non-scaling solution as the first approximation
* Not working properly
*
* DV 28/5/99
*
* Altered tolerance 'mid_value_drop' in routine 'sort_fourier_components'
* to a higher value to enable non-synthetic data-sets to be analysed 
* successfully
*
* DV 16/7/99
*
* Totally reworked the binocular model: it now requires only two images
* rather than the four (left and right at times t0 and t1) in the original
* model.
*
* DV 11/8/99
*
* Modified the relative value of minima in phase difference and
* magnitude difference images when identifying their periodicity
* in identify_phase_changes (new binocular model).
*
* DV 6/2/00
*
* Major overhaul on code to tidy it up and improve efficiency of some
* loops.  This applied in particular to sort_fourier_components which 
* uses the Hough transform.  Indeed, this HT could be optimized even more
* by using a DDA algorithm.  As things stand at present, more than 80% of the 
* time taken by the algorithm is spent computing forward and backward FFTs (occlusion model);
* 65% in the case of the additive model. 
* Since the kfft routine has a considerable data-handling overhead, the next step
* in optimizing the code is to implement a dedicated FFT (after which we can look at
* optimizing the segmentation code again
*
* DV 7/4/01
*
*
* Added monocular model for single sensor stereo
*
* DV 14/4/01
*
*
* Added a flag velocity_only
* If this is set true then only the velocity or disparity values are computed
* from the phase and the segmented images are not reconstructed using the inverse
* Fourier tranform.  This is intended to optimize the computation of optical 
* flow fields and disparity fields when reconstruction is not required
*
* DV 20/5/01
*
*
* Replaced the calls to kfft and related khoros routines with a dedicated
* fft routine based on the code in Numerical Recipes in C
* This resulted in a reduction in total processing time of 65% compared with
* the original Khoros-based code for segmentation using the quadocular model 
* (i.e. 4 ffts and two inverse ffts)
* DV 28/5/01
*
*
* Added mode flag to select method of identifying phase changes in monocular model
* DV 23/11/01
*
*
* Began porting to PC
* DV 23/4/02
*
* Ported with GUI to PC
* DV 18/10/02
*
* Removed model parameter (1/2 for additive/occluding) and associated code: 
* All segmentation is now done with the occluding model which recalculates the phases
* once the velocities have been computed; this forces correct computation of the 
* Fourier phasors.
*
* Removed also option to process using 4 images: now always use 2
* DV 08/12/05
*
* Removed velocity file
* DV 02/08/06
*
* Consolidated all the code into one routine and used new cross_power_spectrum function
* DV 03/08/06
*
* Added filter_radius and non_maxima_suppression_radius parameters
* DV 15/08/06  
*
* Changed output of phase images to cross-power spectrum and Fourier spectrum of second (faster moving) image
* DV 15/08/06  
*
* Added masking facility to superimpose the original image data onto the segmented regions
* DV 18/08/06
*
****************************************************************/
 

void fourier_segmentation_old (DVimage *input_image_1, 
                           DVimage *input_image_2,
                           DVimage *output_image_1, DVimage *output_image_2,
                           DVimage *fourier_output_image_1, DVimage *fourier_output_image_2, 
                           double threshold,
                           int filter_radius,
                           int non_maxima_suppression_radius,
                           double min_max_threshold, 
                           int mask,
                           double mask_threshold) 
{

double log_magnitude(double a_r, double a_i);

     
   static float *input1 = NULL;
   static float *input2 = NULL;
   static float *output1 = NULL;
   static float *output2 = NULL;
   static float *po1 = NULL;      
   static float *po2 = NULL;
   static float *ri1 = NULL;      
   static float *ri2 = NULL;
   static float *ii1 = NULL;      
   static float *ii2 = NULL;
   static float *ro1 = NULL;      
   static float *ro2 = NULL;
   static float *io1 = NULL;      
   static float *io2 = NULL;
   static int   width = 0;
   static int   height = 0;
   static int   depth = 0;

   int     max_velocity = 1;
   int     i, j;
   char    *conversion=NULL;
   char    *seg_datatype=NULL;

   double  ft0_r, ft0_i, ft1_r, ft1_i;
   double  t1_r, t3_r, t4_r, t1_i, t3_i, t4_i;
   double  phi1_r, phi1_i, phi2_r, phi2_i;
   double  f1_r, f1_i, f2_r, f2_i;
   float   vel_x1, vel_y1, vel_x2, vel_y2;
   double  v_max;
   double  temp, temp1,temp2;
   double  scale_factor;
   int     kx,ky;
   float   pd1, pd2;

   int    kx_lb, ky_lb, kx_ub, ky_ub;
   int    w, h, d;
   int    number_of_maxima;
   float  max1, max2;
 

   DVimage *image_cps1;
   DVimage *image_cps2;
   DVimage *temp_image;

   maxima_data_type maxima[10];

   char    debug, dump_debug_image;

   
#define PIX(f,width,i,j)   (*((f) + ( (j) * (width) )  + (i) ))

  /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 
 

   if (debug) printf("fourier_segmentation: debug on \n");  

   if (input_image_1==NULL || input_image_2==NULL) {
	   printf("fourier_segmentation: fatal error - no valid images passed as arguments\n");
	   return;
   }

  
   /* create local images for processing */

   input_image_1->get_size(&w,&h);
   d=input_image_1->get_image_mode();

   if (w != width || h != height && d || depth) {

      width = w; 
      height = h;
      depth = d;

      if (po1 != NULL) free(po1);
      if (po2 != NULL) free(po2);
      if (ro1 != NULL) free(ro1);
      if (ro2 != NULL) free(ro2);
      if (io1 != NULL) free(io1);
      if (io2 != NULL) free(io2);
      if (ri1 != NULL) free(ri1);
      if (ri2 != NULL) free(ri2);
      if (ii1 != NULL) free(ii1);
      if (ii2 != NULL) free(ii2);
      if (input1 != NULL) free(input1);
      if (input2 != NULL) free(input2);
      if (output1 != NULL) free(output1);
      if (output2 != NULL) free(output2);

      input1 =  (float *) malloc(sizeof(float) * width * height);
      input2 =  (float *) malloc(sizeof(float) * width * height);
      ri1   =   (float *) malloc(sizeof(float) * width * height);
      ii1   =   (float *) malloc(sizeof(float) * width * height);
      ri2   =   (float *) malloc(sizeof(float) * width * height);
      ii2   =   (float *) malloc(sizeof(float) * width * height); 
      ro1    =  (float *) malloc(sizeof(float) * width * height);
      ro2    =  (float *) malloc(sizeof(float) * width * height);
      io1    =  (float *) malloc(sizeof(float) * width * height);
      io2    =  (float *) malloc(sizeof(float) * width * height);
      po1    =  (float *) malloc(sizeof(float) * width * height);
      po2    =  (float *) malloc(sizeof(float) * width * height);
      output1 = (float *) malloc(sizeof(float) * width * height); 
      output2 = (float *) malloc(sizeof(float) * width * height);
   }

   if (depth == 1) {
      input_image_1->read(input1);
      input_image_2->read(input2);
   }
   else if (depth == 3) {  // convert to grey-scale
      for (i= 0; i<width;i++) { 
         for (j= 0; j<height; j++) {

            temp =  (int) pixel(input_image_1->idata,width,i,j,0);
            temp += (int) pixel(input_image_1->idata,width,i,j,1); 
				temp += (int) pixel(input_image_1->idata,width,i,j,2); 
				temp = temp / 3;
		      PIX(input1,width,i,j) = (unsigned char) temp;
 
 			   temp = (int) pixel(input_image_2->idata,width,i,j,0);
            temp += (int) pixel(input_image_2->idata,width,i,j,1); 
				temp += (int) pixel(input_image_2->idata,width,i,j,2); 
				temp = temp / 3;
		      PIX(input2,width,i,j) = (unsigned char) temp;
	      }
      }
   }
 

   // FFT inputs

   fft(input1, ri1, ii1, width, height,1);
   fft(input2, ri2, ii2, width, height,1);
 

   // identify phase changes

 
   image_cps1 = new DVimage(width,height,GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
   image_cps2 = new DVimage(width,height,GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);

   cross_power_spectrum (input_image_2, input_image_1, image_cps1);  

   enhance_local_maxima (image_cps1, filter_radius, image_cps2); 	 


   number_of_maxima = 2;
   find_maxima (image_cps2, number_of_maxima, non_maxima_suppression_radius, maxima);  


   vel_x1 = (float)maxima[0].x - width/2 ;
   vel_y1 = (float)maxima[0].y - height/2;
   vel_x2 = (float)maxima[1].x - width/2;
   vel_y2 = (float)maxima[1].y - height/2;
   max1 =   (float)maxima[0].value/(float)1e6;
   max2 =   (float)maxima[1].value/(float)1e6;


   // make sure the second image is the one with the greater velocity 

   if ((vel_x1*vel_x1 + vel_y1*vel_y1) >  (vel_x2*vel_x2 + vel_y2*vel_y2)) {

      v_max = sqrt(vel_x1*vel_x1 + vel_y1*vel_y1); 

      temp1 = vel_x1;  vel_x1 = vel_x2; vel_x2 = (float)temp1;
      temp1 = vel_y1;  vel_y1 = vel_y2; vel_y2 = (float)temp1;
      temp1 = max1;    max1 = max2;     max2   = (float)temp1;

   }
   else {
      v_max = sqrt(vel_x2*vel_x2 + vel_y2*vel_y2);
   }
 

   if (debug)
      printf("Maxima: %3.1f at (%3.0f, %3.0f) and %3.2f at (%3.0f, %3.0f)\n",max1, vel_x1,vel_y1, max2, vel_x2,vel_y2);

   scale_factor = (width) / (float) (2 * 3.14159);

   kx_lb =  width  / 2; kx_lb = -kx_lb;    
   kx_ub =  width  / 2;
   ky_lb =  height / 2; ky_lb = -ky_lb;
   ky_ub =  height / 2;

   /* now compute the ideal phase changes corresponding to these velocities;         */
   /* force phase wrapping to facilitate comparison with the image phase differences */
 
   for (kx = kx_lb; kx < kx_ub; kx++) {
      for (ky = ky_lb; ky < ky_ub; ky++) {

         i = kx + (width / 2);
         j = ky + (height / 2);   

 
         pd1 = -(vel_x1 * kx + vel_y1 * ky)/(float)scale_factor;
 
         if (pd1 > 0)
            pd1 = pd1 - (2 *  (float)3.14159 * (int) (((pd1 / ((float)3.14159)) + 1)/2));
         else 
            pd1 = pd1 - (2 *  (float)3.14159 * (int) (((pd1 / ((float)3.14159)) - 1)/2));

 
         pd2 = -(vel_x2 * kx + vel_y2 * ky)/(float)scale_factor;

         if (pd2 > 0)
            pd2 = pd2 - (2 *  (float)3.14159 * (int) (((pd2 / ((float)3.14159)) + 1)/2));
         else
            pd2 = pd2 - (2 *  (float)3.14159 * (int) (((pd2 / ((float)3.14159)) - 1)/2));

         PIX(po1, width, i, j) = pd1;
         PIX(po2, width, i, j) = pd2;
      }
   }


   /* Now that we know the correct phase differences, we  */
   /* solve for all Fourier components                    */


   for (i= 0; i<width;i++) { 
      for (j= 0; j<height; j++) {
         PIX(ro1,width,i,j) = 0;
         PIX(io1,width,i,j) = 0;
         PIX(ro2,width,i,j) = 0;
         PIX(io2,width,i,j) = 0;
	  }
   }

   for (i=0; i<width; i++) { 
      for (j=0; j<height; j++) {

         // only compute the frequencies inside a band where the phases are not equal
        
         if (    ((((vel_x1 - vel_x2) * (i-width/2))   +  ((vel_y1 - vel_y2) * (j-height/2)) - width ) < 0) 
              &&  (((vel_x1 - vel_x2) * (i-width/2))   +  ((vel_y1 - vel_y2) * (j-height/2)) + width ) > 0)    {
         

            ft0_r = PIX(ri1, width, i, j);
            ft1_r = PIX(ri2, width, i, j);
            ft0_i = PIX(ii1, width, i, j);
            ft1_i = PIX(ii2, width, i, j);

            phi1_r = cos(PIX(po1,width,i,j));
            phi1_i = sin(PIX(po1,width,i,j));
            phi2_r = cos(PIX(po2,width,i,j));
            phi2_i = sin(PIX(po2,width,i,j));
 

            if ((PIX(po1,width,i,j) != PIX(po2,width,i,j))) {  // the phases are equal in the centre of the bank

                                                                                // compute if phi1 != phi2
               multiply_complex(phi1_r, phi1_i,  ft0_r,  ft0_i, &t1_r, &t1_i);     /* ft0 phi1                     */
               subtract_complex( ft1_r,  ft1_i,  t1_r,   t1_i,  &t3_r, &t3_i);     /* ft1 - ft0 phi1               */
               subtract_complex(phi2_r, phi2_i,phi1_r, phi1_i,  &t4_r, &t4_i);     /* phi2 - phi1                  */
               divide_complex  (  t3_r,   t3_i,  t4_r,   t4_i,  &f2_r, &f2_i);     /* ft1 - ft0 phi1 / phi2 - phi1 */
               subtract_complex( ft0_r,  ft0_i,  f2_r,   f2_i,  &f1_r, &f1_i);     /* ft0 - f2                     */
         
               // attenuate if phi2-phi1 is close to zero (i.e. solution above is ill-conditioned)
 
    	         temp2 = fabs(PIX(po1,width,i,j) - PIX(po2,width,i,j));
               if (temp2 > 3.14159) temp2 = fabs(temp2 - (2 * 3.14159));
 
 
               if  (temp2 <= (threshold * v_max)) {
                  scale_factor = sin( ((temp2) /  (threshold * v_max)) * (3.14159/2)) ;
                  scale_factor = scale_factor * scale_factor;
		         }
               else {
                  scale_factor = 1;
               }
  
               f1_r   = f1_r  * scale_factor;
               f1_i   = f1_i  * scale_factor;
               f2_r   = f2_r  * scale_factor;
               f2_i   = f2_i  * scale_factor;
   	      }
         } 
         else {
            f1_r = 0;  f1_i = 0;
            f2_r = 0;  f2_i = 0;
            PIX(po1, width, i, j) = 0;
            PIX(po2, width, i, j) = 0;
 		   }  

         PIX(ro1,width,i,j) = (float)f1_r;
         PIX(io1,width,i,j) = (float)f1_i;
         PIX(ro2,width,i,j) = (float)f2_r;
         PIX(io2,width,i,j) = (float)f2_i;
      }
   }
 
   // create segmented images from Fourier components
  
   fft(output1, ro1, io1, width,height,-1);
   fft(output2, ro2, io2, width,height,-1);
 
 
   /* transfer the processed image to the output images */

   // copying float to int ... need to contrast stretch to allow it to be copied properly
   // only show the results if there was sigificant evidence for it, 
   // i.e. two significant maxima (just one maximum implies no relative motion)

   if (max2 > max1 * (min_max_threshold/100)) { // parameter value is a percentage so convert first

      temp_image = new DVimage(width,height,GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
 
      temp_image->write(output1); 
      temp_image->contrast_stretch();

      for (i= 0; i<width;i++) { 
         for (j= 0; j<height; j++) {

 			   pixel(output_image_1->idata,width,i,j,0) = (unsigned char) PIX(temp_image->fdata,width,i,j);
			   if (output_image_1->colour_mode == COLOUR_IMAGE) {
 			      pixel(output_image_1->idata,width,i,j,1) = (unsigned char) PIX(temp_image->fdata,width,i,j);
 			      pixel(output_image_1->idata,width,i,j,2) = (unsigned char) PIX(temp_image->fdata,width,i,j);
			   }
	      }
      }



      temp_image->write(output2); 
      temp_image->contrast_stretch();

      for (i= 0; i<width;i++) { 
         for (j= 0; j<height; j++) {

 			   pixel(output_image_2->idata,width,i,j,0) = (unsigned char) PIX(temp_image->fdata,width,i,j);
			   if (output_image_2->colour_mode == COLOUR_IMAGE) {
 			      pixel(output_image_2->idata,width,i,j,1) = (unsigned char) PIX(temp_image->fdata,width,i,j);
 			      pixel(output_image_2->idata,width,i,j,2) = (unsigned char) PIX(temp_image->fdata,width,i,j);
			   }
	      }
      }

 

      if (mask) 
         mask_image (input_image_1, output_image_2, output_image_2, mask_threshold);


      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            temp1 =  PIX(ro1,width,i,j);
            temp2 =  PIX(io1,width,i,j);
            temp_image->put_pixel(i, j, (float) log_magnitude(temp1,temp2),0);
         }
      }
      temp_image->contrast_stretch();
      fourier_output_image_1->write(temp_image->fdata); 


      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            temp1 =  PIX(ro2,width,i,j);
            temp2 =  PIX(io2,width,i,j);
            temp_image->put_pixel(i, j, (float) log_magnitude(temp1,temp2),0);
         }
      }
      temp_image->contrast_stretch();
      fourier_output_image_2->write(temp_image->fdata); 

      delete temp_image;
   }
 
 

   delete image_cps1;
   delete image_cps2;

   if (debug) printf("Leaving fourier_segmentation\n"); 

   return;
  
}

 


/****************************************************************
* 
*  Routine Name: fourier_segmentation - Segment a scene using Fourier analysis
* 
*      Purpose: This routine segments a scene into foreground and background
*            by resolving their respective Fourier components.
*            It is assumed that the foreground and background exhibit
*            different velocities (i.e. that the foreground is moving
*            relative to the background). The background velocity can 
*            be zero but it is assumed that the foreground velocity is
*            greater than the background velocity.
*            The foreground and background velocities may be real, in the
*            case of a sequence of imaged depicting moving objects, 
*            or they may be apparent, in the case that the sequence is 
*            generated by camera motion.
*
*            At present, it is assumed that there is no scaling and 
*            no rotation; i.e. that the motions are fronto-parallel.
*
*            The segmentation is effected by making use of the fact that
*            image translation only gives rise to a regular phase shift in
*            the Fourier domain.  Furthermore, the shift is a linear 
*            function of displacement (real or apparent).  This allows
*            the segmentation problem to be cast as a solution to a 
*            system of non-linear complex equations linking:
*
*            1. the Fourier component of the foreground, 
*            2. its (time or space) displacement-dependent phase change, 
*            3. the Fourier component of the background, 
*            4. its (time or space) displacement-dependent phase change, 
*            5. and the Fourier component of the combined image.
*
*            We solve for the resolved components at each spatial frequency
*            independently, and then produce the segmented images by 
*            computing the inverse Fourier transform
*      
*            Whilst the underlying allow for segmentation of multiple
*            objects (i.e. more than two), the current implementation 
*            allows only figure/ground separation (i.e. it assumes only two
*            objects).
*            
*
*       Input: input_1         - intensity image (1st in sequence)
*            input_2         - intensity image (2nd in sequence)
*            threshold        - double: phase change difference defining
*                            the cut-off frequency of the high-pass 
*                            filter (used for the occlusion model only)
*
*      Output: output_1         - segmented object 1
*            output_2         - segmented object 2
*            phase_output_1    - phase changes of Fourier spectrum
*                            of segmented object 1
*            phase_output_2    - phase changes of Fourier spectrum
*                            of segmented object 2
*          
*      Returns: TRUE (1) on success, FALSE (0) otherwise
*
*  Restrictions: Restrictions on data or input as applicable
*   Written By: David Vernon
*        Date: Nov 23, 2001
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications: *
*
* Modified the way in which the Hough transform accumulator is updated.
* Rather than solving for vy given, inter alia, ky regardless of the value
* of ky, we now solve for vy only if ky < kx and solve for vx if ky >= kx
* This avoids problems with truncation of computed value of vy when
* ky becomes large and the consequent problems with incorrect local
* maxima in the Hough accumulator around values where vy approaches zero
*
* After much experiment with the localization of the maxima in the 
* Hough accumulator, we presently use the original approach with NO
* filtering.  
* Note that the evidence values which are based on the magnitude of the
* maxima are presently computed as the sum of the Hough accumulator in
* a 3x3 region centered on the maxima.
*
* These modifications were made following an extensive set of trials to
* ensure accurate computation of the velocity of the image for use with
* the optical flow field function.
*
* DV 3/3/98
*
*
* Modified the way in which the Hough accumulator is updated (again)!
* This time, the minimum and maximum velocities are computed such that
* they are bounded by the range of the accumulator and, more significantly,
* the increment value is now computed to be equivalent to the interpixel
* distance of the accumulator.  This gives less noisy accumulators and
* more distinct maxima.
*
* DV 21/4/98
*
* 
* Added a new routine - dump_float_image - for debugging purposes.
*
* DV 21/4/98
*
*
* Modified routine so that it now takes the intensity images as input
* and produces the segmented images as output, i.e., all the FFT preprocessing
* is now done in this routine.
*
* DV 22/9/98
*
* 
* Modified sort_fourier_components to increase the mid-maxima value to be 60%
* of the maximum (rather than 50%) as a condition in maxima detection in the
* Hough accumulator
*
* Also added a new check on the maxima; if the second maximum is less than
* 10% of the primary maximum, it is assumed now that it is in fact a spurious
* maximum and that the real second maximum was due to a zero-velocity image
* whose components had been suppressed (as their phase difference was less than
* the allowed tolerance).
*
* DV 27/9/98
*
* Added the option of choosing among three residue types
*
* DV 3/10/08
*
*
* Following much work checking the implementation of the residues, 
* it emerges that the advanced residue models don't work well and, in fact, 
* the simple static residue doesn't actually work particularly well either.  
* The best results are obtained with a zero residue but ensuring that the 
* errors that are produced in using the linear model (with no residue) are 
* removed by attenuating those frequencies for which the phase difference of
* foreground and background are approximately equal.
*
* I have left the static residue in as an option to allow it to be compared to 
* the zero residue model but I may take it out at a later date. 
( (taken out 11/8/99 DV)
*
* In summary, the generalized occlusion model now differs from the linear 
* additive model only in the following ways:
*
* 1. Only a subset of (low) frequencies are used when computing the velocity of
*   the foreground and background (and, hence, sorting the components); 
*   the high frequency estimates tend to be error-prone and distort the 
*   Hough transform considerably.
*
* 2. The velocities are used to compute the an estimate of the true phase 
*   changes and these values values are then used to  replace those 
*   computed using the analytic model.
*   We also compute those for the high frequencies which weren't solved.
*
* 3. The Fourier components at all frequencies then re-computed using the true 
*   phase changes
*
* 4. Since the defining equations are only approximate for the occlusion case
*   (as the describe an additive model), the errors that result when computing 
*   Fourier components at frequencies which exhibit approximately the same 
*   phase changes for foreground and background are removed by attenuating 
*   those frequencies.  Note that these errors become more significant at 
*   these frequencies because the expression for the components involves a 
*   denominator which approaches zero as the phase changes approach each other.
*
* It is worth remarking at this point that the failure of the residue model 
* is due, understandably, to the fact that its formulation required an 
* assumption that the residue remains constant from frame to frame.  
* This is clearly not the case for most backgrounds and the error this 
* introduces is greater than the error involved in assuming that the residue 
* is very small or zero.  The key to the  success of the occlusion approach 
* over the raw application of the linear approach lies in the estimation of 
* the correct phase changes and the effective attenuation
* of error-prone frequencies.
*
* DV 22/11/98
*
*
* Binocular model implemented using additive model with attenuation of 
* ill-conditioned spatial frequencies. 
*
* DV 25/11/98
*
*
* Scale model implemented as an optional refinement on the solution
* We do it this way as the scaling is solved by Newton-Raphson using
* the translationary non-scaling solution as the first approximation
* Not working properly
*
* DV 28/5/99
*
* Altered tolerance 'mid_value_drop' in routine 'sort_fourier_components'
* to a higher value to enable non-synthetic data-sets to be analysed 
* successfully
*
* DV 16/7/99
*
* Totally reworked the binocular model: it now requires only two images
* rather than the four (left and right at times t0 and t1) in the original
* model.
*
* DV 11/8/99
*
* Modified the relative value of minima in phase difference and
* magnitude difference images when identifying their periodicity
* in identify_phase_changes (new binocular model).
*
* DV 6/2/00
*
* Major overhaul on code to tidy it up and improve efficiency of some
* loops.  This applied in particular to sort_fourier_components which 
* uses the Hough transform.  Indeed, this HT could be optimized even more
* by using a DDA algorithm.  As things stand at present, more than 80% of the 
* time taken by the algorithm is spent computing forward and backward FFTs (occlusion model);
* 65% in the case of the additive model. 
* Since the kfft routine has a considerable data-handling overhead, the next step
* in optimizing the code is to implement a dedicated FFT (after which we can look at
* optimizing the segmentation code again
*
* DV 7/4/01
*
*
* Added monocular model for single sensor stereo
*
* DV 14/4/01
*
*
* Added a flag velocity_only
* If this is set true then only the velocity or disparity values are computed
* from the phase and the segmented images are not reconstructed using the inverse
* Fourier tranform.  This is intended to optimize the computation of optical 
* flow fields and disparity fields when reconstruction is not required
*
* DV 20/5/01
*
*
* Replaced the calls to kfft and related khoros routines with a dedicated
* fft routine based on the code in Numerical Recipes in C
* This resulted in a reduction in total processing time of 65% compared with
* the original Khoros-based code for segmentation using the quadocular model 
* (i.e. 4 ffts and two inverse ffts)
* DV 28/5/01
*
*
* Added mode flag to select method of identifying phase changes in monocular model
* DV 23/11/01
*
*
* Began porting to PC
* DV 23/4/02
*
* Ported with GUI to PC
* DV 18/10/02
*
* Removed model parameter (1/2 for additive/occluding) and associated code: 
* All segmentation is now done with the occluding model which recalculates the phases
* once the velocities have been computed; this forces correct computation of the 
* Fourier phasors.
*
* Removed also option to process using 4 images: now always use 2
* DV 08/12/05
*
* Removed velocity file
* DV 02/08/06
*
* Consolidated all the code into one routine and used new cross_power_spectrum function
* DV 03/08/06
*
* Added filter_radius and non_maxima_suppression_radius parameters
* DV 15/08/06  
*
* Changed output of phase images to cross-power spectrum and Fourier spectrum of second (faster moving) image
* DV 15/08/06  
*
* Added masking facility to superimpose the original image data onto the segmented regions
* DV 18/08/06
*
* Now takes images of any size as arguments and extracts the power-of-two images here
* The images returned are the same size as the images passed, with the segmented and Fourier data inserted
* DV 19/09/06
*
****************************************************************/
 

void fourier_segmentation (DVimage *input_image_1, 
                           DVimage *input_image_2,
                           DVimage *output_image_1, DVimage *output_image_2,
                           DVimage *fourier_output_image_1, DVimage *fourier_output_image_2, 
                           double threshold,
                           int filter_radius,
                           int non_maxima_suppression_radius,
                           double min_max_threshold, 
                           int mask,
                           double mask_threshold) 
{

double log_magnitude(double a_r, double a_i);

     
   static float *input1 = NULL;
   static float *input2 = NULL;
   static float *output1 = NULL;
   static float *output2 = NULL;
   static float *po1 = NULL;      
   static float *po2 = NULL;
   static float *ri1 = NULL;      
   static float *ri2 = NULL;
   static float *ii1 = NULL;      
   static float *ii2 = NULL;
   static float *ro1 = NULL;      
   static float *ro2 = NULL;
   static float *io1 = NULL;      
   static float *io2 = NULL;
   static int   width = 0;
   static int   height = 0;
   static int   depth = 0;

   int     max_velocity = 1;
   int     i, j;
   char    *conversion=NULL;
   char    *seg_datatype=NULL;

   double  ft0_r, ft0_i, ft1_r, ft1_i;
   double  t1_r, t3_r, t4_r, t1_i, t3_i, t4_i;
   double  phi1_r, phi1_i, phi2_r, phi2_i;
   double  f1_r, f1_i, f2_r, f2_i;
   float   vel_x1, vel_y1, vel_x2, vel_y2;
   double  v_max;
   double  temp, temp1,temp2;
   double  scale_factor;
   int     kx,ky;
   float   pd1, pd2;

   int    kx_lb, ky_lb, kx_ub, ky_ub;
   int    w, h, d;
   int    number_of_maxima;
   float  max1, max2;
 

   DVimage *image_cps1;
   DVimage *image_cps2;
   DVimage *temp_image;

   maxima_data_type maxima[10];

   char    debug, dump_debug_image;

   
#define PIX(f,width,i,j)   (*((f) + ( (j) * (width) )  + (i) ))

  /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 
 

   if (debug) printf("fourier_segmentation: debug on \n");  

   if (input_image_1==NULL || input_image_2==NULL) {
	   printf("fourier_segmentation: fatal error - no valid images passed as arguments\n");
	   return;
   }

  
   /* create local images for processing */

   input_image_1->get_size(&w,&h);
   d=input_image_1->get_image_mode();

   if (w != width || h != height && d || depth) {

      width = w; 
      height = h;
      depth = d;

      if (po1 != NULL) free(po1);
      if (po2 != NULL) free(po2);
      if (ro1 != NULL) free(ro1);
      if (ro2 != NULL) free(ro2);
      if (io1 != NULL) free(io1);
      if (io2 != NULL) free(io2);
      if (ri1 != NULL) free(ri1);
      if (ri2 != NULL) free(ri2);
      if (ii1 != NULL) free(ii1);
      if (ii2 != NULL) free(ii2);
      if (input1 != NULL) free(input1);
      if (input2 != NULL) free(input2);
      if (output1 != NULL) free(output1);
      if (output2 != NULL) free(output2);

      input1 =  (float *) malloc(sizeof(float) * width * height);
      input2 =  (float *) malloc(sizeof(float) * width * height);
      ri1   =   (float *) malloc(sizeof(float) * width * height);
      ii1   =   (float *) malloc(sizeof(float) * width * height);
      ri2   =   (float *) malloc(sizeof(float) * width * height);
      ii2   =   (float *) malloc(sizeof(float) * width * height); 
      ro1    =  (float *) malloc(sizeof(float) * width * height);
      ro2    =  (float *) malloc(sizeof(float) * width * height);
      io1    =  (float *) malloc(sizeof(float) * width * height);
      io2    =  (float *) malloc(sizeof(float) * width * height);
      po1    =  (float *) malloc(sizeof(float) * width * height);
      po2    =  (float *) malloc(sizeof(float) * width * height);
      output1 = (float *) malloc(sizeof(float) * width * height); 
      output2 = (float *) malloc(sizeof(float) * width * height);
   }

   if (depth == 1) {
      input_image_1->read(input1);
      input_image_2->read(input2);
   }
   else if (depth == 3) {  // convert to grey-scale
      for (i= 0; i<width;i++) { 
         for (j= 0; j<height; j++) {

            temp =  (int) pixel(input_image_1->idata,width,i,j,0);
            temp += (int) pixel(input_image_1->idata,width,i,j,1); 
				temp += (int) pixel(input_image_1->idata,width,i,j,2); 
				temp = temp / 3;
		      PIX(input1,width,i,j) = (unsigned char) temp;
 
 			   temp = (int) pixel(input_image_2->idata,width,i,j,0);
            temp += (int) pixel(input_image_2->idata,width,i,j,1); 
				temp += (int) pixel(input_image_2->idata,width,i,j,2); 
				temp = temp / 3;
		      PIX(input2,width,i,j) = (unsigned char) temp;
	      }
      }
   }
 

   // FFT inputs

   fft(input1, ri1, ii1, width, height,1);
   fft(input2, ri2, ii2, width, height,1);
 

   // identify phase changes

 
   image_cps1 = new DVimage(width,height,GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
   image_cps2 = new DVimage(width,height,GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);

   cross_power_spectrum (input_image_2, input_image_1, image_cps1);  

   enhance_local_maxima (image_cps1, filter_radius, image_cps2); 	 

   number_of_maxima = 2;
   find_maxima (image_cps2, number_of_maxima, non_maxima_suppression_radius, maxima);  


   vel_x1 = (float)maxima[0].x - width/2 ;
   vel_y1 = (float)maxima[0].y - height/2;
   vel_x2 = (float)maxima[1].x - width/2;
   vel_y2 = (float)maxima[1].y - height/2;
   max1 = maxima[0].value/(float)1e6;
   max2 = maxima[1].value/(float)1e6;


   // make sure the second image is the one with the greater velocity 

   if ((vel_x1*vel_x1 + vel_y1*vel_y1) >  (vel_x2*vel_x2 + vel_y2*vel_y2)) {

      v_max = sqrt(vel_x1*vel_x1 + vel_y1*vel_y1); 

      temp1 = vel_x1;  vel_x1 = vel_x2; vel_x2 = (float)temp1;
      temp1 = vel_y1;  vel_y1 = vel_y2; vel_y2 = (float)temp1;
      temp1 = max1;    max1 = max2;     max2   = (float)temp1;

   }
   else {
      v_max = sqrt(vel_x2*vel_x2 + vel_y2*vel_y2);
   }
 

   if (debug)
      printf("Maxima: %3.1f at (%3.0f, %3.0f) and %3.2f at (%3.0f, %3.0f)\n",max1, vel_x1,vel_y1, max2, vel_x2,vel_y2);

   scale_factor = (width) / (float) (2 * 3.14159);

   kx_lb =  width  / 2; kx_lb = -kx_lb;    
   kx_ub =  width  / 2;
   ky_lb =  height / 2; ky_lb = -ky_lb;
   ky_ub =  height / 2;

   /* now compute the ideal phase changes corresponding to these velocities;         */
   /* force phase wrapping to facilitate comparison with the image phase differences */
 
   for (kx = kx_lb; kx < kx_ub; kx++) {
      for (ky = ky_lb; ky < ky_ub; ky++) {

         i = kx + (width / 2);
         j = ky + (height / 2);   

 
         pd1 = -(vel_x1 * kx + vel_y1 * ky)/(float)scale_factor;
 
         if (pd1 > 0)
            pd1 = pd1 - (2 *  (float)3.14159 * (int) (((pd1 / ((float)3.14159)) + 1)/2));
         else 
            pd1 = pd1 - (2 *  (float)3.14159 * (int) (((pd1 / ((float)3.14159)) - 1)/2));

 
         pd2 = -(vel_x2 * kx + vel_y2 * ky)/(float)scale_factor;

         if (pd2 > 0)
            pd2 = pd2 - (2 *  (float)3.14159 * (int) (((pd2 / ((float)3.14159)) + 1)/2));
         else
            pd2 = pd2 - (2 *  (float)3.14159 * (int) (((pd2 / ((float)3.14159)) - 1)/2));

         PIX(po1, width, i, j) = pd1;
         PIX(po2, width, i, j) = pd2;
      }
   }


   /* Now that we know the correct phase differences, we  */
   /* solve for all Fourier components                    */


   for (i= 0; i<width;i++) { 
      for (j= 0; j<height; j++) {
         PIX(ro1,width,i,j) = 0;
         PIX(io1,width,i,j) = 0;
         PIX(ro2,width,i,j) = 0;
         PIX(io2,width,i,j) = 0;
	  }
   }

   for (i=0; i<width; i++) { 
      for (j=0; j<height; j++) {

         // only compute the frequencies inside a band where the phases are not equal
        
         if (    ((((vel_x1 - vel_x2) * (i-width/2))   +  ((vel_y1 - vel_y2) * (j-height/2)) - width ) < 0) 
              &&  (((vel_x1 - vel_x2) * (i-width/2))   +  ((vel_y1 - vel_y2) * (j-height/2)) + width ) > 0)    {
         

            ft0_r = PIX(ri1, width, i, j);
            ft1_r = PIX(ri2, width, i, j);
            ft0_i = PIX(ii1, width, i, j);
            ft1_i = PIX(ii2, width, i, j);

            phi1_r = cos(PIX(po1,width,i,j));
            phi1_i = sin(PIX(po1,width,i,j));
            phi2_r = cos(PIX(po2,width,i,j));
            phi2_i = sin(PIX(po2,width,i,j));
 

            if ((PIX(po1,width,i,j) != PIX(po2,width,i,j))) {  // the phases are equal in the centre of the bank

                                                                                // compute if phi1 != phi2
               multiply_complex(phi1_r, phi1_i,  ft0_r,  ft0_i, &t1_r, &t1_i);     /* ft0 phi1                     */
               subtract_complex( ft1_r,  ft1_i,  t1_r,   t1_i,  &t3_r, &t3_i);     /* ft1 - ft0 phi1               */
               subtract_complex(phi2_r, phi2_i,phi1_r, phi1_i,  &t4_r, &t4_i);     /* phi2 - phi1                  */
               divide_complex  (  t3_r,   t3_i,  t4_r,   t4_i,  &f2_r, &f2_i);     /* ft1 - ft0 phi1 / phi2 - phi1 */
               subtract_complex( ft0_r,  ft0_i,  f2_r,   f2_i,  &f1_r, &f1_i);     /* ft0 - f2                     */
         
               // attenuate if phi2-phi1 is close to zero (i.e. solution above is ill-conditioned)
 
    	         temp2 = fabs(PIX(po1,width,i,j) - PIX(po2,width,i,j));
               if (temp2 > 3.14159) temp2 = fabs(temp2 - (2 * 3.14159));
 
 
               if  (temp2 <= (threshold * v_max)) {
                  scale_factor = sin( ((temp2) /  (threshold * v_max)) * (3.14159/2)) ;
                  scale_factor = scale_factor * scale_factor;
		         }
               else {
                  scale_factor = 1;
               }
  
               f1_r   = f1_r  * scale_factor;
               f1_i   = f1_i  * scale_factor;
               f2_r   = f2_r  * scale_factor;
               f2_i   = f2_i  * scale_factor;
   	      }
         } 
         else {
            f1_r = 0;  f1_i = 0;
            f2_r = 0;  f2_i = 0;
            PIX(po1, width, i, j) = 0;
            PIX(po2, width, i, j) = 0;
 		   }  

         PIX(ro1,width,i,j) = (float)f1_r;
         PIX(io1,width,i,j) = (float)f1_i;
         PIX(ro2,width,i,j) = (float)f2_r;
         PIX(io2,width,i,j) = (float)f2_i;
      }
   }
 
   // create segmented images from Fourier components
  
   fft(output1, ro1, io1, width,height,-1);
   fft(output2, ro2, io2, width,height,-1);
 
 
   /* transfer the processed image to the output images */

   // copying float to int ... need to contrast stretch to allow it to be copied properly
   // only show the results if there was sigificant evidence for it, 
   // i.e. two significant maxima (just one maximum implies no relative motion)

   if (max2 > max1 * (min_max_threshold/100)) { // parameter value is a percentage so convert first

      temp_image = new DVimage(width,height,GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);
 
      temp_image->write(output1); 
      temp_image->contrast_stretch();

      for (i= 0; i<width;i++) { 
         for (j= 0; j<height; j++) {

 			   pixel(output_image_1->idata,width,i,j,0) = (unsigned char) PIX(temp_image->fdata,width,i,j);
			   if (output_image_1->colour_mode == COLOUR_IMAGE) {
 			      pixel(output_image_1->idata,width,i,j,1) = (unsigned char) PIX(temp_image->fdata,width,i,j);
 			      pixel(output_image_1->idata,width,i,j,2) = (unsigned char) PIX(temp_image->fdata,width,i,j);
			   }
	      }
      }



      temp_image->write(output2); 
      temp_image->contrast_stretch();

      for (i= 0; i<width;i++) { 
         for (j= 0; j<height; j++) {

 			   pixel(output_image_2->idata,width,i,j,0) = (unsigned char) PIX(temp_image->fdata,width,i,j);
			   if (output_image_2->colour_mode == COLOUR_IMAGE) {
 			      pixel(output_image_2->idata,width,i,j,1) = (unsigned char) PIX(temp_image->fdata,width,i,j);
 			      pixel(output_image_2->idata,width,i,j,2) = (unsigned char) PIX(temp_image->fdata,width,i,j);
			   }
	      }
      }

 

      if (mask) 
         mask_image (input_image_1, output_image_2, output_image_2, mask_threshold);


      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            temp1 =  PIX(ro1,width,i,j);
            temp2 =  PIX(io1,width,i,j);
            temp_image->put_pixel(i, j, (float) log_magnitude(temp1,temp2),0);
         }
      }
      temp_image->contrast_stretch();
      fourier_output_image_1->write(temp_image->fdata); 


      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            temp1 =  PIX(ro2,width,i,j);
            temp2 =  PIX(io2,width,i,j);
            temp_image->put_pixel(i, j, (float) log_magnitude(temp1,temp2),0);
         }
      }
      temp_image->contrast_stretch();
      fourier_output_image_2->write(temp_image->fdata); 

      delete temp_image;
   }
 
 

   delete image_cps1;
   delete image_cps2;

   if (debug) printf("Leaving fourier_segmentation\n"); 

   return;
  
}
/****************************************************************
* 
*  Routine Name: cross_power_spectrum - compute the inverse Fourier transform
*            of the cross-power spectrum of two images
* 
*      Purpose: The cross-power spectrum of two images is defined as
*            
*             F(w_x, w_y) G*(w_x, w_y)
*            ------------------------
*            |F(w_x, w_y) G(w_x, w_y)|
*
*            where F(w_x, w_y) and G(w_x, w_y) are the Fourier tranforms 
*            of images f(x, y) and g(x, y), and G*(w_x, w_y) is the 
*            complex conjugate of G(w_x, w_y)
*
*            
*       Input: input_1   - pointer to DVimage 
*            input_2   - pointer to DVimage 
*
*      Output: output   - pointer to DVimage representing cross power spectrum
*                     NB it is assumed that this image exists and is of the same dimensions
*                     as the inputs
*
*   Written By: David Vernon
*        Date: April 30, 2005
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications:
*
*
****************************************************************/
 

void cross_power_spectrum (DVimage *input_image_1, DVimage *input_image_2, DVimage *output_image)
{
   static int   width = 0;
   static int   height = 0;
   static int   depth = 0; 
   static float *input1=NULL;
   static float *input2=NULL;
   static float *output1=NULL;
   static float *ri1=NULL;
   static float *ii1=NULL;   
   static float *ri2=NULL;   
   static float *ii2=NULL;
   static float *ro1=NULL;      
   static float *io1=NULL;  

   int w, h, d;
 
   double temp;     
   double  f1_r, f1_i, f2_r, f2_i;   
   double  temp1_r, temp2_r, temp1_i, temp2_i;
   float ftemp;
   int  i, j;
  
   char debug, dump_debug_image;
   
#define PIX(f,width,i,j)   (*((f) + ( (j) * (width) )  + (i) ))

  /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("cross_power_spectrum: debug on \n");  
  

   if (input_image_1 != NULL) {
	   
      input_image_1->get_size(&w,&h);
      d = input_image_1->get_image_mode();
   
      /* create local images for processing but only if there was no change since the previous call */

      if (w != width || h != height || d != depth) {

         width = w; 
         height = h;
         depth = d;

         if (ri1 != NULL) free(ri1);
         if (ri2 != NULL) free(ri2);
         if (ii1 != NULL) free(ii1);
         if (ii2 != NULL) free(ii2);
         if (ro1 != NULL) free(ro1);
         if (io1 != NULL) free(io1);
         if (input1 != NULL) free(input1);
         if (input2 != NULL) free(input2);
         if (output1 != NULL) free(output1);

         input1  =  (float *) malloc(sizeof(float) * width * height);
         input2  =  (float *) malloc(sizeof(float) * width * height);
	      ri1    =   (float *) malloc(sizeof(float) * width * height);
         ii1    =   (float *) malloc(sizeof(float) * width * height);
         ri2    =   (float *) malloc(sizeof(float) * width * height);
         ii2    =   (float *) malloc(sizeof(float) * width * height);
         ro1    =   (float *) malloc(sizeof(float) * width * height);
         io1    =   (float *) malloc(sizeof(float) * width * height);
         output1 =  (float *) malloc(sizeof(float) * width * height); 	      
      }

      if (depth == 1) {
         input_image_1->read(input1);
         input_image_2->read(input2);
      }
      else if (depth == 3) {
         for (i= 0; i<width;i++) { 
            for (j= 0; j<height; j++) {

               ftemp = (pixel(input_image_1->idata,width,i,j,0) +
                        pixel(input_image_1->idata,width,i,j,1) +
                        pixel(input_image_1->idata,width,i,j,2))/(float)3;
 	            PIX(input1,width,i,j) = (unsigned char) ftemp; 
 	
               ftemp = (pixel(input_image_2->idata,width,i,j,0) +
                        pixel(input_image_2->idata,width,i,j,1) +
                        pixel(input_image_2->idata,width,i,j,2))/(float)3; 
 	            PIX(input2,width,i,j) = (unsigned char) ftemp; 	
	         }
         }
      }

	  fft(input1, ri1, ii1, width, height,1);
      fft(input2, ri2, ii2, width, height,1); 

      /* now compute the cross power spectrum */
 
      for (i= 0; i<width; i++) {
        for (j= 0; j<height; j++) {

          f1_r = PIX(ri1,width,i,j);
          f1_i = PIX(ii1,width,i,j);
          f2_r = PIX(ri2,width,i,j);
          f2_i = PIX(ii2,width,i,j);  

          multiply_complex(f1_r, f1_i, f2_r, -f2_i, &temp1_r, &temp1_i);
          multiply_complex(f1_r, f1_i, f2_r,  f2_i,  &temp2_r, &temp2_i);

          temp = _magnitude(temp2_r, temp2_i);

          if (temp != 0) {
            PIX(ro1,width,i,j) = (float) (temp1_r / temp);
            PIX(io1,width,i,j) = (float) (temp1_i / temp);
			 }
		  }
	   }

      /* inverse FFT */
 
      fft(output1, ro1, io1, width,height,-1);

      /* swap quadrants to get origin at width/2, height/2 */

      for (i=0; i<width/2; i++) {
        for (j=0; j<height; j++) {
  
		    ftemp = PIX(output1,width,i,j);
		    PIX(output1,width,i,j) = PIX(output1,width,(i+width/2)%width,j);
          PIX(output1,width,(i+width/2)%width,j) = ftemp;
		  }
	   }
      for (i=0; i<width; i++) {
        for (j=0; j<height/2; j++) {
  
		    ftemp = PIX(output1,width,i,j);
		    PIX(output1,width,i,j) = PIX(output1,width,i,(j+height/2)%height);
          PIX(output1,width,i,(j+height/2)%height) = ftemp;
		  }
	   }



      if (dump_debug_image) {
        //dump_float_image(output1, width, height);
	   }
 
     /* transfer the processed image to the output images */

     if (output_image != NULL)
	    output_image->write(output1); 
 
   }
     
   if (debug) printf("Leaving cross_power_spectrum\n\n");

   return;
   
}



/****************************************************************
* 
*  Routine Name: find_maxima - find the first n maxima in an image
*
*            
*       Input: input                     pointer to DV GREYSCALE image, either float or int
*            number_of_maxima_required      int
*            non_maxima_suppression_radius   radius of region in which to supress other
*                                    maxima in which a maximum has already been located
*
*
*      Output: maxima                    array of maxima_data structures
*                                    this array must exist before calling this function
*                                    and it must have at least number_of_maxima_required elements
*
*   Written By: David Vernon
*        Date: April 29, 2005
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications: 
*
*
****************************************************************/
 

void find_maxima (DVimage *input_image, int number_of_maxima_required, int non_maxima_suppression_radius, maxima_data_type maxima[])
{
   int height;
   int width;
   int x1, y1;
   float *input1=NULL;
   int  i, j, k;
   float max1;

   char debug, dump_debug_image;


   
#define PIX(f,width,i,j)   (*((f) + ( (j) * (width) )  + (i) ))

  /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("find_maxima: debug on \n");  
 
   if (input_image != NULL) { 

     if (input_image->get_image_mode() != GREYSCALE_IMAGE) {
       printf("find_maxima: error - image is not greyscale\n");
		   return;
     }
     
	   input_image->get_size(&width,&height);

     input1 =  (float *) malloc(sizeof(float) * width * height);
     input_image->read(input1);
 
	   for (k=0; k<number_of_maxima_required; k++) {

	     // find maximum

       max1 = -1e10;		
       
       for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            if (PIX(input1,width,i,j) > max1) {
              max1 = (float) PIX(input1,width,i,j);
              x1 = i;
              y1 = j;
            }
         } 
       }


	    // suppress all data in the non-maxima suppression region

		 if( max1 != -1e10) {
         for (i=x1-non_maxima_suppression_radius; i<x1+non_maxima_suppression_radius; i++) {
            for (j=y1-non_maxima_suppression_radius; j<y1+non_maxima_suppression_radius; j++) {
              if ((i>=0 && i<width) && (j>=0 && j<height)) PIX(input1,width,i,j) = -1e10; 
			   }
			} 

	      maxima[k].x = x1;
	      maxima[k].y = y1;
	      maxima[k].value = max1;
     
		   if (debug) {
            printf("max1 = %6.0f at (%3d, %3d)\n", max1, x1, y1);
			}
       }
     }

 
     /* free up all allocated space, i.e., release local image space */
 
     free(input1);
   }
 
 
   if (debug) printf("Leaving find_maxima\n\n");

   return;
   
}


 

/***************************************/
/*                            */
/*   sort_fourier_components         */
/*                            */
/***************************************/

void sort_fourier_components
               (float *ro1, float *io1, 
               float *ro2, float *io2, 
               float *po1, float *po2,
               int width, int height, int correct_phase, 
               FILE *velocity_info_file,
               float *vel_x1, float *vel_y1, float *vel_x2, float *vel_y2)
{


   int   ht_width, ht_height, half_ht_width, half_ht_height;      
   int   *ht_r=NULL;
   int   *ht_p=NULL;

   int   half_width, half_height;

  int   i, j, k, l, kx,ky;
  char debug, dump_debug_image;

  float  vx1, vy1, vx2, vy2, vx_lb, vx_ub, vy_lb, vy_ub, v1, v2, ftemp;
  float  pd1, pd2;

  int temp, kx_lb, ky_lb, kx_ub, ky_ub;
  float phase_difference;
  float scale_factor;
  float max1, max2;
  float tempf, mid_value;
  int x1, y1, x2, y2;
  float pi_by_2;
  int image_number;
  int mid_x, mid_y;
  float m,c, adjusted_c;
  float mid_value_drop;
  float v_inc;

 
#define D1 1         /* non-maxima suppression radius                */
#define D2 9         /* inhibition radius: required separation in H space */
       


#define ZERO_TOLERANCE 0.01

   pi_by_2 = (float)(2 * 3.14159);

   mid_value_drop = (float)(0.9); /* fraction of 2nd maximum value required of the */
                          /* mid-point between both maxima;            */
                          /* purpose of this is to inhibit smeared       */
                          /* maximum being detected twice              */

                     /* 0.6 */



  /* set debug flag */

   debug = FALSE;
   dump_debug_image = FALSE;

   if (debug || dump_debug_image) 
     printf("sort_fourier_components: debug on \n"); 

   /* first we create the Hough accumulators                        */
   /* the size of which will depend on the maximum velocity to be detected */
   /* we will create the accumulator to be the  max velocity * 10 * 2     */
   /* (i.e. max velocity per image times 2                         */
   /* because we must cater for + and - the maximum velocity)           */
   /* we will assume that the maximum velocity is 10% of the field of view */


   ht_width = width * 2;
   ht_height = height * 2; 

   ht_r =  (int *) malloc(sizeof(int) * ht_width * ht_height);
   ht_p =  (int *) malloc(sizeof(int) * ht_width * ht_height);

   scale_factor = (width * 10) / (float)(2 * 3.14159);

   /* initialize Hough Transform accumulators to zero */

   for (i=0; i<ht_width; i++) 
     for (j=0; j<ht_height; j++) {
        PIX(ht_r,ht_width,i,j) =  0;
        PIX(ht_p,ht_width,i,j) =  0;
     }

   /* compute Hough Transform */

   kx_lb =  -width / 2; 
   kx_ub =   width / 2;
   ky_lb =  -height / 2; 
   ky_ub =   height / 2;

   vx_lb = -pi_by_2 / 10;
   vx_ub =  pi_by_2 / 10;
   vy_lb = -pi_by_2 / 10;
   vy_ub =  pi_by_2 / 10;

   v_inc = vx_ub / (float) width;  

   half_ht_width = ht_width/2;
   half_ht_height = ht_height/2;
   half_width = width/2;
   half_height = height/2;

   /* process the phase change image no. 1 and image no. 2  */

   for (image_number = 1; image_number <= 2; image_number++) {

     /* NB we only sub-sample the spatial frequencies when computing the velocity */

     for (kx = kx_lb; kx < kx_ub; kx+=2) {
       for (ky = ky_lb; ky < ky_ub; ky+=2) {

         i = kx + half_width;
         j = ky + half_height;
            
         if (image_number == 1)
            phase_difference=PIX(po1,width,i,j);
         else
            phase_difference=PIX(po2,width,i,j);

          if ((phase_difference < -ZERO_TOLERANCE) || 
            (phase_difference > +ZERO_TOLERANCE)) {

            /* audit trail: altered code to use a different loop structure */
            /* if ky > kx to avoid problems with truncation of the       */
            /* computed vy component to zero when ky becomes large.      */

            if (ky > kx) {

              if (ky != 0) {
                m = -(float)kx/(float)ky;
                c = -phase_difference/ky;
                adjusted_c = c * scale_factor + half_ht_height;
              

                for (k=0; k<ht_width; k++) {
                  if (ky != 0) {
 
                     l = (int)(m*(k-half_ht_width) + adjusted_c);

                     if ((l >= 0) && (l < ht_height)) {
                       PIX(ht_r,ht_width,k,l) = PIX(ht_r,ht_width,k,l) + 1;
                     }
                  }
                }
   
              }
            }

            else {
        
              if (kx != 0) {
                m = -(float)ky/(float)kx;
                c = -phase_difference/kx;
                adjusted_c = c * scale_factor + half_ht_width;
 
                for (l=0; l<ht_height; l++) {
                  if (kx != 0) {

                     k = (int)(m*(l-half_ht_height) + adjusted_c);

                     if ((k >= 0) && (k < ht_width)) {
                       PIX(ht_r,ht_width,k,l) = PIX(ht_r,ht_width,k,l) + 1;
                     }
                  }
                }
   
              }
            }
         }
       }
     }
   }


   /* now that the Hough Transform has been computed, we need to proceed to: */
   /*                                                      */
   /* 1. Find the principal two maxima in the Hough velocity space         */
   /* 2. Sort the components into two sets depending on whether they satisfy */
   /*   the phase/frequency relationship for the first velocity or the     */
   /*   second                                               */

   /* New code now added to cater for a problem with the variation of the   */
   /* HT maxima values with velocity.  Specifically, we note that the      */
   /* HT becomes smeared as the velocity of the image(s) increases.        */
   /* Consequently, we are capable of detecting two maxima caused by the    */
   /* one image which are separated by more than the lateral inhibition     */
   /* radius.  As a result, the maximum corresponding to the second image   */
   /* is missed.  To counter this, we require that the mid-point between    */
   /* the two maxima is less than xx% of the smaller of the two maxima      */



   for (i=0; i<ht_width; i++) {
     for (j=0; j<ht_height; j++) {
       PIX(ht_p,ht_width,i,j) = PIX(ht_r,ht_width,i,j);
     }
   }

   
   max1 = 0;  max2 = 0; 
   x1 = 0; y1 = 0; x2 = 0; y2 = 0; 

   for (i=0; i<ht_width; i++) {
     for (j=0; j<ht_height; j++) {
       if (PIX(ht_p,ht_width,i,j) > max1) {
         max1 = (float) PIX(ht_p,ht_width,i,j);
         x1 = i;
         y1 = j;
       }
     }
   }

   for (i=0; i<ht_width; i++) {
     for (j=0; j<ht_height; j++) {
       if ((PIX(ht_p,ht_width,i,j) > max2) && 
         (( (i < (x1-D2)) || (i > (x1+D2)) ) ||
          ( (j < (y1-D2)) || (j > (y1+D2)) )   ) ) {
               
         /* now check that we are not dealing with a smeared pair of */
         /* maxima both of which derive from the one object        */
         /* we do this by checking the mid-point between both      */
         /* maxima and ensuring it is less than xx% of the both max  */     

         mid_x = (x1+i)/2;
         mid_y = (y1+j)/2;
         mid_value =  (float) PIX(ht_p,ht_width,mid_x, mid_y);
 
         
         if (mid_value < mid_value_drop * (PIX(ht_p, ht_width, i, j))) {
            max2 = (float) PIX(ht_p,ht_width,i,j);
            x2 = i;
            y2 = j;


            if (debug) {
              printf("max1 = %f x1, y1 = %d, %d; max2 = %f x2, y2 = %d, %d \n", 
              max1, x1, y1, max2, x2, y2);
              printf("mid point = %f at %d, %d \n", 
              mid_value, mid_x, mid_y);
            }
         }
       }
     }
   } 

   if (max2 < (max1/10)) {

     /* we assume that the second maximum is spurious and that the real second */
     /* maximum is at the origin and has been supressed (zero phase change)   */

     x2 = ht_width/2;
     y2 = ht_height/2;
   }




   /* we now have the two maxima and the corresponding velocities */
   /* before  proceeding, we wish to ensure that the first maxima */
   /* corresponds not to the greater of the two maxima but to the */
   /* greater of the two velocities (and, hence, by assumption,   */
   /* to the foreground object).                          */

   /* if velocity 1 < velocity 2 then swap velocities and maxima  */

   
   v1 = (x1 - (float)(ht_width/2)) *  (x1 - (float)(ht_width/2)) +
      (y1 - (float)(ht_height/2)) *  (y1 - (float)(ht_height/2));   /* squared */
   
   v2 = (x2 - (float)(ht_width/2)) *  (x2 - (float)(ht_width/2)) +
      (y2 - (float)(ht_height/2)) *  (y2 - (float)(ht_height/2));   /* squared */

   if (v1 < v2) {
     temp = x1; x1 = x2; x2 = temp;
     temp = y1; y1 = y2; y2 = temp;
     ftemp = max1; max1 = max2; max2 = ftemp;     
   }


   if (debug) {
     printf("max1 = %f x1, y1 = %d, %d; max2 = %f x2, y2 = %d, %d \n", max1, x1, y1, max2, x2, y2);
     mid_x = (x1+x2)/2;
     mid_y = (y1+y2)/2;
     mid_value =  (float)PIX(ht_p,ht_width,mid_x, mid_y);
     printf("mid point = %f at %d, %d \n", 
           mid_value, mid_x, mid_y);
   }



   /* compute accumulator sum in 3x3 region around maxima        */
   /* This is used to estimate confidence in velocity measurement  */


   max1 = 0;
   for (i=x1-1; i<=x1+1; i++) {
     for (j=y1-1; j<=y1+1; j++) {

       if ( (i >=0 ) &&  (i < ht_width) &&
           (j >= 0) &&  (j < ht_height) ) {

         max1 += PIX(ht_p,ht_width,i,j);
       }
     }
   }

   max2 = 0;
   for (i=x2-1; i<=x2+1; i++) {
     for (j=y2-1; j<=y2+1; j++) {

       if ( (i >=0 ) &&  (i < ht_width) &&
           (j >= 0) &&  (j < ht_height) ) {

         max2 += PIX(ht_p,ht_width,i,j);
       }
     }
   }



   if (velocity_info_file != NULL) {
     fprintf(velocity_info_file, "%f %f %f %f %f %f", 
               ((float)(x1-ht_width/2))/10,  ((float)(y1-ht_height/2))/10, 
               ((float)(x2-ht_width/2))/10,  ((float)(y2-ht_height/2))/10,
               (max1 * 100)/(max1+max2), (max2 *100)/(max1+max2));
   }
   
  

   /* now run through each spatial frequency and sort the components into  */
   /* one of two sets depending on whether its phase change is closer to   */
   /* the first computed velocity or the second                     */

   vx1 = ((float)(x1) -  (float)(ht_width / 2));
   vy1 = ((float)(y1) -  (float)(ht_width / 2));

   vx2 = ((float)(x2) -  (float)(ht_width / 2));
   vy2 = ((float)(y2) -  (float)(ht_width / 2));

   for (kx = kx_lb; kx < kx_ub; kx++) {
     for (ky = ky_lb; ky < ky_ub; ky++) {

       i = kx + (width / 2);
       j = ky + (height / 2);   

       /* compute correct phase differences and force phase wrapping */
       /* to facilitate comparison with the image phase differences  */

       pd1 = -(vx1 * kx + vy1 * ky)/scale_factor;

       if (pd1 > 0)
         pd1 = pd1 - (2 * (float)3.14159 * (int) (((pd1 / (float)(3.14159)) + 1)/2));
       else 
         pd1 = pd1 - (2 * (float)3.14159 * (int) (((pd1 / (float)(3.14159)) - 1)/2));

       pd2 = -(vx2 * kx + vy2 * ky)/scale_factor;

       if (pd2 > 0)
         pd2 = pd2 - (2 * (float)3.14159 * (int) (((pd2 / (float)(3.14159)) + 1)/2));
       else
         pd2 = pd2 - (2 * (float)3.14159 * (int) (((pd2 / (float)(3.14159)) - 1)/2));

       if ((PIX(po1,width,i,j) !=  0) || (PIX(po2,width,i,j) != 0)) {

         /* both phase changes aren't zero so we must have solved for this frequency */

         if (fabs(pd1-PIX(po1,width,i,j)) > fabs(pd2 - PIX(po1,width,i,j))) {

            /* swap components */

            tempf = PIX(ro2,width,i,j); 
            PIX(ro2,width,i,j) = PIX(ro1,width,i,j);
            PIX(ro1,width,i,j) = tempf;

            tempf = PIX(io2,width,i,j); 
            PIX(io2,width,i,j) = PIX(io1,width,i,j);             
            PIX(io1,width,i,j) = tempf;

            tempf = PIX(po2,width,i,j); 
            PIX(po2,width,i,j) = PIX(po1,width,i,j);             
            PIX(po1,width,i,j) = tempf;
 
            /* assign correct phase */
   
            if (correct_phase) {
              PIX(po2,width,i,j) =  pd2;  
              PIX(po1,width,i,j) =  pd1;
            }
         }
         else  {

            /* else just correct the phase */

            if (correct_phase) {
              PIX(po2,width,i,j) = pd2;  
              PIX(po1,width,i,j) = pd1;
            }
         }

       }
       else {

          /* insert the correct phase even though we didn't solve for it */
            
         if (correct_phase) {

            PIX(po2,width,i,j) = pd2;  
            PIX(po1,width,i,j) = pd1;
         }
       }

     }
   }



   /* return the computed velocities */

   *vel_x1 = ((float)(x1-ht_width/2))/10;
   *vel_y1 = ((float)(y1-ht_height/2))/10;
   *vel_x2 = ((float)(x2-ht_width/2))/10;
   *vel_y2 = ((float)(y2-ht_height/2))/10;
   
   free(ht_r);
   free(ht_p);
}



/****************************************************************
* 
*  Routine Name: plot_field  
* 
*   Written By: D. Vernon, Department of Computer Science, Maynooth College, Ireland
*        Date: Nov 13, 1996
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications: Added arrow heads.  D.V. 9/1/97
****************************************************************/
 
int plot_field(DVimage *f_mag, DVimage *f_phase, DVimage **plot_image, float scale_factor, int colour)
 
{ 

#define ARROW_HEAD_SIZE (4.0)
#define ARROW_HEAD_ANGLE (3.14159 / 4.0)
 
   int width, height;   
   float *f_m=NULL;   
   float *f_p=NULL;   
   float *plot=NULL;  
   int   i, j, i2, j2, i3, j3;
   double i_offset, j_offset, theta;
   char debug;
 

   /* set debug flag */

	debug = FALSE;
	if (debug) printf("plot_field: debug on \n");  

	// find size of the images

   f_mag->get_size(&width,&height);


	// allocate space

	f_m  =  (float *) malloc(sizeof(float) * width * height);
	f_p  =  (float *) malloc(sizeof(float) * width * height);
	plot  =  (float *) malloc(sizeof(float) * width * height);


	// read the images

	f_mag->read(f_m);
	f_phase->read(f_p);
 

	if (debug) printf("plot_field: width %d  height %d \n", width, height);
	
	// create the output image

	if (*plot_image != NULL) {
		delete *plot_image;
      *plot_image = NULL;
	}
	*plot_image = new DVimage(width,height,GREYSCALE_IMAGE,NULL, NULL,DVFLOAT);


   /* Now draw vectors for all non-zero points in the magnitude image */


   for (i=0; i<width; i++) {
     for (j=0; j<height; j++) {
        if (PIX(f_m, width, i, j) > 0) {

          i_offset = PIX(f_m,width,i,j) * cos(PIX(f_p,width,i,j)) *
              scale_factor;
          j_offset = PIX(f_m,width,i,j) * sin(PIX(f_p,width,i,j)) *
              scale_factor;

          i2 = i + (int)(i_offset);
          j2 = j + (int)(j_offset);

          if ((i2 >= 0) && (i2 < width) && (j2 >= 0) && (j2 < height)) {

            //draw_line(plot,width,i,j,i2,j2,colour);
          
            /* add arrow head */

            theta = PIX(f_p,width,i,j) + 3.14159 + ARROW_HEAD_ANGLE;
            i_offset = ARROW_HEAD_SIZE * cos(theta);
            j_offset = ARROW_HEAD_SIZE * sin(theta);

            i3 = i2 + (int)(i_offset);
            j3 = j2 + (int)(j_offset);

            //draw_line(plot,width,i2,j2,i3,j3,colour);

            theta = PIX(f_p,width,i,j) + 3.14159 - ARROW_HEAD_ANGLE;
            i_offset = ARROW_HEAD_SIZE * cos(theta);
            j_offset = ARROW_HEAD_SIZE * sin(theta);;

            i3 = i2 + (int)(i_offset);
            j3 = j2 + (int)(j_offset);

            //draw_line(plot,width,i2,j2,i3,j3,colour);
 
          }
        }
      }
   }
 
  
   (*plot_image)->write(plot);

   /* free up all allocated space, i.e., release local image space */

   free(f_p);
   free(f_m);
   free(plot);

   return(TRUE);
   
}


/****************************************************************
* 
*   Routine Name: plot_field
* 
*   Written By: D. Vernon, Department of Computer Science, Maynooth College, Ireland
*   Date: Nov 13, 1996
*
*   Modifications: 
*
*   - Added arrow heads.  D.V. 9/1/97
*   - Use new draw_line() with capability to draw on RGB images DV  1/4/2011
*     NB: the plot image MUST be a colour image
*                
****************************************************************/
 
int plot_field(DVimage *f_mag, DVimage *f_phase, DVimage *plot_image, float scale_factor, int red, int green, int blue)
 
{ 

#define ARROW_HEAD_SIZE (4.0)
#define ARROW_HEAD_ANGLE (3.14159 / 4.0)
 
   int width, height, depth;   
   unsigned char *plot=NULL;  
   int   i, j, i2, j2, i3, j3;
   double i_offset, j_offset, theta;
   float magnitude_value;
   float phase_value;
   char debug;
 
   /* set debug flag */

	debug = FALSE;
	if (debug) printf("plot_field: debug on \n");  

	// find size of the images

    f_mag->get_size(&width,&height);

    depth = plot_image->get_image_mode();

    if (depth != COLOUR_IMAGE) {
       printf("plot_field: Error ... not plotting on a colour image\n");
       return(0);
    }

	// allocate space

	plot  =  (unsigned char *) malloc(sizeof(unsigned char) * width * height * depth);

	// read the image

    plot_image->read(plot);

	if (debug) printf("plot_field: width %d  height %d depth %d\n", width, height, depth);
	
   /* Now draw vectors for all non-zero points in the magnitude image */


   for (i=0; i<width; i++) {
     for (j=0; j<height; j++) {
        f_mag->get_pixel(i, j, &magnitude_value);
        if (magnitude_value > 0) {

          f_phase->get_pixel(i, j, &phase_value);

          i_offset = magnitude_value * cos(phase_value) *
              scale_factor;

          j_offset = magnitude_value * sin(phase_value) *
              scale_factor;

          i2 = i + (int)(i_offset);
          j2 = j + (int)(j_offset);

          if ((i2 >= 0) && (i2 < width) && (j2 >= 0) && (j2 < height)) {

            draw_line(plot, width, height, i, j, i2, j2, 
                      red, red, green, green, blue, blue, // RGB values at start and end of line
                      1.0, 1.0);                          // x and y scale values 

            /* add arrow head */

            theta = phase_value + 3.14159 + ARROW_HEAD_ANGLE;
            i_offset = ARROW_HEAD_SIZE * cos(theta);
            j_offset = ARROW_HEAD_SIZE * sin(theta);

            i3 = i2 + (int)(i_offset);
            j3 = j2 + (int)(j_offset);

            draw_line(plot, width, height, i2, j2, i3, j3, 
                      red, red, green, green, blue, blue, // RGB values at start and end of line
                      1.0, 1.0);                                      // x and y scale values 

            theta = phase_value + 3.14159 - ARROW_HEAD_ANGLE;
            i_offset = ARROW_HEAD_SIZE * cos(theta);
            j_offset = ARROW_HEAD_SIZE * sin(theta);;

            i3 = i2 + (int)(i_offset);
            j3 = j2 + (int)(j_offset);

            draw_line(plot, width, height, i2, j2, i3, j3, 
                      red, red, green, green, blue, blue, // RGB values at start and end of line
                      1.0, 1.0);                                      // x and y scale values 

          }
        }
      }
   }
 
  
   plot_image->write(plot);

   /* free up all allocated space, i.e., release local image space */

   free(plot);

   return(TRUE);
  
}


/*****************************************************************************
 *  
 * ---  Subprogram Name:   draw_line() 
 * 
 * ---  Functional Description:  
 * 
 *     This function draws a line in an intensity image, at a specified  
 *     shade
 * 
 * ---  Libraries and External Sub-Programs Referenced: 
 * 
 *
 * ---  Calling Procedure:
 *
 *      draw_line(image,usx1,usy1,usx2,usy2,shade,scale_x,scale_y);
 *
 * ---  Input Parameters:  
 * 
 *     image                 The intensity image
 *     width, height, depth 
 *     usx1, usy1            The co-ordinates of the start line
 *     usx2, usy2            The co-ordinates of the end of the line
 *     rshade1, rshade2      The red component of the line colour at start and end of the line
 *     gshade1, gshade2      The green component of the line colour at start and end of the line
 *     bshade1, bshade2      The blue component of the line colour at start and end of the line
 *     scale_x,scale_y       The scale factors to use for the co-ordinates.
 *
 * ---  Output Parameters:
 *
 *     image ........ The intensity image (with the line drawn).
 *
 * ---  Global Parameters: none. 
 * 
 * ---  Local Variables:
 * 
 *     x,y,r,count,diffx,diffy, y_inc, x_inc 
 *     param1,param2
 *     x1,y1,x2,y2 
 * 
 * ---  Bugs:  
 * 
 * ---  Author: Sean O'Neill, TCD. 
 *
 * ---  Revisions
 * 
 *     date:     17/12/86, Ken Dawson, tcd. 
 *     revision: Alterations to cope with scaling,shading and use with intensity images. 
 *     reason:
 *            
 *     date:     9/ 1/89, Ken Dawson, tcd. 
 *     revision: Inclusion of multiple shading levels. 
 *     reason:   To allow a basic representation of depth 
 *  
 *     date:     22/ 5/89, Ken Dawson, tcd.
 *     revision: Rewrite of entire routine - same functionality...
 *     reason:   Correction of some apparent errors and making the code
 *               into an understandable form.  
 * 
 *     date:     21/03/04, David Vernon 
 *     revision: Draw in colour in an RGB image
 *     reason:   Port to FLTK example program 
 * 
 *     date:     29/03/04, David Vernon 
 *     revision: check coordinate to make sure they within image bounds
 *     reason:   Port to FLTK example program 
 *
 *****************************************************************************/

void draw_line(unsigned char *image, int width, int height, int usx1,int usy1,int usx2, int usy2, 
		       int rshade1, int rshade2, int gshade1, int gshade2, int bshade1, int bshade2,
			   double scale_x, double scale_y)
{
   double x1,y1,x2,y2,lengthx,lengthy,length,x_increment,y_increment,x,y,
	   rshade_increment, gshade_increment, bshade_increment,
	   rshade, gshade, bshade;
   int temp1,temp2,i;

   temp1 = 0;  /* image->window.x1; */
   temp2 = 0;  /* image->window.y1; */

   x1 = (((double) usx1 * scale_x) + (double) temp1);
   y1 = (((double) usy1 * scale_y) + (double) temp2);
   x2 = (((double) usx2 * scale_x) + (double) temp1);
   y2 = (((double) usy2 * scale_y) + (double) temp2);

   if (x1<0) x1=0; 
   if (x1>=width) x1=width-1;
   if (x2<0) x2=0; 
   if (x2>=width) x2=width-1;
   if (y1<0) y1=0; 
   if (y1>=height) y1=height-1;
   if (y2<0) y2=0; 
   if (y2>=height) y2=height-1;

   if ((lengthx = (x2 - x1)) < 0.0)
    lengthx = -lengthx;
   if ((lengthy = (y2 - y1)) < 0.0)
    lengthy = -lengthy;
   if (lengthx > lengthy)
    length = lengthx;
    else length = lengthy;
   x_increment = (x2-x1) / length;
   y_increment = (y2-y1) / length;
   rshade_increment = ((double) rshade2 - (double) rshade1) / length;
   rshade = ((double) rshade1) + 0.5;
   gshade_increment = ((double) gshade2 - (double) gshade1) / length;
   gshade = ((double) gshade1) + 0.5;
   bshade_increment = ((double) bshade2 - (double) bshade1) / length;
   bshade = ((double) bshade1) + 0.5;

   x = x1 + 0.5;
   y = y1 + 0.5;

   for(i=0; (i < ((int) length)); i++) {

      pixel(image, width, (int)x, (int)y, 0) = (unsigned char) rshade; // red shade
      pixel(image, width, (int)x, (int)y, 1) = (unsigned char) gshade; // green shade
      pixel(image, width, (int)x, (int)y, 2) = (unsigned char) bshade; // blue shade

      x = x + x_increment;
      y = y + y_increment;
      rshade = rshade + rshade_increment;
	  gshade = gshade + gshade_increment;
      bshade = bshade + bshade_increment;
   }
}

 


/****************************************************************
* 
*  Routine Name: interpolate2
* 
*  Purpose: create an output image by performing bi-linear interpolation
*           between uniformly-sampled data.  Sampling period is assumed 
*           to be the same in both x and y directions.
*
*  Input: sampled_image - uniformly sampled input data
*
*  Output: interpolated_image  - interpolated output data
*
*  Returns: TRUE (1) on success, FALSE (0) otherwise
*
*  Restrictions: Restrictions on data or input as applicable
*  Written By: D. Vernon, Department of Computer Science, 
*              Maynooth College, Ireland.
*  Date: Jan 09, 1997
*  Modifications: made last_i = width - first_i - grid_spacing
*                 rather than searching for it.  Same for last_j
*                 DV 12/12/00
*
****************************************************************/
 
int interpolate2(DVimage *sampled_image, DVimage *interpolated_image)
{
   int   width, height;  
   int   i, j, k, l, found, 
         first_i, first_j, 
         last_i, last_j, 
         grid_spacing;
   char  debug;
   float a, b, c, d, p, q;
   float pixel_value;
   float pixel_value1;
   float pixel_value2;
   float pixel_value3;
   float pixel_value4;

   /* set debug flag */

   debug = FALSE;
   if (debug) printf("interpolate: debug on \n");  
 
   // find size of the images

   sampled_image->get_size(&width,&height);
   
   /* initialize output */

   for (i=0; i<width; i++) 
     for (j=0; j<height; j++) 
        interpolated_image->put_pixel(i,j,(float)0.0);

   /* find first point */

   found = FALSE;
   first_i = 0;
   first_j = 0;
   for (i=0; (i<width) && (!found); i++) 
      for (j=0; (j<height) && (!found); j++) {
         sampled_image->get_pixel(i,j,&pixel_value);

         if (pixel_value > 0) {
            found = TRUE;
            first_i = i;
            first_j = j;
         }
      }

   if (!found) {
     if (debug) printf("interpolate: no sample points\n");
     return(FALSE);
   }

   /* find grid spacing */

   if (width > height)
      grid_spacing = width;
   else 
      grid_spacing = height;

   for (i=0; i<width; i++) 
     for (j=0; j<height; j++) {
       sampled_image->get_pixel(i,j,&pixel_value);
       if (pixel_value > 0) {
         if (i != first_i && j != first_j) {
            if (abs(i-first_i) < grid_spacing) {
              grid_spacing = abs(i-first_i);
            }

            if (abs(j-first_j) < grid_spacing) {
              grid_spacing = abs(j-first_j);
            }
         }
       }
     }

   while (first_i - grid_spacing > 0)   
     first_i = first_i - grid_spacing;

   while (first_j - grid_spacing > 0)   
     first_j = first_j - grid_spacing;

   last_i = first_i;
   last_j = first_j;

   while (last_i + grid_spacing < width)   
     last_i = last_i + grid_spacing;

   while (last_j + grid_spacing < height)   
     last_j = last_j + grid_spacing;

   if (debug) 
     printf("interpolate2: first_i,j = %d, %d; last_i,j = %d, %d; grid_spacing = %d", first_i, first_j, last_i, last_j, grid_spacing);

   for (i=first_i; i<last_i; i+=grid_spacing) {
     for (j=first_j; j<last_j; j+=grid_spacing) {

       sampled_image->get_pixel(i,j,&pixel_value1);
       sampled_image->get_pixel(i,j+grid_spacing,&pixel_value2);
       sampled_image->get_pixel(i+grid_spacing,j,&pixel_value3);
       sampled_image->get_pixel(i+grid_spacing,j+grid_spacing,&pixel_value4);

       a = pixel_value3 - pixel_value1;
       b = pixel_value2 - pixel_value1;
       c = pixel_value4 
          + pixel_value1
          - pixel_value3
          - pixel_value2;
       d = pixel_value1;

       if (debug) 
         printf("interpolate: a = %f, b = %f, c = %f, d = %f\n", a, b, c, d);

       for (k=i; k <= i+grid_spacing; k++) {
         for (l=j; l<= j+grid_spacing; l++) {

            p = (float)(k - i) / (float)grid_spacing;
            q = (float)(l - j) / (float)grid_spacing;

            interpolated_image->put_pixel(k, l, a*p + b*q + c*p*q + d);
         }
       }
     }
   }

   return(TRUE);
}



/****************************************************************
* 
*  Routine Name: interpolate
* 
*  Purpose: create an output image by performing bi-linear interpolation
*           between uniformly-sampled data.  Sampling period is assumed 
*           to be the same in both x and y directions.
*
*  Input: sampled_image - uniformly sampled input data
*
*  Output: interpolated_image  - interpolated output data
*
*  Returns: TRUE (1) on success, FALSE (0) otherwise
*
*  Restrictions: Restrictions on data or input as applicable
*  Written By: D. Vernon, Department of Computer Science, 
*              Maynooth College, Ireland.
*  Date: Jan 09, 1997
*  Modifications: made last_i = width - first_i - grid_spacing
*                 rather than searching for it.  Same for last_j
*                 DV 12/12/00
****************************************************************/
 
int interpolate(DVimage *sampled_image, DVimage **interpolated_image)

{
   int width, height;  

   float *f1=NULL;
   float *f2=NULL;
   int   i, j, k, l, found, 
         first_i, first_j, 
         last_i, last_j, 
         grid_spacing;
   char debug;
   float a, b, c, d, p, q;


  /* set debug flag */

   debug = FALSE;
   if (debug) printf("intpolate: debug on \n");  
 
 	// find size of the images

   sampled_image->get_size(&width,&height);


	// allocate space

	f1  =  (float *) malloc(sizeof(float) * width * height);
	f2  =  (float *) malloc(sizeof(float) * width * height);

	// read the input sampled image

	sampled_image->read(f1);
   
	// create the output image

	if (*interpolated_image != NULL) {
		delete *interpolated_image;
        *interpolated_image = NULL;
	}
	*interpolated_image = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);


   /* initialize output */

   for (i=0; i<width; i++) 
     for (j=0; j<height; j++) 
        PIX(f2,width,i,j) =  0.0;


   /* find first point */

   found = FALSE;
   first_i = 0;
   first_j = 0;
   for (i=0; (i<width) && (!found); i++) 
     for (j=0; (j<height) && (!found); j++) 
       if (PIX(f1,width,i,j) > 0) {
         found = TRUE;
         first_i = i;
         first_j = j;
       }


   if (!found) {
     printf("interpolate: Error - no sample points");
     return(FALSE);
   }

 

   /* find grid spacing */

   if (width>height)
     grid_spacing = width;
   else 
     grid_spacing = height;

   for (i=0; i<width; i++) 
     for (j=0; j<height; j++) 
       if (PIX(f1,width,i,j) > 0) {
         if (i != first_i && j != first_j) {
            if (abs(i-first_i) < grid_spacing) {
              grid_spacing = abs(i-first_i);
            }

            if (abs(j-first_j) < grid_spacing) {
              grid_spacing = abs(j-first_j);
            }
         }
       }


   while (first_i - grid_spacing > 0)   
     first_i = first_i - grid_spacing;

   while (first_j - grid_spacing > 0)   
     first_j = first_j - grid_spacing;

   last_i = first_i;
   last_j = first_j;

   while (last_i + grid_spacing < width)   
     last_i = last_i + grid_spacing;

   while (last_j + grid_spacing < height)   
     last_j = last_j + grid_spacing;



   if (debug) 
     printf("interpolate: first_i,j = %d, %d; last_i,j = %d, %d; grid_spacing = %d", first_i, first_j, last_i, last_j, grid_spacing);

   for (i=first_i; i<last_i; i+=grid_spacing) {
     for (j=first_j; j<last_j; j+=grid_spacing) {
       a = PIX(f1, width, i+grid_spacing, j) - PIX(f1, width, i, j);
       b = PIX(f1, width, i, j+grid_spacing) - PIX(f1, width, i, j);
       c = PIX(f1, width, i+grid_spacing, j+grid_spacing)  
          + PIX(f1, width, i, j)
          - PIX(f1, width, i+grid_spacing, j) 
          - PIX(f1, width, i, j+grid_spacing);
       d = PIX(f1, width, i, j);

       if (debug) 
         printf("interpolate: a = %f, b = %f, c = %f, d = %f\n", a, b, c, d);

       for (k=i; k <= i+grid_spacing; k++) {
         for (l=j; l<= j+grid_spacing; l++) {

            p = (float)(k - i) / (float)grid_spacing;
            q = (float)(l - j) / (float)grid_spacing;

            PIX(f2,width,k,l) = a*p + b*q + c*p*q + d;
         }
       }
     }
   }

   /* transfer the processed image to the output DVimage */
 
   (*interpolated_image)->write(f2);



   /* free up all allocated space, i.e., release local image space */

   free(f1);
   free(f2);

   return(TRUE);

}


 
/****************************************************************
* 
*  Routine Name: mask_image - mask an image with a gradient-based mask pattern.
*                                       
*  Input: input_image   - pointer to DVimage 
*         mask_image    - pointer to a mask image: pixels with a gradient magnitude less than the threshold form the mask
*         threshold       threshold for the gradient magnitude  (percentage of maximum gradient magnitude)
*
*  Output output        - pointer to DVimage representing gradient magnitude
*                      
*
*  Written By: David Vernon
*  Date:       August 18, 2006
*  Modifications:
*
*
****************************************************************/
 

void mask_image (DVimage *input_image, DVimage *mask_image, DVimage *output_image, double threshold)
{
 
   int width, height, depth_in, depth_out; 
 
   DVimage *gradient_image;
   double temp, temp1, temp2, max;   
   int  i, j;
   int scale;

  
   // printf("mask_image: debug on \n");  
  
   if (input_image != NULL) {
	   
	   /* input data is intensity image */

      input_image->get_size(&width,&height);
      depth_in  = input_image->get_image_mode();
      depth_out = output_image->get_image_mode();

      gradient_image = new DVimage(width, height, GREYSCALE_IMAGE, NULL, NULL, DVFLOAT);


      // printf("width %d, height %d depth %d\n",width, height, depth);


      if (input_image->colour_mode == COLOUR_IMAGE && mask_image->colour_mode == COLOUR_IMAGE  && output_image->colour_mode == COLOUR_IMAGE)  {

         /* compute the gradient magnitude */

        // use red channel of mask for gradient

         max = 0;
         scale = 1;
 
         for (i= scale; i<width-scale; i++) {
            for (j= scale; j<height-scale; j++) {

              temp1 = (pixel(mask_image->idata,width,i-scale,j-1,0) +
                       pixel(mask_image->idata,width,i-scale,j  ,0) +
                       pixel(mask_image->idata,width,i-scale,j+1,0))    - 
                      (pixel(mask_image->idata,width,i+scale,j-1,0) +
                       pixel(mask_image->idata,width,i+scale,j  ,0) +
                       pixel(mask_image->idata,width,i+scale,j+1,0));
  
              temp2 = (pixel(mask_image->idata,width,i-1,j-scale,0) +
                       pixel(mask_image->idata,width,i  ,j-scale,0) +
                       pixel(mask_image->idata,width,i+1,j-scale,0))    -
                      (pixel(mask_image->idata,width,i-1,j+scale,0) +
                       pixel(mask_image->idata,width,i  ,j+scale,0) +
                       pixel(mask_image->idata,width,i+1,j+scale,0));
  

               temp = fabs(temp1) + fabs(temp2);
            
               PIX(gradient_image->fdata,width,i,j) = (float) (temp);

               if (temp > max) max = temp;
		      }
	      } 

         output_image->initialize();

         for (i= scale; i<width-scale; i++) {
            for (j= scale; j<height-scale; j++) {
 
               if (PIX(gradient_image->fdata, width, i, j) > (max * (threshold / 100))) {
                   pixel(output_image->idata, width, i, j,0) =  pixel(input_image->idata, width, i, j,0);
                   pixel(output_image->idata, width, i, j,1) =  pixel(input_image->idata, width, i, j,1);
                   pixel(output_image->idata, width, i, j,2) =  pixel(input_image->idata, width, i, j,2);
               }
               else {
                  pixel(output_image->idata, width, i, j,0) = 0;
                  pixel(output_image->idata, width, i, j,1) = 0;
                  pixel(output_image->idata, width, i, j,2) = 0;
               }
            }

         }
 
      }
      else  if (input_image->colour_mode == GREYSCALE_IMAGE && mask_image->colour_mode == GREYSCALE_IMAGE  && output_image->colour_mode == GREYSCALE_IMAGE)  {

         /* compute the gradient magnitude */

         max = 0;
         scale = 1;
 
         for (i= scale; i<width-scale; i++) {
            for (j= scale; j<height-scale; j++) {

              temp1 = (PIX(mask_image->idata,width,i-scale,j-1) +
                       PIX(mask_image->idata,width,i-scale,j  ) +
                       PIX(mask_image->idata,width,i-scale,j+1))    - 
                      (PIX(mask_image->idata,width,i+scale,j-1) +
                       PIX(mask_image->idata,width,i+scale,j  ) +
                       PIX(mask_image->idata,width,i+scale,j+1));
  
              temp2 = (PIX(mask_image->idata,width,i-1,j-scale) +
                       PIX(mask_image->idata,width,i  ,j-scale) +
                       PIX(mask_image->idata,width,i+1,j-scale))    -
                      (PIX(mask_image->idata,width,i-1,j+scale) +
                       PIX(mask_image->idata,width,i  ,j+scale) +
                       PIX(mask_image->idata,width,i+1,j+scale));
  

               temp = fabs(temp1) + fabs(temp2);
            
               PIX(gradient_image->fdata,width,i,j) = (float) (temp);

               if (temp > max) max = temp;
		      }
	      } 

         output_image->initialize();

         for (i= scale; i<width-scale; i++) {
            for (j= scale; j<height-scale; j++) {
 
               if (PIX(gradient_image->fdata, width, i, j) > (max * (threshold / 100))) {
                   PIX(output_image->idata, width, i, j) =  PIX(input_image->idata, width, i, j);
               }
               else {
                  PIX(output_image->idata, width, i, j) = 0;
               }
            }

         }
      }
      else {

         printf ("mask_image: image arguments must be either all colour or all greyscale \n");

      }

      delete gradient_image;
   }
 
   // printf("Leaving mask_image\n\n");

   return;
   
}


/****************************************************************
* 
*  Routine Name: enhance_local_maxima  
* 
*      Purpose: filter an image to accentuate local maxima and suppress other image data
*                        
*       Input: input_image    - pointer to DVimage 
*            half_kernel_size  int
*
*      Output: output_image   - pointer to DVimage representing cross power spectrum
*                          NB it is assumed that this image exists and is of the same dimensions
*                          as the inputs
*
*   Written By: David Vernon
*        Date: May 1, 2005
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications:
*
*
****************************************************************/
 

void enhance_local_maxima (DVimage *input_image, int half_kernel_size,
                     DVimage *output_image)
{
   int width, height; 
   float *input=NULL;
   float *output=NULL;   
   char debug, dump_debug_image;


   
#define PIX(f,width,i,j)   (*((f) + ( (j) * (width) )  + (i) ))

   /* set debug flags */

   debug = FALSE;
   dump_debug_image = FALSE; 


   if (debug) printf("enhance_local_maxima: debug on \n");  
   
 
   /* create local images for processing */

   if (input_image != NULL) {

	   /* input data is intensity image */
      input_image->get_size(&width,&height);

      input =  (float *) malloc(sizeof(float) * width * height);
      output =  (float *) malloc(sizeof(float) * width * height); 

      input_image->read(input);
 
	   //enhance_local_maxima_by_thinning(input, output, width, height);
 	   //enhance_local_maxima_by_suppression(input, output, width, height);
 	   enhance_local_maxima_by_filtering(input, half_kernel_size, output, width, height);
 	   //suppress_weak_maxima(output, half_kernel_size, output, width, height);
      
	   if (dump_debug_image) {
        //dump_float_image(output, width, height);
	   }
 

     /* transfer the processed image to the output images */

     if (output_image != NULL)
	    output_image->write(output); 


     /* free up all allocated space, i.e., release local image space */

     free(input);
     free(output);
   }
     
   if (debug) printf("Leaving enhance_local_maxima\n");

   return;
   
}


/****************************************************************
* 
*  Routine Name: enhance_local_maxima_by_filtering
* 
*      Purpose: 
*
*       Input: source_image -  
*            kernel_size  -  int: the radius of the filter kernel
*
*      Output: maxima_image  - image with local maxima enhanced
*
*      Returns: TRUE (1) on success, FALSE (0) otherwise
*
*  Restrictions: Restrictions on data or input as applicable
*   Written By: D. Vernon 
*        Date: May 2, 2005
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications: 
****************************************************************/

int enhance_local_maxima_by_filtering(float *source_image, int half_kernel_size, float *maxima_image, int width, int height)

{
 
   float *f1=NULL;
   float *f2=NULL;
 
   int i, j, p, q;
   int i_low_limit, i_high_limit, j_low_limit, j_high_limit;
   float sum, normalization;


	// allocate space

	f1  =  (float *) malloc(sizeof(float) * width * height);
	f2  =  (float *) malloc(sizeof(float) * width * height);

   /* initialize input */

	for (i=0; i<width; i++) {
	   for (j=0; j<height; j++) {
        PIX(f1,width,i,j) =  PIX(source_image, width, i, j);
		  PIX(f2,width,i,j) = 0;
	   }
	}

   i_low_limit = (half_kernel_size);
   i_high_limit = width- (half_kernel_size);
   j_low_limit = (half_kernel_size);
   j_high_limit = height-(half_kernel_size);

   normalization = (2 * (float) half_kernel_size + 1) * (2 * (float) half_kernel_size + 1);

   /* scan image */

   for (i=i_low_limit; i<i_high_limit; i++) {
      for (j=j_low_limit; j<j_high_limit; j++) {

 
         sum = 0;
 
         for (p=i-half_kernel_size; p<=i+half_kernel_size; p++) {
		    for (q=j-half_kernel_size; q<=j+half_kernel_size; q++) {
			   sum+= PIX(f1,width,p,q);
			}
		 }
		 PIX(f2,width,i,j) = PIX(f1,width,i,j) * sum / normalization;
      }
   }

   for (i=0; i<width; i++) 
      for (j=0; j<height; j++) 
         PIX(maxima_image,width,i,j) =  PIX(f2, width, i, j);


   /* free up all allocated space, i.e., release local image space */

   free(f1);
   free(f2);

   return(TRUE);

 }


/****************************************************************
* 
*  Routine Name: enhance_local_maxima_by_thinning
* 
*      Purpose: create an output image by performing a thinning followed by 
*            end-point shrinking; the values of all deleted points are  
*            transferred to the locally-maximal pixel
*
*       Input: source_image - image with distributed data
*
*      Output: merged_image  - image with single merged points representing the 
*                        cumulative value of a cluster
*
*      Returns: TRUE (1) on success, FALSE (0) otherwise
*
*  Restrictions: Restrictions on data or input as applicable
*   Written By: D. Vernon 
*        Date: Jun 20, 2002
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications: 
****************************************************************/

int enhance_local_maxima_by_thinning(float *source_image, float *merged_image, int width, int height)

{
 
   float *f1=NULL;
   float *f2=NULL;
 
 
   float  *a, *b, *temp;
   int border, passes, no_change, num_neighbors, critically_connected;
   int i, j, p, q, max_p, max_q;
   int i_low_limit, i_high_limit, j_low_limit, j_high_limit;
   float threshold = (float)0.1;
   float max;


	// allocate space

	f1  =  (float *) malloc(sizeof(float) * width * height);
	f2  =  (float *) malloc(sizeof(float) * width * height);

   /* initialize output */

   for (i=0; i<width; i++) 
     for (j=0; j<height; j++) 
        PIX(f1,width,i,j) =  PIX(source_image, width, i, j);

   i_low_limit = 1;
   i_high_limit = width-1;
   j_low_limit = 1;
   j_high_limit = height-1;

  

   /* thin from image to scratch and back etc. until */
   /* four successive passes without change.       */

   a = f1;
   b = f2;
   border = 2;   /* north */
   passes = 0;
 
   do {

      no_change = TRUE;

     /* scan image */

     for (i=i_low_limit; i<i_high_limit; i++) 
       for (j=j_low_limit; j<j_high_limit; j++) {

       /* copy pixel */

	   PIX(b,width,i,j) = PIX(a,width,i,j);

	   /* now see if it can be removed */

	   if (PIX(a,width,i,j) > threshold) {
	      if (border == 2 && PIX(a,width,i-1,j) < threshold  ||
		   border == 4 && PIX(a,width,i,j+1) < threshold  ||
		   border == 6 && PIX(a+1,width,i,j) < threshold  ||
		   border == 8 && PIX(a,width,i,j-1) < threshold) {

		  /* count number of neighbors */

		  num_neighbors = 0;
		  if (PIX(a,width,i-1,j-1) > threshold) num_neighbors++;
		  if (PIX(a,width,i-1,j) > threshold) num_neighbors++;
		  if (PIX(a,width,i-1,j+1) > threshold) num_neighbors++;
		  if (PIX(a,width,i,j+1) > threshold) num_neighbors++;
		  if (PIX(a,width,i+1,j+1) > threshold) num_neighbors++;
		  if (PIX(a,width,i+1,j) > threshold) num_neighbors++;
		  if (PIX(a,width,i+1,j-1) > threshold) num_neighbors++;
		  if (PIX(a,width,i,j-1) > threshold) num_neighbors++;

		  if (num_neighbors != 0) {

		    /* see if critically connected */

		    critically_connected = FALSE;

		    if (PIX(a,width,i,j+1) < threshold)
			if (PIX(a,width,i,j-1) < threshold)
			   if (PIX(a,width,i-1,j-1) > threshold || 
                        PIX(a,width,i-1,j) > threshold || 
                        PIX(a,width,i-1,j+1) > threshold)
			     if (PIX(a,width,i+1,j+1) > threshold || 
                          PIX(a,width,i+1,j) > threshold ||
                          PIX(a,width,i+1,j-1) > threshold)
				 critically_connected = TRUE;

		    if (critically_connected == FALSE)
			if (PIX(a,width,i-1,j) < threshold)
			   if (PIX(a,width,i+1,j) < threshold)
			     if (PIX(a,width,i-1,j-1) > threshold || PIX(a,width,i+1,j-1) > threshold 
                          || PIX(a,width,i,j-1) > threshold)
				 if (PIX(a,width,i-1,j+1) > threshold || PIX(a,width,i,j+1) > threshold
                            || PIX(a,width,i+1,j+1) > threshold)
				   critically_connected = TRUE;

		    if (critically_connected == FALSE)
			if (PIX(a,width,i-1,j) < threshold)
			   if (PIX(a,width,i,j-1) < threshold)
			     if (PIX(a,width,i-1,j-1) > threshold)
				 if (PIX(a,width,i-1,j+1) > threshold || 
                            PIX(a,width,i,j+1) > threshold || 
                            PIX(a,width,i+1,j+1) > threshold || 
                            PIX(a,width,i+1,j+1) > threshold || 
                            PIX(a,width,i+1,j-1) > threshold)
				   critically_connected = TRUE;

		    if (critically_connected == FALSE)
			if (PIX(a,width,i-1,j) < threshold)
			   if (PIX(a,width,i,j+1) < threshold)
			     if (PIX(a,width,i-1,j+1) > threshold)
				 if (PIX(a,width,i+1,j+1) > threshold || 
                            PIX(a,width,i+1,j) > threshold || 
                            PIX(a,width,i+1,j-1) > threshold || 
                            PIX(a,width,i,j-1) > threshold || 
                            PIX(a,width,i-1,j-1) > threshold)
				   critically_connected = TRUE;

		    if (critically_connected == FALSE)
			if (PIX(a,width,i,j+1) < threshold)
			   if (PIX(a,width,i+1,j) < threshold)
			     if (PIX(a,width,i+1,j+1) > threshold)
				 if (PIX(a,width,i+1,j-1) > threshold || 
                            PIX(a,width,i,j-1) > threshold || 
                            PIX(a,width,i-1,j-1) > threshold ||
                            PIX(a,width,i-1,j) > threshold || 
                            PIX(a,width,i-1,j+1) > threshold)
				   critically_connected = TRUE;

		    if (critically_connected == FALSE)
			if (PIX(a,width,i+1,j) < threshold)
			   if (PIX(a,width,i,j-1) < threshold)
			     if (PIX(a,width,i+1,j-1) > threshold)
				 if (PIX(a,width,i-1,j-1) > threshold || 
                            PIX(a,width,i-1,j) > threshold || 
                            PIX(a,width,i-1,j+1) > threshold || 
                            PIX(a,width,i,j+1) > threshold || 
                            PIX(a,width,i+1,j+1) > threshold)
				   critically_connected = TRUE;

		    if (critically_connected == FALSE) {

			/* remove pixel and transfer to  local maximum */

			PIX(b,width,i,j) = 0;
			no_change = FALSE;

			max = 0;
 
         for (p=i-1; p<=i+1; p++) {
				for (q=j-1; q<=j+1; q++) {
					if ((p!=i) && (q!=j)) {
						if (PIX(a,width,p,q) > max) {
					   	max = PIX(a,width,p,q);
					   	max_p = p;
						   max_q = q;
						}
					}
				}
			}
			PIX(b,width,max_p,max_q) += PIX(a,width,i,j);


		    }
		  }
	      }
	   }
	 }

     if (no_change==TRUE) 
	 ++passes;
     else
	 passes = 0;

     /* determine the next border orientation. */

     border = (border+4) - ((border+4) / 10)*10;
     if (border == 0) border = 4;


     /* for next passs, image and scratch swap */

     temp = a;
     a = b;
     b = temp;

   } while (passes < 4);

   /* copy final thinned image, remember that we */
   /* have just swapped labels                      */

   if (a != f1)
     for (i=i_low_limit; i<i_high_limit; i++) 
       for (j=j_low_limit; j<j_high_limit; j++) 
         PIX(f1,width,i,j) = PIX(a,width,i,j);

		   
   for (i=0; i<width; i++) 
     for (j=0; j<height; j++) 
        PIX(merged_image,width,i,j) =  PIX(f1, width, i, j);


   /* free up all allocated space, i.e., release local image space */

   free(f1);
   free(f2);

   return(TRUE);

 }

/****************************************************************
* 
*  Routine Name: enhance_local_maxima_by_suppression
* 
*      Purpose: 
*
*       Input: source_image -  
*
*      Output: maxima_image  - image with local maxima enhanced
*
*      Returns: TRUE (1) on success, FALSE (0) otherwise
*
*  Restrictions: Restrictions on data or input as applicable
*   Written By: D. Vernon 
*        Date: Jun 20, 2002
*     Verified: 
*  Side Effects: 
*     Examples: 
* Modifications: 
****************************************************************/

int enhance_local_maxima_by_suppression(float *source_image, float *maxima_image, int width, int height)

{
 
   float *f1=NULL;
   float *f2=NULL;
 
   int i, j, p, q;
   int i_low_limit, i_high_limit, j_low_limit, j_high_limit;
   float sum;
   int half_kernel_size = 4; //+++++++++++++


	// allocate space

	f1  =  (float *) malloc(sizeof(float) * width * height);
	f2  =  (float *) malloc(sizeof(float) * width * height);

   /* initialize input */

	for (i=0; i<width; i++) {
	   for (j=0; j<height; j++) {
        PIX(f1,width,i,j) =  PIX(source_image, width, i, j);
		  PIX(f2,width,i,j) = 0;
	   }
	}

   i_low_limit = (half_kernel_size);
   i_high_limit = width- (half_kernel_size);
   j_low_limit = (half_kernel_size);
   j_high_limit = height-(half_kernel_size);

  

   /* scan image */

   for (i=i_low_limit; i<i_high_limit; i++) {
     for (j=j_low_limit; j<j_high_limit; j++) {

 
       sum = 0;
 
       for (p=i-half_kernel_size; p<=i+half_kernel_size; p++) {
		   for (q=j-half_kernel_size; q<=j+half_kernel_size; q++) {
				if (p!=i && q!=j) {
					sum+= PIX(f1,width,p,q);
				}
			}
		 }
		 PIX(f2,width,i,j) = (PIX(f1,width,i,j)*(half_kernel_size+1)*(half_kernel_size+1))- sum ;
	  }
	}

 

   for (i=0; i<width; i++) 
     for (j=0; j<height; j++) 
        PIX(maxima_image,width,i,j) =  PIX(f2, width, i, j);


   /* free up all allocated space, i.e., release local image space */

   free(f1);
   free(f2);

   return(TRUE);

 }



/****************************************************************
* 
*  Routine Name: suppress_weak_maxima
* 
*  Input: source_image -  
*         kernel_size  -  int: the radius of the filter kernel
*
*  Output: maxima_image  - image with local maxima enhanced
*
*  Returns: TRUE (1) on success, FALSE (0) otherwise
*
*  Restrictions: Restrictions on data or input as applicable
*  Written By: D. Vernon 
*  Date: April 4, 2011
*  Modifications: 
****************************************************************/

int suppress_weak_maxima(float *source_image, int half_kernel_size, float *maxima_image, int width, int height)

{
   float *f1=NULL;
   float *f2=NULL;
 
   int i, j, p, q;
   int i_low_limit, i_high_limit, j_low_limit, j_high_limit;
   float sum, normalization, double_average;

	// allocate space

	f1  =  (float *) malloc(sizeof(float) * width * height);
	f2  =  (float *) malloc(sizeof(float) * width * height);

   /* initialize input */

	for (i=0; i<width; i++) {
	   for (j=0; j<height; j++) {
          PIX(f1,width,i,j) =  PIX(source_image, width, i, j);
		  PIX(f2,width,i,j) = 0;
	   }
	}

   i_low_limit = (half_kernel_size);
   i_high_limit = width- (half_kernel_size);
   j_low_limit = (half_kernel_size);
   j_high_limit = height-(half_kernel_size);

   normalization = (2 * (float) half_kernel_size + 1) * (2 * (float) half_kernel_size + 1);

   /* scan image */

   for (i=i_low_limit; i<i_high_limit; i++) {
      for (j=j_low_limit; j<j_high_limit; j++) {

         sum = 0;
 
         for (p=i-half_kernel_size; p<=i+half_kernel_size; p++) {
		    for (q=j-half_kernel_size; q<=j+half_kernel_size; q++) {
			   sum+= PIX(f1,width,p,q);
			}
		 }
         double_average = 2* sum / normalization;
         if (PIX(f1,width,i,j) > double_average) {
            PIX(f2,width,i,j) = PIX(f1,width,i,j);
         }
      }
   }

   for (i=0; i<width; i++) 
      for (j=0; j<height; j++) 
         PIX(maxima_image,width,i,j) =  PIX(f2, width, i, j);

   /* free up all allocated space, i.e., release local image space */

   free(f1);
   free(f2);

   return(TRUE);

 }
 
/****************************************************************
* 
*  Routine Name: gaussianApodization
* 
*  Purpose: apodize an image by pixel-by-pixel multiplication with a Gaussian image
*
*  Input:  input_image          - pointer to DVimage to be apodized
*          std_dev              - float giving the standard deviation of the Gaussian 
*
*  Output: output_image         - pointer to apodized DVimage 
*
*   Written By: David Vernon
*        Date:  July 20, 2007
*     Verified: 
* Side Effects: 
*     Examples: 
* Modifications:  extended to work with DVFLOAT images as well as DVINT image
*
*
****************************************************************/
 
void gaussianApodization (DVimage *input_image, float std_dev, DVimage *output_image)
{

   static int width=0, height=0, depth=0;   // static to allow us to avoid recomputing gaussian image
   static float sigma=0;                    // when multiple calls are made with the same arguments
   static DVimage *gaussian = NULL;         // 

   char debug;
   int w, h, d;
   int i, j;
   unsigned char  pixel_value;
   float float_pixel_value;
   float gaussian_value;
   float temp;
   int image_type;
    
    
   /* set debug flags */

   debug = FALSE;

   if (debug) printf("gaussianApodization: debug on \n");  
   if (debug) printf("standard deviation = %f\n",std_dev);  
   
  
   if (input_image == NULL  || output_image == NULL) return;  // images don't exist ... quit

   /* input data is intensity image */

   input_image->get_size(&w,&h);
   d          = input_image->get_image_mode();
   image_type = input_image->get_image_type();

   if (debug) printf("width, height, depth, type = %d %d %d %d\n", w, h, d, image_type);

   if (gaussian == NULL || width != w || height != h || sigma != std_dev) {

      if (debug) printf("generating gaussian image\n");

      width  = w;
      height = h;
      depth  = d;
      sigma = std_dev;

      if (gaussian != NULL)
         delete gaussian;

      gaussian = new DVimage(width,height,GREYSCALE_IMAGE,NULL,NULL,DVFLOAT);
 
      /* Generate a normalized 2D circular Gaussian image                       */
      /* with size nxm pixels, centred at pixel n/2, m/2, and centre value of 1 */
      /* sigma is the standard deviation of the Gaussian function               */

      for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {

            // gaussian_value = ( exp(-( (i-width/2) *(i-width/2)  )/(2*sigma*sigma)) / (sigma*sqrt(2*3.14159))  ) *  
            //                  ( exp(-( (j-height/2)*(j-height/2) )/(2*sigma*sigma)) / (sigma*sqrt(2*3.14159))  ); 

            gaussian_value = (float)(( exp(-( (i-width /2)*(i-width /2) )/(2*sigma*sigma))  ) *  
                                     ( exp(-( (j-height/2)*(j-height/2) )/(2*sigma*sigma))  ) );   // maximum value = 1

	         gaussian->put_pixel(i, j, gaussian_value); 

         }
      }
   }

   // now perform the apodization

   if (image_type == DVINT) {
      for (i=0; i< width; i++) {
         for (j=0; j< height; j++) {

            gaussian->get_pixel(i,j,&gaussian_value);
            input_image->get_pixel(i,j,&pixel_value, 0);
			   temp = (float) pixel_value * gaussian_value;
            output_image->put_pixel(i,j, (unsigned char) temp,0);
 
            if (depth > 1) {
               gaussian->get_pixel(i,j,&gaussian_value);
               input_image->get_pixel(i,j,&pixel_value, 1);
			      temp = (float) pixel_value * gaussian_value;
               output_image->put_pixel(i,j, (unsigned char) temp,1);


               gaussian->get_pixel(i,j,&gaussian_value);
               input_image->get_pixel(i,j,&pixel_value, 2);
			      temp = (float) pixel_value * gaussian_value;
               output_image->put_pixel(i,j, (unsigned char) temp,2);
            }
         }
      }
   }
   else {
      for (i=0; i< width; i++) {
         for (j=0; j< height; j++) {

            gaussian->get_pixel(i,j,&gaussian_value);
            input_image->get_pixel(i,j,&float_pixel_value, 0);
			   temp = float_pixel_value * gaussian_value;
            output_image->put_pixel(i,j, temp,0);
 
            if (depth > 1) {
               gaussian->get_pixel(i,j,&gaussian_value);
               input_image->get_pixel(i,j,&float_pixel_value, 1);
			      temp = float_pixel_value * gaussian_value;
               output_image->put_pixel(i,j, temp,1);


               gaussian->get_pixel(i,j,&gaussian_value);
               input_image->get_pixel(i,j,&float_pixel_value, 2);
			      temp = float_pixel_value * gaussian_value;
               output_image->put_pixel(i,j, temp,2);
            }
         }
      }
   }


 
   if (debug) printf("Leaving gaussianApodization\n");

   return;
   
}


/****************************************************************
* 
*  Routine Name: superimposeGaussian
* 
*  Purpose: Superimpose a Gaussian function on an image at given location
*           if the value of the image is non-zero, then the maximum of the image and Gaussian value is used.
*
*  Input:  image         - pointer to DVimage on which to superimpose the Gaussian; this must be GREYSCALE_IMAGE and DVFLOAT
*          std_dev       - double giving the standard deviation of the Gaussian 
*          support       - int giving the extent of support for the Gaussian 
*          time_constant - double giving the interval over which the Gaussian either decays to zero or grows to 1  
*                          The Gaussian value is scaled by a function of the difference between 
*                          the time the Gaussian was created and the current time, 
*                          with a scaling factor of 1 if the current time is the same as the creation time
*                          and a scaling factor of 0 if the current time is greater than the creation time plus the decay time.
*          x, y          - ints giving the point at which the Gaussian is to be centred
*          time          - double giving the time in seconds (possibly fractional) at which the Gaussian was created.
*          decay         - boolean flag: if true, then the Gaussian decays with time, otherwise it grows.
*
*  Output: image   - pointer to the resultant DVimage 
*
*  Written By:     David Vernon
*  Date:           June 18, 2011
*  Modifications:  August 21, 2012: growth and decay implemented
*
*
****************************************************************/
 
void superimposeGaussian (DVimage *image, double std_dev, int support, double time_constant, int x, int y, double time, bool decay)
{

   char debug;
   int w, h, d;
   int i, j;
   float pixel_value;
   double gaussian_value;
   double scale_factor;
   //portable_timeb_ tb;
   double now;

    
   /* set debug flags */

   debug = FALSE;
   //debug = TRUE;

   if (debug) printf("superimposeGaussian: sd=%4.2f, supp=%d, decay=%4.2f, x=%d, y=%d, t=%4.2f\n",std_dev, support, decay, x, y, time);  
   
  
   if (image == NULL) return;  // image doesn't exist ... quit

   /* input data is intensity image */

   image->get_size(&w,&h);
   d = image->get_image_mode();

   // if (debug) printf("superimposeGaussian: width, height, depth = %d %d %d \n", w, h, d);

   //portable_ftime(&tb); 
   // now =(double) tb.time + (double)tb.millitm / 1000; 

   now = yarp::os::Time::now();
 
   if (debug) printf("superimposeGaussian: now=%4.2f\n",now);  

   if (decay) {
      if ((now - time) > time_constant) {
         scale_factor = 0;
      }
      else {
         scale_factor = (1.0 - (now - time) / time_constant);
      }
   }
   else {
      if ((now - time) > time_constant) {
         scale_factor = 1;
      }
      else {
         scale_factor = ((now - time) / time_constant);
      }
   }

   if (debug) printf("superimposeGaussian: scale factor=%4.2f\n",scale_factor);  

   /* Generate a normalized 2D circular Gaussian image                       */
   /* with size nxm pixels, centred at pixel n/2, m/2, and centre value of 1 */
   /* sigma is the standard deviation of the Gaussian function               */

   for (i=0; i<support; i++) {
      for (j=0; j<support; j++) {

         if ((x+i-support/2 > 0) && (x+i-support/2 < w) && (y+j-support/2 > 0) && (y+j-support/2 < h)) {
            gaussian_value = (float)(( exp(-( (i-support/2)*(i-support/2) )/(2*std_dev*std_dev))  ) *  
                                     ( exp(-( (j-support/2)*(j-support/2) )/(2*std_dev*std_dev))  ) );   // maximum value = 1

            gaussian_value = gaussian_value * scale_factor;

            image->get_pixel(x+i-support/2,y+j-support/2,&pixel_value);
            if (gaussian_value > pixel_value) 
               image->put_pixel(x+i-support/2,y+j-support/2, (float) gaussian_value);

         }
      }
   }

   //if (debug) printf("superimposeGaussian: leaving\n");

   return;
   
}

/****************************************************************
* 
*  Routine Name: rectify
* 
*  Purpose: rectify an image by computing and applying a spatial transformation derived from 
*  the intrinsic and extrinsic parameters of the camera
*
*  The approach is based on two principal papers:
*
*  A. Dankers, N. Barnes, and A. Zelinsky, 2004.  Active Vision - Rectification and Depth Mapping, 
*  Proc. 2004 Australian Conference on Robotics and Automation. 
*
*  A. Fusiello, E. Trucco, and A. Verri, 2000.  A Compact Algorithms for rectification of stereo pairs, 
*  Machine Vision and Applications, Vol. 12, pp. 16-22.
*
*
*  Input:  input_image_left     - pointer to DVimages to be rectified
*          input_image_right
*
*                                 intrinsic camera parameters for the left camera:
*          fx_left              - float giving the x component of the focal length 
*          fy_left              - float giving the y component of the focal length 
*          px_left              - float giving the x coordinate of the principal point 
*          py_left              - float giving the y coordinate of the principal point 
*                                 extrinsic camera parameter for the left camera:
*          theta_y_left         - float: rotation of the camera about the Y axis relative to the gaze direction
*                                 i.e. relative to the version angle NOT relative to the absolute Y zero direction
*
*                                 intrinsic camera parameters for the right camera:
*          fx_right             - float giving the x component of the focal length 
*          fy_right             - float giving the y component of the focal length 
*          px_right             - float giving the x coordinate of the principal point 
*          py_right             - float giving the y coordinate of the principal point 
*                                 extrinsic camera parameter for the right camera:
*          theta_y_right        - float: rotation of the camera about the Y axis relative to the gaze direction
*                                 i.e. relative to the version angle NOT relative to the absolute Y zero direction
*
*  Output: output_image_left    - pointer to rectified DVimages
*          output_image_right
*
*  Written By: David Vernon
*  Date:       September 11, 2009
* 
****************************************************************/

void rectify(DVimage *input_image_left, DVimage *input_image_right, 
             float fx_left,  float fy_left,  float px_left,  float py_left,  float theta_y_left,
             float fx_right, float fy_right, float px_right, float py_right, float theta_y_right,
             DVimage *output_image_left,  DVimage *output_image_right)

{
   bool debug;
   int width,  height,  depth;
   int width1, height1, depth1;
   int i, j, k;
   int ii, jj;
   int i_prime, j_prime;
   float x_offset, y_offset;
   float x, y;
   unsigned char  pixel_value;
   unsigned char p1, p2, p3, p4;
   float x_frac, y_frac;


   static float theta_y_left_radians = 999;  // initial values are non-realizable angles to force the computation 
   static float theta_y_right_radians = 999; // of the transformation matrix upon the first call to this funtion.

   float tx_left = -10;   // arbitrary camera translation parameters 
   float ty_left = 0;
   float tz_left = 0;

   float tx_right = 10;
   float ty_right = 0;
   float tz_right = 0;

   float temp1, temp2;

   float **A_L;
   float **A_R;
   float **A_N;
   float **B;
   float **I;
   float **Rt_o;
   float **Rt_n;     
   float **P_o;
   float **P_n;
   float **Q_o;
   float **QI_o_L;
   float **QI_o_R;
   float **Q_n_L;
   float **Q_n_R;
   float *p_i;
   float *p_o;

   static float **T_L = NULL;
   static float **T_R = NULL;

 
   A_L     = matrix(1,3,1,3);
   A_R     = matrix(1,3,1,3);
   A_N     = matrix(1,3,1,3);
   Rt_o    = matrix(1,3,1,4);
   Rt_n    = matrix(1,3,1,4); 
   P_o     = matrix(1,3,1,4);
   P_n     = matrix(1,3,1,4);
   Q_o     = matrix(1,3,1,3);
   B       = matrix(1,3,1,1);
   QI_o_L  = matrix(1,3,1,3);
   QI_o_R  = matrix(1,3,1,3);
   Q_n_L   = matrix(1,3,1,3);
   Q_n_R   = matrix(1,3,1,3);
   I       = matrix(1,3,1,3);
   p_i     = vector(1,3);
   p_o     = vector(1,3);

   if (T_L == NULL) T_L     = matrix(1,3,1,3);
   if (T_R == NULL) T_R     = matrix(1,3,1,3);


   /* set debug flags */

   debug = false;

   if (debug) {
      printf("rectify: left intrinsic parameters  %4.1f, %4.1f, %4.1f, %4.1f\n",fx_left,  fy_left,  px_left,  py_left);  
      printf("rectify: right intrinsic parameters %4.1f, %4.1f, %4.1f, %4.1f\n",fx_right, fy_right, px_right, py_right);  
      printf("rectify: left and right angles      %4.1f, %4.1f\n",              theta_y_left, theta_y_right);  
   }
   
   if (input_image_left == NULL || input_image_right == NULL || output_image_left == NULL || output_image_right == NULL) {
      printf("rectify: one or more input and output images note provided; quitting.\n");
      return;  // images don't exist ... quit
   }

   /* input data is intensity image */

   input_image_left->get_size(&width,&height);
   depth = input_image_left->get_image_mode();

   input_image_right->get_size(&width1,&height1);
   depth1 = input_image_right->get_image_mode();

   if (width!=width1 || height!=height1 || depth!=depth1) {
      printf("rectify: images provided are a different size; quitting.\n");
      return;   // sizes aren't identical ... quit
   }                 

   /*
    * convert angles to radians and negate since the angle required is from current to world FoR, 
    * not vice versa as is specified by the vergence angle from which these angles are derived 
    */

   temp1 = -(theta_y_left  / (float) 180.0) * (float) 3.14159;
   temp2 = -(theta_y_right  / (float) 180.0) * (float) 3.14159;

   if ((fabs(theta_y_left_radians  - temp1) > 0.01) ||
       (fabs(theta_y_right_radians - temp2) > 0.01)   ) {

      /* new angles so we have to recompute the transformation */

      theta_y_left_radians  = -(theta_y_left  / (float) 180.0) * (float) 3.14159;
      theta_y_right_radians = -(theta_y_right / (float) 180.0) * (float) 3.14159;
 
      /*
       * LEFT CAMERA
       * old PPM
       */
   
      /* intrinsic camera parameter matrix A */

      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
           A_L[i][j] = 0;
         }
      }
      A_L[1][1] = fx_left;
      A_L[2][2] = fy_left;
      A_L[1][3] = px_left;
      A_L[2][3] = py_left;
      A_L[3][3] = 1;  

      /* old extrinsic camera parameter matrix [R t] */

      Rt_o[1][1] = cos(theta_y_left_radians);
      Rt_o[1][2] = 0;
      Rt_o[1][3] = sin(theta_y_left_radians);
      Rt_o[1][4] = tx_left;  
      Rt_o[2][1] = 0;
      Rt_o[2][2] = 1;
      Rt_o[2][3] = 0;
      Rt_o[2][4] = ty_left;
      Rt_o[3][1] = -sin(theta_y_left_radians);
      Rt_o[3][2] = 0;
      Rt_o[3][3] = cos(theta_y_left_radians);
      Rt_o[3][4] = tz_left;
 

      /* old perspective projection matrix (PPM) P_o = A x Rt_o */

      for (i=1;i<=3;i++) {
         for (j=1;j<=4;j++) {
            P_o[i][j]=0.0;
            for (k=1;k<=3;k++)
               P_o[i][j] += (A_L[i][k]*Rt_o[k][j]);
         }
      }

      if (debug) {
         printf("\nrectify: P_o left \n");
         print_matrix(P_o,1,3,1,4);
         printf(" \n");
      } 


      /* Q_o = left 3x3 of P_o */

      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
            Q_o[i][j] =P_o[i][j];
            QI_o_L[i][j]=Q_o[i][j]; // to be overwritten by gaussj() when determining the inverse
         }
      }

      /* QI_o ... inverse of Q_o */

      for (i=1;i<=3;i++) B[i][1] = 1;   // dummy RHS vector
     
      gaussj(QI_o_L,3,B,1);             // QI_o = Q_o on input, and inverse of Q_o on output


      /*
       * RIGHT CAMERA
       * old PPM
       */
   
      /* intrinsic camera parameter matrix A */

      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
            A_R[i][j] = 0;
         }
      }
      A_R[1][1] = fx_right;
      A_R[2][2] = fy_right;
      A_R[1][3] = px_right;
      A_R[2][3] = py_right;
      A_R[3][3] = 1;  

      /* old extrinsic camera parameter matrix [R t] */

      Rt_o[1][1] = cos(theta_y_right_radians);
      Rt_o[1][2] = 0;
      Rt_o[1][3] = sin(theta_y_right_radians);
      Rt_o[1][4] = tx_right;    
      Rt_o[2][1] = 0;
      Rt_o[2][2] = 1;
      Rt_o[2][3] = 0;
      Rt_o[2][4] = ty_right;
      Rt_o[3][1] = -sin(theta_y_right_radians);
      Rt_o[3][2] = 0;
      Rt_o[3][3] = cos(theta_y_right_radians);
      Rt_o[3][4] = tz_right;
    
      /* old perspective projection matrix (PPM) P_o = A x Rt_o */

      for (i=1;i<=3;i++) {
         for (j=1;j<=4;j++) {
            P_o[i][j]=0.0;
            for (k=1;k<=3;k++)
               P_o[i][j] += (A_R[i][k]*Rt_o[k][j]);
         }
      }
 
      if (debug) {
         printf("\nrectify: P_o right \n");
         print_matrix(P_o,1,3,1,4);
         printf(" \n");
      } 


      /* Q_o = left 3x3 of P_o */

      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
            Q_o[i][j] =P_o[i][j];
            QI_o_R[i][j]=Q_o[i][j]; // to be overwritten by gaussj() when determining the inverse
         }
      }

      
      /* QI_o ... inverse of Q_o */

      for (i=1;i<=3;i++) B[i][1] = 1; // dummy RHS vector
  
      gaussj(QI_o_R,3,B,1);             // QI_o = Q_o on input, and inverse of Q_o on output

 
      /*
       * LEFT CAMERA
       * new PPM
       */
   
      /* 
       * intrinsic camera parameter matrix A 
       * using the same (average) intrinsic parameter matrix satisfies the requirement  
       * that conjugate points must have the same vertical coordinate
       */
   
      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
            A_N[i][j] = (A_L[i][j] + A_R[i][j])/2;  
         }
      }

      Rt_n[1][1] = 1;
      Rt_n[1][2] = 0;
      Rt_n[1][3] = 0;
      Rt_n[1][4] = 0;
      Rt_n[2][1] = 0;
      Rt_n[2][2] = 1;
      Rt_n[2][3] = 0;
      Rt_n[2][4] = 0;
      Rt_n[3][1] = 0;
      Rt_n[3][2] = 0;
      Rt_n[3][3] = 1;
      Rt_n[3][4] = 1;

      if (debug) {
         printf("rectify: Rt_n left \n");
         print_matrix(Rt_n,1,3,1,4);
      }

      /* P_n = A x Rt_n */

      for (i=1;i<=3;i++) {
         for (j=1;j<=4;j++) {
            P_n[i][j]=0.0;
            for (k=1;k<=3;k++)
               P_n[i][j] += (A_N[i][k]*Rt_n[k][j]);
         }
      }

   
      /* Q_n = left 3x3 of P_n */

      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
            Q_n_L[i][j]=P_n[i][j];
         }
      }

      /* T = Q_n * QI_o */

      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
            T_L[i][j]=0.0;
            for (k=1;k<=3;k++)
               T_L[i][j] += (Q_n_L[i][k]*QI_o_L[k][j]);
         }
      }

      /* 
       * however, it is easier to build the output image by pixel filling; 
       * this means we really need a transformation mapping the rectified image back to the old image
       * we comput this by inverting the transformation
       */

      /* Invert T_L in place */

      for (i=1;i<=3;i++) B[i][1] = 1;   // dummy RHS vector
  
      gaussj(T_L,3,B,1);                // T on input, and inverse of T on output

      if (debug) {
         printf("rectify: transformation mapping left rectified image to old image\n");
         print_matrix(T_L,1,3,1,3);
      }


      /*
       * RIGHT CAMERA
       * new PPM
       */
   
      /* 
       * intrinsic camera parameter matrix A ... same as LEFT camera
       */
   

      /* new extrinsic camera parameter matrix [R t] */
 
      /* R is the same as the LEFT camera */

      /* P_n = A x Rt_n */

      for (i=1;i<=3;i++) {
         for (j=1;j<=4;j++) {
            P_n[i][j]=0.0;
            for (k=1;k<=3;k++)
               P_n[i][j] += (A_N[i][k]*Rt_n[k][j]);
         }
      }

   
      /* Q_n = left 3x3 of P_n */

      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
            Q_n_R[i][j]=P_n[i][j];
         }
      }

      /* T = Q_n * QI_o */

      for (i=1;i<=3;i++) {
         for (j=1;j<=3;j++) {
            T_R[i][j]=0.0;
            for (k=1;k<=3;k++)
               T_R[i][j] += (Q_n_R[i][k]*QI_o_R[k][j]);
         }
      }

      /* Invert T_R in place */

      for (i=1;i<=3;i++) B[i][1] = 1;   // dummy RHS vector
  
      gaussj(T_R,3,B,1);                // T on input, and inverse of T on output

      if (debug) {
         printf("rectify: transformation mapping right rectified image to old image\n");
         print_matrix(T_R,1,3,1,3);
      }
   }

   /* Apply the transformation to the LEFT image */

   /* first, compute the transformation of the principal point */

   p_i[1] = px_left;
   p_i[2] = py_left;
   p_i[3] = 1;

   if (debug) {
      printf("rectify: px_left, py_left \n");
      print_vector(p_i,1,3);
   }

   for (i=1;i<=3;i++) {
      p_o[i] = 0.0;
      for (j=1;j<=3;j++) {
         p_o[i] += (T_L[i][j]*p_i[j]);
      }
   }

   if (debug) {
      printf("rectify: px'_left, py'_right \n");
      print_vector(p_o,1,3);
   }

   /* 
    * Compute x and y offsets as the difference between the coordinate of the old principal point 
    * and the point to which it maps.
    * These offsets are then used to shift all transformed points  so that the position of the principal point is unchanged
    * in the original and rectified image
    */

   x_offset = p_i[1] - p_o[1];
   y_offset = p_i[2] - p_o[2];

   if (debug) printf("rectify: left x and y offsets are %f %f\n", x_offset, y_offset);

   /* initialize output, i.e. rectified, image */

   for (i=0; i<width; i++) {
      for (j=0; j<height; j++) {
         for (k=0; k<depth; k++) {
            output_image_left->put_pixel(i,j, (unsigned char) 0,k);
         }
      }
   }

   for (i=0; i<width; i++) {
      for (j=0; j<height; j++) {

         p_i[1] = (float) i;
         p_i[2] = (float) j;
         p_i[3] = 1;
   
         for (ii=1;ii<=3;ii++) {
            p_o[ii] = 0.0;
            for (jj=1;jj<=3;jj++) {
               p_o[ii] += (T_L[ii][jj]*p_i[jj]);
            }
         }

         x = (p_o[1] / p_o[3]) + x_offset;  // remember to normalize coordinates 
         y = (p_o[2] / p_o[3]) + y_offset;

         i_prime = (int) (x);   
         j_prime = (int) (y);   

         x_frac = x - (float) i_prime;
         y_frac = y - (float) j_prime;

         if (((i_prime > 0) && (i_prime < width-1)) && ((j_prime > 0) && (j_prime < height-1))) {
            for (k=0; k<depth; k++) {
             
               input_image_left->get_pixel(i_prime,j_prime,    &p1, k);
               input_image_left->get_pixel(i_prime,j_prime+1,  &p2, k);
               input_image_left->get_pixel(i_prime+1,j_prime,  &p3, k);
               input_image_left->get_pixel(i_prime+1,j_prime+1,&p4, k);
               x_frac = x - i_prime;
               y_frac = y - j_prime;

               pixel_value = (unsigned char) (                        // bilinear interpolation
                                              ((p3-p1)*x_frac) +
                                              ((p2-p1)*y_frac) + 
                                              ((p4+p1-p3-p2)*x_frac*y_frac) +
                                               (p1)
                                              );
           
               output_image_left->put_pixel(i,j, pixel_value,k);
            }
         }
      }
   }
 

    
   /* Apply the transformation to the RIGHT image */

   /* first, compute the transformation of the principal point */

   p_i[1] = px_right;
   p_i[2] = py_right;
   p_i[3] = 1;

   if (debug) {
      printf("rectify: px_right, py_right \n");
      print_vector(p_i,1,3);
   }

   for (i=1;i<=3;i++) {
      p_o[i] = 0.0;
      for (j=1;j<=3;j++) {
         p_o[i] += (T_R[i][j]*p_i[j]);
      }
   }

   if (debug) {
      printf("rectify: px'_right, py'_right \n");
      print_vector(p_o,1,3);
   }

   /* 
    * Compute x and y offsets as the difference between the coordinate of the old principal point 
    * and the point to which it maps.
    * These offsets are then used to shift all transformed points  so that the position of the principal point is unchanged
    * in the original and rectified image
    */

   x_offset = p_i[1] - p_o[1];
   y_offset = p_i[2] - p_o[2];

   if (debug) printf("rectify: right x and y offsets are %f %f\n", x_offset, y_offset);

   /* initialize output, i.e. rectified, image */

   for (i=0; i<width; i++) {
      for (j=0; j<height; j++) {
         for (k=0; k<depth; k++) {
            output_image_right->put_pixel(i,j, (unsigned char) 0,k);
         }
      }
   }

   for (i=0; i<width; i++) {
      for (j=0; j<height; j++) {

         p_i[1] = (float) i;
         p_i[2] = (float) j;
         p_i[3] = 1;
   
         for (ii=1;ii<=3;ii++) {
            p_o[ii] = 0.0;
            for (jj=1;jj<=3;jj++) {
               p_o[ii] += (T_R[ii][jj]*p_i[jj]);
            }
         }


         /* 
          * normalize coordinates 
          * and shift the right image to compensate for the difference in the coordinates of the principal points 
          */
         
         x = (p_o[1] / p_o[3]) + x_offset + (px_right - px_left);  
         y = (p_o[2] / p_o[3]) + y_offset + (py_right - py_left);  

         i_prime = (int) (x);   
         j_prime = (int) (y);   

         x_frac = x - (float) i_prime;
         y_frac = y - (float) j_prime;

         if (((i_prime > 0) && (i_prime < width-1)) && ((j_prime > 0) && (j_prime < height-1))) {
            for (k=0; k<depth; k++) {
             
               input_image_right->get_pixel(i_prime,j_prime,    &p1, k);
               input_image_right->get_pixel(i_prime,j_prime+1,  &p2, k);
               input_image_right->get_pixel(i_prime+1,j_prime,  &p3, k);
               input_image_right->get_pixel(i_prime+1,j_prime+1,&p4, k);
               x_frac = x - i_prime;
               y_frac = y - j_prime;

               pixel_value = (unsigned char) (                        // bilinear interpolation
                                              ((p3-p1)*x_frac) +
                                              ((p2-p1)*y_frac) + 
                                              ((p4+p1-p3-p2)*x_frac*y_frac) +
                                               (p1)
                                              );
           
               output_image_right->put_pixel(i,j, pixel_value,k);
            }
         }
      }
   }


   free_matrix(A_L,1,3,1,3);
   free_matrix(A_R,1,3,1,3);
   free_matrix(A_N,1,3,1,3);
   free_matrix(Rt_o,1,3,1,4);
   free_matrix(Rt_n,1,3,1,4); 
   free_matrix(P_o,1,3,1,4);
   free_matrix(P_n,1,3,1,4);
   free_matrix(Q_o,1,3,1,3);
   free_matrix(B,1,3,1,1);
   free_matrix(QI_o_L,1,3,1,3);
   free_matrix(QI_o_R,1,3,1,3);
   free_matrix(Q_n_L,1,3,1,3);
   free_matrix(Q_n_R,1,3,1,3);
   free_matrix(I,1,3,1,3);
   free_vector(p_i,1,3);
   free_vector(p_o,1,3);

   
   /* don't free the transformation matrices ... we may use them again next time */

   //free_matrix(T_L,1,3,1,3);
   //free_matrix(T_R,1,3,1,3);

   if (debug) printf("rectify: leaving ...\n");

   return;
   
}
 



 
/*****************************************************************************/
/*                                                                           */
/* ---  Subprogram Name:   find_blobs                                        */
/*                                                                           */
/*                                                                           */ 
/* ---  Functional Description:                                              */
/*                                                                           */
/* Analyse a binary image (zero: background; non-zero: object or foreground) */
/* Perform connectivity analysis on a binary image and label each distinct   */
/* 8-connected region                                                        */
/*                                                                           */
/* Compute:                                                                  */
/*                                                                           */
/*     number of blobs                                                       */
/*                                                                           */ 
/*     for each blob:                                                        */
/*       area                                                                */
/*       centroid coordinates                                                */
/*       perimeter                                                           */
/*       top-left and bottom-right of bounding rectangle                     */
/*                                                                           */
/* ---  Input Parameters:                                                    */
/*                                                                           */
/*     binary_image        unsigned char *                                   */
/*                     pointer to an array for RGB triples                   */
/*                     representing a segmented binary colour image          */
/*                     Only the red channel (i.e first byte of RGB           */
/*                     triple) is used.                                      */
/*                                                                           */
/*     image_size_x        int  horizontal size of image                     */
/*     image_size_y        int  vertical size of image                       */
/*                                                                           */
/*                                                                           */
/* ---  Output Parameters:                                                   */
/*                                                                           */
/*     blob_image         unsigned char *                                    */
/*                     pointer to an array for RGB triples                   */
/*                     representing a labelled blob colour image             */
/*                     The blobs are colour-coded                            */
/*                                                                           */
/*     blob_list          array of blob_type structures containing           */
/*                     blob statistics                                       */
/*                     see blob.h for definition                             */
/*                                                                           */
/* ---  Libraries and External Sub-Programs Referenced:                      */
/*                                                                           */
/*     blob.h                                                                */
/*                                                                           */
/* ---  Bugs:                                                                */
/*                                                                           */
/* ---  Author: David Vernon, 26/3/04                                        */
/*                                                                           */
/* ---  Revisions                                                            */
/*                                                                           */
/*     date:                        .                                        */
/*     revision:                                                             */
/*     by:                                                                   */
/*                                                                           */
/*****************************************************************************/

 

void find_blobs  (unsigned char *binary_image, 
				     int image_size_x, int image_size_y, 
				     unsigned char *labelled_image,
				     blob_type blob_list[],  int *no_of_blobs)

{

	int i, j, k;
	int n1, n2, n3, n4;
	int equivalence[MAX_NUMBER_OF_EQUIVALENCES+1][2];
	int look_up_table[MAX_NUMBER_OF_EQUIVALENCES][3];
	int label = 1;  // start region labelling at 1
	int ecount = 0; // number of equivalences
   int change;
   int temp; 
	int debug = FALSE; // set true if you want debug statements
	int *blob_image;
	int blob_count;
	int blob_labels[MAX_NUMBER_OF_EQUIVALENCES+1];
	int blob_colour[MAX_BLOBS];
	blob_type temp_blob;
	blob_image = new int[image_size_x*image_size_y*3];// NB creating a new array

   // first copy the binary image to the blob image
   // this is necessary as we use the image to store the label of each blob 
   // and since the number of blobs can exceen 256 we need to use an int image not 
   // an unsigned char image 

   for (i=0; i<image_size_x; i++) {
	   for (j=0; j<image_size_y; j++) {
	     pixel(blob_image,image_size_x,i,j,0) = pixel(binary_image,image_size_x,i,j,0);
	   }
   }
 
   // now zero the border pixels to ensure that all object pixels are embedded in the background

   for (i=0; i<image_size_x; i++) {
	  pixel(blob_image,image_size_x,i,0,0) = 0;
	  pixel(blob_image,image_size_x,i,image_size_y-1,0) = 0;
	  pixel(blob_image,image_size_x,i,0,1) = 0;
	  pixel(blob_image,image_size_x,i,image_size_y-1,1) = 0;
	  pixel(blob_image,image_size_x,i,0,2) = 0;
	  pixel(blob_image,image_size_x,i,image_size_y-1,2) = 0;
   }
    
   for (j=0; j<image_size_y; j++) {
	  pixel(blob_image,image_size_x,0,j,0) = 0;
	  pixel(blob_image,image_size_x,image_size_x-1,j,0) = 0;
	  pixel(blob_image,image_size_x,0,j,1) = 0;
	  pixel(blob_image,image_size_x,image_size_x-1,j,1) = 0;	  
	  pixel(blob_image,image_size_x,0,j,2) = 0;
	  pixel(blob_image,image_size_x,image_size_x-1,j,2) = 0;
   }

   
   // having done that, we now intialize the array of label equivalences
   // entries in this array of tuples (i.e. pairs of integers) indicate that regions labelled
   // with the two integers are actually the same regions.  
   // During the connectivity analysis, we label a region with possibly different labels
   // but then record the equivalence in the equivalence table.  Once the connectivity
   // analysis is complete, we then re-label the region with just a single label based
   // on the equivalence table.

   for (i=0; i<MAX_NUMBER_OF_EQUIVALENCES; i++) {

	   equivalence[i][0] = 0;
	   equivalence[i][1] = 0;

   }


   // Next we do the connectivity analysis and build and equivalence table

   for (i=1; i<image_size_x-1; i++) {          // note the fact that we are not
	  for (j=1; j<image_size_y-1; j++) {        // scanning the border pixels
		                                // because we assume each pixel has an 
		                                // 8-connected neighbour

	    if (pixel(blob_image,image_size_x,i,j,0) != 0) {
		   
	  	   // check the 8-connected neighbours n1, n2, n3, and n4 of the current pixel
		   //
		   // n1 n2 n3
		   //
		   // n4 x  n5
		   //
		   // n6 n7 n8 
		   //

		   if (pixel(blob_image,image_size_x,i-1,j-1,0) != 0) n1 = TRUE; else n1 = FALSE;
		   if (pixel(blob_image,image_size_x,i-1,j,0)   != 0) n2 = TRUE; else n2 = FALSE;
		   if (pixel(blob_image,image_size_x,i-1,j+1,0) != 0) n3 = TRUE; else n3 = FALSE;
		   if (pixel(blob_image,image_size_x,i,j-1,0)   != 0) n4 = TRUE; else n4 = FALSE;

			if (!n1 && !n2 && !n3 && !n4) {  // no connecting object so label it with a new label
			   pixel(blob_image,image_size_x,i,j,0) = label;

            // add an equivalence to itself to ensure we count single pixel blobs

			   add_equivalence(label,label, equivalence,&ecount,MAX_NUMBER_OF_EQUIVALENCES);

			   if (label < MAX_BLOBS) label++; else printf("Maximum number of blobs reached\n");
			}

		   else if (n1) {  // n1 connects so label pixel with n1 label
			   pixel(blob_image,image_size_x,i,j,0) = pixel(blob_image,image_size_x,i-1,j-1,0);
			   
			   // now add other connecting pixels to the equivalence table

			   if ((n2) && (pixel(blob_image,image_size_x,i-1,j,0) != pixel(blob_image,image_size_x,i,j,0))) {
				   add_equivalence(pixel(blob_image,image_size_x,i-1,j,0), // neighbour
				               pixel(blob_image,image_size_x,i,j,0),   // this
								   equivalence,&ecount,MAX_NUMBER_OF_EQUIVALENCES);
			   }
			   if ((n3)  && (pixel(blob_image,image_size_x,i-1,j+1,0) != pixel(blob_image,image_size_x,i,j,0))) {
				   add_equivalence(pixel(blob_image,image_size_x,i-1,j+1,0), // neighbour
				               pixel(blob_image,image_size_x,i,j,0),   // this
								   equivalence,&ecount,MAX_NUMBER_OF_EQUIVALENCES);
			   }
			   if ((n4)  && (pixel(blob_image,image_size_x,i,j-1,0) != pixel(blob_image,image_size_x,i,j,0))) {
				   add_equivalence(pixel(blob_image,image_size_x,i,j-1,0), // neighbour
				               pixel(blob_image,image_size_x,i,j,0),   // this
								   equivalence,&ecount,MAX_NUMBER_OF_EQUIVALENCES);
			   }
			}
		
		   else if (n2) {  // n2 connects so label pixel with n2 label
			   pixel(blob_image,image_size_x,i,j,0) = pixel(blob_image,image_size_x,i-1,j,0);
			   			   
			   // now add other connecting pixels to the equivalence table

			   if (n3 && (pixel(blob_image,image_size_x,i-1,j+1,0) != pixel(blob_image,image_size_x,i,j,0))) {
				   add_equivalence(pixel(blob_image,image_size_x,i-1,j+1,0), // neighbour
				               pixel(blob_image,image_size_x,i,j,0),   // this
								   equivalence,&ecount,MAX_NUMBER_OF_EQUIVALENCES);
			   }
			   if (n4 && (pixel(blob_image,image_size_x,i,j-1,0) != pixel(blob_image,image_size_x,i,j,0))) {
				   add_equivalence(pixel(blob_image,image_size_x,i,j-1,0), // neighbour
				               pixel(blob_image,image_size_x,i,j,0),   // this
								   equivalence,&ecount,MAX_NUMBER_OF_EQUIVALENCES);
			   }
			}
		   else if (n3) {  // n3 connects so label pixel with n3 label
			   pixel(blob_image,image_size_x,i,j,0) = pixel(blob_image,image_size_x,i-1,j+1,0);
			   
			   // now add other connecting pixels to the equivalence table

			   if (n4 && (pixel(blob_image,image_size_x,i,j-1,0) != pixel(blob_image,image_size_x,i,j,0))) {
				   add_equivalence(pixel(blob_image,image_size_x,i,j-1,0), // neighbour
				               pixel(blob_image,image_size_x,i,j,0),   // this
								   equivalence,&ecount,MAX_NUMBER_OF_EQUIVALENCES);
			   }
			}
		   else if (n4) {  // only n4 connects so label pixel with n4 label
			   pixel(blob_image,image_size_x,i,j,0) = pixel(blob_image,image_size_x,i,j-1,0);
			}
			
		 }
	  }
   }
 
   // now resolve the equivalences, 
   // e.g. if label a is equivalent to label b, and label c is equivalent to label a
   // change second equivalence to be label c is equivalent to label b 
   // and use label b as the final label for the region
  
   k=0;


   do {

      change = FALSE;
 
	   if (FALSE) {
        for (i=0; i<ecount; i++) {
	       printf("Equivalence: %d %d\n",equivalence[i][0],equivalence[i][1]);
		  }
	   }
 

      // sort on index 0

      for (i=0; i<ecount-1; i++) {
        for (j=i; j>=0; j--) {
			 if (equivalence[j][0] > equivalence[j+1][0]) {
				temp = equivalence[j][0];
				equivalence[j][0] = equivalence[j+1][0];
				equivalence[j+1][0] = temp;

				temp = equivalence[j][1];
				equivalence[j][1] = equivalence[j+1][1];
				equivalence[j+1][1] = temp;
			 }
		  }
	   }
 

     // resolve the transitive equivalences
 
       for (i=0; i<ecount; i++) {
         for (j=0; j<ecount; j++) {
			   if ((i!=j) && (equivalence[j][1] == equivalence[i][0])) {
				  equivalence[j][1] = equivalence[i][1];
			   }
			}
		 }

 

     // now remove one-to-many mappings  
    

     for (i=0; i<ecount; i++) {
       for (j=0; j<ecount; j++) {
		    if ((i!=j) && (equivalence[j][0] == equivalence[i][0]) && (equivalence[j][1] != equivalence[i][1]) ) { // swap
			   temp = equivalence[j][1];
			   equivalence[j][0] = equivalence[j][1];
			   equivalence[j][1] = equivalence[i][1];
			   change= TRUE;
			 }
		 }
	  }
   
 
     if (debug) {
       for (i=0; i<ecount; i++) {
	      printf("Resolved Equivalence %d %d pass 2\n",equivalence[i][0],equivalence[i][1]);
		 } 
	  }

   
   } while (change == TRUE);



 
   // now assign unique colour labels to each pixel in each blob using the resolved equivalence table


   // create a look up table by using the equivalence values (i.e. labels) as the index
   // and load some arbitrary colour
  
 
   for (k=0; k<ecount; k++) {
			     
	   // the constants are arbitrary numbers to ensure different colours for
      // blobs with similar label values
      // leave plane zero untouched as we will use this to compute the blob statistics.

	   // this part picks up the target labels

	   look_up_table[equivalence[k][1]][0] = equivalence[k][1]; 
 	   look_up_table[equivalence[k][1]][1] = (123*equivalence[k][1])%255; // 123: arbitary number to ensure colour image
	   look_up_table[equivalence[k][1]][2] = (39*equivalence[k][1])%255;  // 39:  same
 

	   // this part picks up the regions labels are equivalent to (or reference) the target labels

	   look_up_table[equivalence[k][0]][0] = equivalence[k][1]; 
 	   look_up_table[equivalence[k][0]][1] = (123*equivalence[k][1])%255;
	   look_up_table[equivalence[k][0]][2] = (39*equivalence[k][1])%255;
 
   }

   // background

   look_up_table[0][0] = 0; 
   look_up_table[0][1] = 0;
   look_up_table[0][2] = 0;


   for (i=0; i< image_size_x; i++) {  
		for (j=0; j< image_size_y; j++) { 
			
			pixel(labelled_image, image_size_x, i, j, 0) = (unsigned char) look_up_table[pixel(blob_image, image_size_x, i, j, 0)][0];
			pixel(labelled_image, image_size_x, i, j, 1) = (unsigned char) look_up_table[pixel(blob_image, image_size_x, i, j, 0)][1];
			pixel(labelled_image, image_size_x, i, j, 2) = (unsigned char) look_up_table[pixel(blob_image, image_size_x, i, j, 0)][2];

			// we also label the blob_image to allow us to computer statistics

		 	pixel(blob_image, image_size_x, i, j, 0) = look_up_table[pixel(blob_image, image_size_x, i, j, 0)][0];

		}
	}
 
 
   // count the number of blobs 
   // first sort on index 1

   for (i=0; i<ecount-1; i++) {
      for (j=i; j>=0; j--) {
	     if (equivalence[j][1] > equivalence[j+1][1]) {
			temp = equivalence[j][0];
			equivalence[j][0] = equivalence[j+1][0];
			equivalence[j+1][0] = temp;

			temp = equivalence[j][1];
			equivalence[j][1] = equivalence[j+1][1];
			equivalence[j+1][1] = temp;
		 }
	   }
   }


   // now create another look up table, 
   // this time indexed by blob label, and storing the blob number

   blob_count = 0;
   k = 0;
   while (k<ecount) {
 			    
	   blob_labels[equivalence[k][1]] = blob_count;

	   do {
		   k++;
		   blob_labels[equivalence[k][1]] = blob_count;
	   } while (k<ecount && equivalence[k][1]  ==  equivalence[k-1][1]);
 
	   blob_count++;

   }
 
   *no_of_blobs = blob_count;

   //printf("Blob count: %d\n",blob_count);

   // scan the image and compute the relevant blob statistics:
   //
   //   area
   //   centroid
   //   perimeter
   //   bounding box
 
   for (i=0; i<blob_count; i++) {
	  blob_list[i].area=0; 
	  blob_list[i].perimeter=0; 
	  blob_list[i].centroid_x=0;
	  blob_list[i].centroid_y=0;
	  blob_list[i].top_left_x=image_size_x;
	  blob_list[i].top_left_y=image_size_y;
	  blob_list[i].bottom_right_x=0;
	  blob_list[i].bottom_right_y=0;
   }

   for (i=0; i< image_size_x; i++) {  
		for (j=0; j< image_size_y; j++) { 
			if (pixel(blob_image, image_size_x, i, j, 0) != 0) {

				// area

			   blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].area+=1;

				// centroid

				blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].centroid_x+=i;
				blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].centroid_y+=j;

				// bounding rectangle coordinate

				if (i < blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].top_left_x)
					blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].top_left_x=i;

				if (j < blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].top_left_y)
					blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].top_left_y=j;

				if (i > blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].bottom_right_x)
					blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].bottom_right_x=i;

				if (j > blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].bottom_right_y)
					blob_list[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]].bottom_right_y=j;

				// need to keep a record of the blob colour to facilitate perimeter calculation

				blob_colour[blob_labels[pixel(blob_image, image_size_x, i, j, 0)]] = pixel(blob_image, image_size_x, i, j, 0);
 
			}
		}
	}

   for (i=0; i<blob_count; i++) {

	   // centroid 

	   blob_list[i].centroid_x=blob_list[i].centroid_x/blob_list[i].area ;
	   blob_list[i].centroid_y=blob_list[i].centroid_y/blob_list[i].area ;
 

	   // perimeter

      blob_list[i].perimeter = length_of_perimeter(blob_image, image_size_x, image_size_y,
						                     blob_list[i].top_left_x, blob_list[i].top_left_y,
						                     blob_colour[i]);
   }


   // sort blob list by area
      

   for (i=0; i<ecount-1; i++) {
     for (j=i; j>=0; j--) {
	    if (blob_list[j].area < blob_list[j+1].area) {
				temp_blob = blob_list[j];
				blob_list[j] = blob_list[j+1];
				blob_list[j+1] = temp_blob;

				temp = blob_colour[j];
				blob_colour[j] = blob_colour[j+1];
				blob_colour[j+1] = temp;
		 }
	  }
   }


   // relabel the biggest blob so that it is always the same colour

 
   for (i=0; i< image_size_x; i++) {  
		for (j=0; j< image_size_y; j++) { 
			
			if ( pixel(labelled_image, image_size_x, i, j, 0) == blob_colour[0])  {

			   pixel(labelled_image, image_size_x, i, j, 0) = (unsigned char) 255;
            pixel(labelled_image, image_size_x, i, j, 1) = (unsigned char) 0;
			   pixel(labelled_image, image_size_x, i, j, 2) = (unsigned char) 0;
         }

		}
	}
 

   delete(blob_image);
}



 
/*****************************************************************************/
/*                                                         */
/* ---  Subprogram Name:   add_equivalence                           */
/*                                                         */
/*                                                         */
/* ---  Functional Description:                                   */
/*                                                         */
/* Add a pair of pixel labels to an equivalence table                  */
/* The pair are inserted so that the first in the pair is the larger number  */
/* An equivalence pair is only added if it does not already exist         */
/*                                                         */
/*                                                         */
/* ---  Input Parameters:                                       */
/*                                                         */
/*     label1, label2      int  the two labels to be inserted          */
/*                                                         */
/*     equivalence         int[][2]  2-D array into which the labels     */
/*                     are to be inserted                      */
/*     image_size_y        int  vertical size of image               */
/*                                                         */
/*                                                         */
/* ---  Output Parameters:                                       */
/*                                                         */
/*     ecount            int *   the current number of equivalences    */
/*                                                         */
/*     max               int    the maximum number of equivalence     */
/*                           allowed                        */
/*                                                         */
/* ---  Libraries and External Sub-Programs Referenced:                 */
/*                                                         */
/*     blob.h                                                */
/*                                                         */
/* ---  Bugs:                                                */
/*                                                         */
/* ---  Author: David Vernon, 26/3/04                              */
/*                                                         */
/* ---  Revisions                                             */
/*                                                         */
/*     date:                        .                        */
/*     revision:                                             */
/*     by:                                                  */
/*                                                         */
/*****************************************************************************/

 
int add_equivalence(int label1, int label2, int equivalence[][2], int *ecount, int max) 
{
   int i;
	int finished;
	int return_value;
	int temp;
 
	// ensure smaller is second

	if (label1 < label2) {
		temp = label1;
		label1 = label2;
		label2 = temp;
	}

 
	i=0;
	finished = FALSE;
	while (!finished) {
		if (i<*ecount) {
			if ((equivalence[i][0] == label1) && (equivalence[i][1] == label2)){

				finished = TRUE;
			}
			else {
				i++;
			}
		}
		else {
			finished = TRUE;
		}
	}

	if (i == *ecount) { // equivalence doesn't exist so add it
	   equivalence[i][0] = label1;
	   equivalence[i][1] = label2;
	  
	   //printf("equivalence %d, %d\n",equivalence[i][0],equivalence[i][1]); 
   
	   // increment count of equivalences 
	 
	   if (*ecount < max) {
	   	  *ecount = *ecount+1; 
        return_value = TRUE;
	   }
 	   else {
	 	  printf("Maximum number of equivalences reached \n");
		  return_value = FALSE;
	   }
	
	}

   return(return_value);

}

 


/*****************************************************************************/
/*                                                         */
/* ---  Subprogram Name:   length_of_perimeter                        */
/*                                                         */
/*                                                         */
/* ---  Functional Description:                                   */
/*                                                         */
/* Computer the perimeter of a single blob                           */
/* Do this by finding a border pixel (i.e. one with the background as a     */
/* neighbour) and then following the border around the perimeter          */
/*                                                         */
/*                                                         */
/* ---  Input Parameters:                                       */
/*                                                         */
/*     blob_image         int *                                */
/*                     pointer to an array for RGB triples         */
/*                     representing a labelled blob colour image     */
/*                     The blobs are colour-coded                */
/*                                                         */
/*     image_size_x        int  horizontal size of image              */
/*     image_size_y        int  vertical size of image               */
/*                                                         */
/*     top_left_x         int  x coordinate of top-left corner of the   */
/*                         blob's bounding rectangle             */
/*     top_left_y         int  y coordinate of top-left corner of the   */
/*                         blob's bounding rectangle             */
/*                                                         */
/*     blob_label         int  the Red component value used to label    */
/*                         this blob                         */
/*                                                         */
/* ---  Output Parameters:                                       */
/*                                                         */
/*     function value      int length of perimeter                  */
/*                                                         */
/* ---  Libraries and External Sub-Programs Referenced:                 */
/*                                                         */
/*     blob.h                                                */
/*                                                         */
/* ---  Bugs:                                                */
/*                                                         */
/* ---  Author: David Vernon, 30/3/04                              */
/*                                                         */
/* ---  Revisions                                             */
/*                                                         */
/*     date:                        .                        */
/*     revision:                                             */
/*     by:                                                  */
/*                                                         */
/*****************************************************************************/

 

int length_of_perimeter  (int *blob_image, 
				        int image_size_x, int image_size_y,
						  int top_left_x, int top_left_y,
						  int blob_label)

{

	int current_x, current_y,
		previous_x, previous_y,
		first_x, first_y,
		next_x, next_y;
   float perimeter_value = 0.0;
   int isolated_pixel;
 

   // find initial point, i.e. top-most point of blob

   current_x = top_left_x;
   current_y = top_left_y;

   while ((current_x < image_size_x) && 
	     (pixel(blob_image, image_size_x, current_x, current_y, 0) != blob_label)) {
	   current_x++; // move to next pixel
   }

   if (pixel(blob_image, image_size_x, current_x, current_y, 0) != blob_label) {

	   // problem ... unable to find a pixel belonging to the blob

	   return (0);
   }


   // Now follow the boundary around in a clockwise direction until we arrive back
   // where we started.  
   // We do this by rotating from the previous pixel about the current pixel, until
   // we find another blob pixel.
   // To start things off, we assume the previous pixel is directly above the 
   // current one (this is safe to assume, since we know we are not connected to another
   // blob and therefore this pixel is a background pixel

   // However, we have to do one safety check before we get started:
   // we check to see if the pixel is an isolated single-pixel blob
   // this is a special case and the above algorithm won't work

   isolated_pixel = TRUE;

   if ( (pixel(blob_image, image_size_x, current_x-1, current_y-1, 0) == blob_label) ||
      (pixel(blob_image, image_size_x, current_x  , current_y-1, 0) == blob_label) ||
      (pixel(blob_image, image_size_x, current_x+1, current_y-1, 0) == blob_label) ||
      (pixel(blob_image, image_size_x, current_x+1, current_y  , 0) == blob_label) ||
      (pixel(blob_image, image_size_x, current_x+1, current_y+1, 0) == blob_label) ||
      (pixel(blob_image, image_size_x, current_x  , current_y+1, 0) == blob_label) ||
      (pixel(blob_image, image_size_x, current_x-1, current_y+1, 0) == blob_label) ||
      (pixel(blob_image, image_size_x, current_x-1, current_y  , 0) == blob_label) ) {

      // note: we formulate the test with || since as soon as a TRUE clause is evaluate
	   // the condition is TRUE and we don't evaluate any more clauses
	   // this is more efficient that using && and checking that it's not a blob pixel

	   isolated_pixel = FALSE;

   }

   if (isolated_pixel == TRUE) {

	   return(1);  // true value is 0.5^2 * 3.14159 = 0.785

   }
   else {

	   // Now do the boundary following

	   first_x = current_x;
	   first_y = current_y;

	   previous_x = current_x;
	   previous_y = current_y - 1;  // pixel above 
	   
	   do {

		   // find next ... rotate clockwise 45 degrees
		   next_x = previous_x;
		   next_y = previous_y;
		   
		   do {

		     if     (((current_x - next_x) ==  0) && ((current_y - next_y) ==  1))  next_x--;
		     else if (((current_x - next_x) ==  1) && ((current_y - next_y) ==  1))  next_y++; 
		     else if (((current_x - next_x) ==  1) && ((current_y - next_y) ==  0))  next_y++; 
		     else if (((current_x - next_x) ==  1) && ((current_y - next_y) == -1))  next_x++; 
		     else if (((current_x - next_x) ==  0) && ((current_y - next_y) == -1))  next_x++; 
		     else if (((current_x - next_x) == -1) && ((current_y - next_y) == -1))  next_y--; 
		     else if (((current_x - next_x) == -1) && ((current_y - next_y) ==  0))  next_y--; 
		     else if (((current_x - next_x) == -1) && ((current_y - next_y) ==  1))  next_x--; 


		   } while (pixel(blob_image, image_size_x, next_x, next_y, 0) != blob_label);


	     // update perimeter value depending on the direction of the next pixel
	     // if it is diagonal, add the square_root of two, otherwise add 1


	     if ( (abs(next_x - current_x) == 1) && (abs(next_y - current_y) == 1)) {
	   	    perimeter_value += (float) 1.4142;
		  }
		  else {
		    perimeter_value += 1;
		  }
	  
		  previous_x = current_x;
	     previous_y = current_y;

		  	  
		  current_x = next_x;
	     current_y = next_y;

		  // printf("previous %d %d; current %d %d; next %d %d\n",previous_x, previous_y, current_x, current_y, next_x, next_y);

	   
	   } while ( (current_x != first_x) || (current_y != first_y));


      return((int) perimeter_value);

   }
}





 
/****************************************************************************/
/***                                                                      ***/
/***                      Utility routines                                ***/
/***                                                                      ***/
/****************************************************************************/


__inline double _magnitude(double a_r, double a_i)
{
   return(sqrt( (a_r * a_r) + (a_i * a_i)  ));
}


__inline double log_magnitude(double a_r, double a_i)
{
   return(log10(sqrt( (a_r * a_r) + (a_i * a_i))+1));
}



__inline void add_complex(double op1_r, double op1_i, double op2_r, double op2_i, 
         double *result_r,  double *result_i)
{
   *result_r = op1_r + op2_r;
   *result_i = op1_i + op2_i;
}

__inline void subtract_complex(double op1_r, double op1_i, double op2_r, double op2_i, 
         double *result_r,  double *result_i)
{
   *result_r = op1_r - op2_r;
   *result_i = op1_i - op2_i;
}

__inline void multiply_complex(double op1_r, double op1_i, double op2_r, double op2_i, 
         double *result_r,  double *result_i)
{
   *result_r = (op1_r * op2_r) - (op1_i * op2_i);
   *result_i = (op1_r * op2_i) + (op1_i * op2_r);
}

__inline void divide_complex(double op1_r, double op1_i, double op2_r, double op2_i, 
         double *result_r,  double *result_i)
{
   *result_r = ((op1_r * op2_r) + (op1_i * op2_i)) / 
            ((op2_r * op2_r) + (op2_i * op2_i));
   *result_i = ((op1_i * op2_r) - (op1_r * op2_i)) / 
            ((op2_r * op2_r) + (op2_i * op2_i));
}



__inline void power_complex(double op1_r, double op1_i, double op2, 
         double *result_r,  double *result_i)
{

   /* raise complex number to the power of a real number */


   double r, theta;

   r = sqrt( (op1_r * op1_r) + (op1_i * op1_i)  );
   theta = atan2(op1_i, op1_r);

   *result_r = pow(r, op2) * cos(op2 * theta);
   *result_i = pow(r, op2) * sin(op2 * theta);
}


__inline void exp_complex(double op1_r, double op1_i, 
            double *result_r,  double *result_i)
{

   /* raise  e to the power of a complex number and return real and imaginary parts */

   double t1;

   t1 = exp(op1_r);
   *result_r = t1 * cos(op1_i);
   *result_i = t1 * sin(op1_i);
}

 

void bilinear_interpolation_complex(float x_offset, float y_offset,
                          double v00_r, double v00_i, 
                          double v01_r, double v01_i, 
                          double v10_r, double v10_i, 
                          double v11_r, double v11_i, 
                          double *result_r,  double *result_i)
{

   /* perform binlinear interpolation between four complex points   */
   /* the interpolated value is return as a complex point         */


   double a_r, a_i, b_r, b_i, c_r, c_i, d_r, d_i, p_r, p_i, q_r, q_i,
        t1_r, t1_i, t2_r, t2_i, t3_r, t3_i, t4_r, t4_i;


   char debug = TRUE;


   subtract_complex(v10_r, v10_i, v00_r, v00_i, &a_r, &a_i);
   subtract_complex(v01_r, v01_i, v00_r, v00_i, &b_r, &b_i);
   add_complex    (v11_r, v11_i, v00_r, v00_i, &t1_r, &t1_i);
   subtract_complex( t1_r,  t1_i, v10_r, v10_i, &t2_r, &t2_i);
   subtract_complex( t2_r,  t2_i, v01_r, v01_i, &c_r, &c_i);
   d_r = v00_r;
   d_i = v00_i;

   p_r = x_offset;
   p_i = 0;
   q_r = y_offset;
   q_i = 0;

   /* interpolated value = a*p + b*q + c*p*q + d */

   multiply_complex(a_r, a_i, p_r, p_i, &t1_r, &t1_i);
   multiply_complex(b_r, b_i, q_r, q_i, &t2_r, &t2_i);
   multiply_complex(c_r, c_i, p_r, p_i, &t3_r, &t3_i);
   multiply_complex(t3_r, t3_i, q_r, q_i, &t3_r, &t3_i);
   add_complex    (t1_r, t1_i, t2_r, t2_i, &t4_r, &t4_i);
   add_complex    (t4_r, t4_i, t3_r, t3_i, &t4_r, &t4_i);
   add_complex    (t4_r, t4_i,  d_r,  d_i, &t4_r, &t4_i);

   *result_r = t4_r;
   *result_i = t4_i;


   if (debug) 
     printf("binlinear_interpolation_complex: \n %f %f\n %f %f\n  %f %f\n %f %f\n ->  %f %f\n", v00_r, v00_i, v01_r, v01_i, v10_r, v10_i, v11_r, v11_i, *result_r, *result_i);

}
 

/*--------------------------------------------------------
The following routines are taken from Numerical Recipes by
Press, Teukolsky, Vetterling, and Flannery
----------------------------------------------------------*/



/* nrerror - numerical recipies standard error handler */

void nrerror(const char *error_text)
{
   if (FALSE) {
     printf("Numerical Recipies run-time error ... \n");
     printf("%s\n", error_text);
     /* printf("... now exiting to system ...\n"); */
   }
   return;
}


/* ivector - allocate a int vector with subscript range v[nl..nh] */

int *ivector(long nl, long nh) {

   int *v;

   v = (int *) malloc ((size_t) ((nh-nl+1+NR_END)*sizeof(int)));
   if (!v) nrerror("allocation failure in ivector()");
   return v-nl+NR_END;
}




/* free_ivector - free a float vector allocated with vector() */

void free_ivector(int *v, long nl, long nh) {

   free((FREE_ARG) (v+nl-NR_END));
}


/* vector - allocate a float vector with subscript range v[nl..nh] */

REAL *vector(long nl, long nh) {

   REAL *v;

   v = (REAL *) malloc ((size_t) ((nh-nl+1+NR_END)*sizeof(REAL)));
   if (!v) nrerror("allocation failure in vector()");
   return v-nl+NR_END;
}




/* free_vector - free a float vector allocated with vector() */

void free_vector(REAL *v, long nl, long nh) {

   free((FREE_ARG) (v+nl-NR_END));
}



/* matrix - allocate a float matrix with subscript range v[nrl..nrh][ncl..nch]*/

REAL **matrix(long nrl, long nrh, long ncl, long nch) {

   long i, nrow=nrh-nrl+1, ncol=nch-ncl+1;
   REAL **m;

   /* allocate pointers to rows */

   m = (REAL **) malloc ((size_t) ((nrow+NR_END)*sizeof(REAL*)));

   if (!m) nrerror("allocation failure 1 in matrix()");
   m += NR_END;
   m -= nrl;

   /* allocate rows and set pointers to them */

   m[nrl] = (REAL *) malloc((size_t)((nrow*ncol+NR_END)*sizeof(REAL)));
   if (!m[nrl]) nrerror("allocation failure 2 in matrix ()");
   m[nrl] += NR_END;
   m[nrl] -= ncl;

   for (i=nrl+1; i<=nrh; i++) 
     m[i] = m[i-1] + ncol;

   /* return pointer to array of pointers to rows */

   return m;
}


/* free_matrix - free a float matrix  allocated with matrix() */

void free_matrix(REAL **m, long nrl, long nrh, long ncl, long nch) {

   free((FREE_ARG) (m[nrl]+ncl-NR_END));
   free((FREE_ARG) (m+nrl-NR_END));

}



/* f3tensor - allocate a float 3tensor with subscript range 
   t[nrl..nrh][ncl..nch][ndl..ndh]                     */

REAL ***f3tensor(long nrl, long nrh, long ncl, long nch, long ndl, long ndh) {

   long i, j, nrow=nrh-nrl+1, ncol=nch-ncl+1, ndep=ndh-ndl+1;
   REAL ***t;

   /* allocate pointers to pointers to rows */

   t = (REAL ***) malloc ((size_t) ((nrow+NR_END)*sizeof(REAL**)));

   if (!t) nrerror("allocation failure 1 in f3tensor()");
   t += NR_END;
   t -= nrl;

   /* allocate pointers to rows and set pointers to them */

   t[nrl] = (REAL **) malloc((size_t)((nrow*ncol+NR_END)*sizeof(REAL*)));
   if (!t[nrl]) nrerror("allocation failure 2 in f3tensor ()");
   t[nrl] += NR_END;
   t[nrl] -= ncl;


   /* allocate rows and set pointers to them */

   t[nrl][ncl] = (REAL *)malloc((size_t)((nrow*ncol*ndep+NR_END)*sizeof(REAL)));
   if (!t[nrl][ncl]) nrerror("allocation failure 3 in f3tensor ()");
   t[nrl][ncl] += NR_END;
   t[nrl][ncl] -= ncl;

   for (j=ncl+1; j<=nch; j++) t[nrl][j]=t[nrl][j-1]+ndep;
   for (i=nrl+1; i<=nrh; i++) {
     t[i] = t[i-1] + ncol;
     t[i][ncl]=t[i-1][ncl]+ncol*ndep;
     for (j=ncl+1; j<=nch; j++) t[i][j]=t[i][j-1]+ndep;
   }

   /* return pointer to array of pointers to rows */

   return t;
}


/* free_f3tensor - free a float f3tensor  allocated with f3tensor() */

void free_f3tensor(REAL ***t, long nrl, long nrh, long ncl, long nch, long ndl, long ndh) {

   free((FREE_ARG) (t[nrl][ncl]+ndl-NR_END));
   free((FREE_ARG) (t[nrl]+ncl-NR_END));
   free((FREE_ARG) (t+nrl-NR_END));

}


/* print_matrix */

void print_matrix(REAL **m, long nrl, long nrh, long ncl, long nch) {

  int i,j;
   
  //printf(" ---------------\n");
  for (i=nrl; i<=nrh; i++) {
     for (j=ncl; j<= nch; j++) {
       printf("%f  ", m[i][j]);
     }
     printf("\n");
   }
  //printf(" ---------------\n");

}


/* print_vector*/

void print_vector(REAL *v, long nrl, long nrh) {

  int i;
   
  //printf(" ---------------\n");
  for (i=nrl; i<=nrh; i++) {
     printf("%f  ", v[i]);
  }
  printf("\n");
  //printf(" ---------------\n");

}

#define SWAP(a,b) tempr=(a);(a)=(b);(b)=tempr;

void fourn(REAL data[], unsigned  long nn[], int ndim, int isign)

/* replaces data with its ndim-dimensional discrete Fourier Transform, if isign is input as 1   */
/* nn[1..ndim] is an integer array containing the lengths of each dimension (number of complex   */
/* values), which  MUST be all powers of 2. data is a real array of length twice the product of  */
/* these lengths, in which the data are stored as in a multidimensional complex array: read and  */
/* imaginary parts of each element are in consecutive locations, and the rightmost index of the  */
/* array increases most rapidly as one proceeds along data. For a two-dimensional array, this is */
/* equivalent to storing the array by rows.  If isign is input as �1, data is replaced by its   */
/* inverse transform times the product of the lengths of all dimensions                    */

{
   int idim;
   unsigned long i1, i2, i3, i2rev, i3rev, ip1, ip2, ip3, ifp1, ifp2;
   unsigned long ibit, k1, k2, n, nprev, nrem, ntot;
   REAL tempi, tempr;
   double theta, wi, wpi, wpr, wr, wtemp;  /* double precision for trigonometric recurrences */
   
   for (ntot=1,idim=1; idim<=ndim; idim++) /* compute total number of complex values       */
     ntot *= nn[idim];

   nprev=1;

   for (idim=ndim; idim>=1; idim--) {     /* main loop over the dimensions */
     n=nn[idim];
     nrem=ntot/(n*nprev);
     ip1 = nprev << 1;
     ip2 = ip1*n;
     ip3 = ip2*nrem;
     i2rev=1;
   
     for (i2=1; i2<=ip2; i2+=ip1) {      /* this is the bit reversal section of the routine */
       if (i2 < i2rev) {
         for (i1=i2; i1<=i2+ip1-2; i1+=2) {
            for (i3=i1; i3<=ip3; i3+=ip2) {
              i3rev=i2rev+i3-i2;
              SWAP(data[i3], data[i3rev]);
              SWAP(data[i3+1], data[i3rev+1]);
            }
         }
       }
       ibit = ip2 >> 1;
       while (ibit >= ip1 && i2rev > ibit) {
         i2rev -= ibit;
         ibit >>= 1;
       }
       i2rev += ibit;
     }
     ifp1 = ip1;  /* here begins the Danielson-Lanczos section of the routine */
     while (ifp1 < ip2) {
       ifp2 = ifp1 << 1;
       theta = isign * 6.28318530717959/(ifp2/ip1);  /* initialize the trig. Recurrence */
       wtemp = sin(0.5 * theta);
       wpr = -2.0*wtemp*wtemp;
       wpi = sin(theta);
       wr = 1.0;
       wi = 0.0;
       for (i3=1; i3<=ifp1; i3+=ip1) {
         for (i1=i3; i1<=i3+ip1-2; i1+=2) {
            for (i2=i1; i2<=ip3; i2+=ifp2) {
              k1=i2; /* Danielson-Lanczos formula */
              k2 = k1+ifp1;
              tempr = (float)wr*data[k2]-(float)wi*data[k2+1];
              tempi = (float)wr*data[k2+1]+(float)wi*data[k2];
              data[k2] = data[k1]-tempr;
              data[k2+1] = data[k1+1]-tempi;
              data[k1] += tempr;
              data[k1+1] += tempi;
            }
         }
         wr=(wtemp=wr)*wpr-wi*wpi+wr; /* trigonometric recurrence */
         wi = wi*wpr + wtemp*wpi + wi;
       }
       ifp1 = ifp2;
     }
     nprev *= n;
   }
}

#undef SWAP

void rlft3(REAL ***data, REAL **speq, unsigned long nn1, unsigned long nn2, unsigned long nn3, int 
isign)

/* Given a three-dimensional real array data[1..nn1][1..nn2][1..nn3] (where nn1 = 1 for the case */
/* of a logically two-dimensional array), this routine returns (for isign=1) the complex fast   */
/* Fourier transform as two complex arrays: on output, data contains the zero and positive      */
/* frequency values of the third frequency component, while speq[1..nn1][1..2*nn2] contains the  */
/* Nyquist critical frequency values of the third frequency component.  First (and second)      */
/* frequency components are stored for zero, positive, and negative frequencies, in standard    */
/* wrap-around order. See text for description of how complex values are arranged. For isign=-1  */
/* the inverse transform (times nn1*nn2*nn3/2 as a constant multiplicative factor) is performed  */
/* with output data (viewed as a real array) deriving from the input data (viewed as complex)   */
/* and speq.  The dimensions nn1, nn2, nn3 must always be integer powers of 2               */

{
   unsigned long i1, i2, i3, j1, j2, j3, nn[4], ii3;
   double theta, wi, wpi, wpr, wr, wtemp;
   REAL c1, c2, h1r, h1i, h2r, h2i;

   //if (1+&data[nn1][nn2][nn3]-&data[1][1][1] != nn1*nn2*nn3)
   if (1+&data[nn1][nn2][nn3]-&data[1][1][1] != (REAL)(nn1*nn2*nn3))
     nrerror("rlft3: problems with dimensions or contiguity of data array\n");
   
   c1=0.5;
   c2 = (float)(-0.5)*isign;
   theta = isign*(6.28318530717959/nn3);
   wtemp=sin(0.5*theta);
   wpr= -2.0*wtemp*wtemp;
   wpi = sin(theta);
   nn[1] = nn1;
   nn[2] = nn2;
   nn[3] = nn3 >> 1;  
   if (isign == 1) {                 /* case of forward transform */
     fourn(&data[1][1][1]-1,nn,3,isign); /* here is where most of the compute time is spent */
     for (i1=1; i1<=nn1; i1++) 
       for (i2=1, j2=0; i2<=nn2; i2++) { /* extend data periodically into speq */
         speq[i1][++j2]=data[i1][i2][1];
         speq[i1][++j2]=data[i1][i2][2];
       }
   }
   for (i1=1; i1<=nn1;i1++) {
     j1 = (i1 != 1 ? nn1-i1+2 : 1);

     /* zero frequency is its own reflection, otherwise locate corresponding negative */
     /* frequency in wrap-around  order                                    */

     wr = 1.0;
     wi = 0.0;
     for (ii3=1, i3=1; i3<=(nn3>>2)+1;i3++,ii3+=2) {
       for (i2=1;i2<=nn2;i2++) {
         if (i3 ==1) {
            j2 = (i2 != 1 ? ((nn2-i2) << 1)+3 : 1);
            h1r = c1 * (data[i1][i2][1] +  speq[j1][j2]);
            h1i = c1 * (data[i1][i2][2] -  speq[j1][j2+1]);
            h2i = c2 * (data[i1][i2][1] -  speq[j1][j2]);
            h2r = -c2 * (data[i1][i2][2] +  speq[j1][j2+1]);
            data[i1][i2][1] = h1r +h2r;
            data[i1][i2][2] = h1i +h2i;
            speq[j1][j2] = h1r - h2r; 
            speq[j1][j2+1] = h2i - h1i; 
         }
         else {
            j2 = (i2 != 1 ? nn2-i2+2 : 1);
            j3 = nn3 + 3 - (i3 << 1);
            h1r = c1 * (data[i1][i2][ii3] +  data[j1][j2][j3]);
            h1i = c1 * (data[i1][i2][ii3+1] - data[j1][j2][j3+1]);
            h2i = c2 * (data[i1][i2][ii3] - data[j1][j2][j3]);
            h2r = -c2 * (data[i1][i2][ii3+1] + data[j1][j2][j3+1]);
            data[i1][i2][ii3] = (float)(h1r + wr*h2r - wi*h2i);
            data[i1][i2][ii3+1] = (float)(h1i + wr*h2i + wi*h2r);
            data[j1][j2][j3] = (float)(h1r - wr*h2r + wi*h2i);
            data[j1][j2][j3+1] = (float)(-h1i + wr*h2i + wi*h2r);
         }
       }
       wr = (wtemp=wr)*wpr-wi*wpi+wr;
       wi = wi*wpr+wtemp*wpi+wi;
     }
   }
   if (isign == -1)
     fourn(&data[1][1][1]-1,nn,3,isign);
}
  
#define SWAP(a,b) {temp=(a);(a)=(b);(b)=temp;}

 void gaussj(float **a, int n, float **b, int m)
{
	int *indxc,*indxr,*ipiv;
	int i,icol,irow,j,k,l,ll;
	float big,dum,pivinv,temp;

	indxc=ivector(1,n);
	indxr=ivector(1,n);
	ipiv=ivector(1,n);
	for (j=1;j<=n;j++) ipiv[j]=0;
	for (i=1;i<=n;i++) {
		big=0.0;
		for (j=1;j<=n;j++)
			if (ipiv[j] != 1)
				for (k=1;k<=n;k++) {
					if (ipiv[k] == 0) {
						if (fabs(a[j][k]) >= big) {
							big=fabs(a[j][k]);
							irow=j;
							icol=k;
						}
					}
				}
		++(ipiv[icol]);
		if (irow != icol) {
			for (l=1;l<=n;l++) SWAP(a[irow][l],a[icol][l])
			for (l=1;l<=m;l++) SWAP(b[irow][l],b[icol][l])
		}
		indxr[i]=irow;
		indxc[i]=icol;
		if (a[icol][icol] == 0.0) nrerror("gaussj: Singular Matrix");
		pivinv=(float)1.0/a[icol][icol];
		a[icol][icol]=1.0;
		for (l=1;l<=n;l++) a[icol][l] *= pivinv;
		for (l=1;l<=m;l++) b[icol][l] *= pivinv;
		for (ll=1;ll<=n;ll++)
			if (ll != icol) {
				dum=a[ll][icol];
				a[ll][icol]=0.0;
				for (l=1;l<=n;l++) a[ll][l] -= a[icol][l]*dum;
				for (l=1;l<=m;l++) b[ll][l] -= b[icol][l]*dum;
			}
	}
	for (l=n;l>=1;l--) {
		if (indxr[l] != indxc[l])
			for (k=1;k<=n;k++)
				SWAP(a[k][indxr[l]],a[k][indxc[l]]);
	}
	free_ivector(ipiv,1,n);
	free_ivector(indxr,1,n);
	free_ivector(indxc,1,n);
}

#undef SWAP

/* 

  fft

  This function implements a 2D FFT using the routines provided in 
  Numerical Recipes in C.

  Since the NR routines use a different array representation, the input/output
  images have to be mapped into temporary NR vector/matrix/tensor arrays

  Parameters passed:

  image ........... pointer to real image data (floating point array)
  real ............ pointer to real part of the FT (floating point array)
  imaginary ....... pointer to imaginary part of the FT (floating point array)
  width, height ... integers specifying the width and height of the images
               these must be a power of 2
  direction ....... integer specifying forward or reverse direction for FFT
               value 1  -> forward (FFT)
               value -1 -> reverse (inverse FFT)

*/



void fft(float *image, float *real, float *imaginary, int width, int height, 
      int direction) {

   REAL ***data, **speq;
   int i, j, w2, h2;
   double a, b;
   int debug=FALSE;

   float  *mag;
   w2 = width/2;
   h2 = height/2;

   /* allocate Numerical Recipes format arrays */

   data = f3tensor(1,1,1,width,1,height);
   speq = matrix(1,1,1,width * 2);

   if (direction ==1) {

     /* forward FFT */
   
     /* load image into data. */

     for (i=0; i<width; i++) {
       for (j=0; j<height; j++) {
         data[1][i+1][j+1] = PIX(image, width, i, j);
       }
     }

     rlft3(data,speq,1,width,height,1);  /* forward FFT */

     /* extract real and imaginary data */

     for (i=0; i<width; i++) {
       for (j=0; j<height; j++) {
          PIX(real,width,i,j) = 0;
          PIX(imaginary,width,i,j) =  0;
       }
     }


     for (i=1; i<=w2; i++) {
       for (j=1; j<=h2; j++) {
          PIX(real, width, w2+i-1, h2+j-1) = data[1][i][2*j-1];
          PIX(imaginary, width, w2+i-1, h2+j-1) = data[1][i][2*j];
       }
     }

     for (i=w2+1; i<=width; i++) {
       for (j=1; j<=h2; j++) {
          PIX(real, width, i-w2-1, h2+j-1) = data[1][i][2*j-1];
          PIX(imaginary, width, i-w2-1, h2+j-1) = data[1][i][2*j]; 
       }
     }

     /* get the rest by symmetry - remembering to use the complex conjugate */

     for (i=1; i<width; i++) {
       for (j=1; j<h2; j++) {
          PIX(real,width,i,j) = PIX(real,width,width-i,height-j);
          PIX(imaginary,width,i,j) =  -PIX(imaginary,width,width-i,height-j);
       }
     }


     /* now fill in the aliased values */

     for (i=1; i<=w2; i++) {
        PIX(real,width,w2-i+1,0) = speq[1][2*(i)-1];
        PIX(imaginary,width,w2-i+1,0) = -speq[1][2*(i)];
     }

     PIX(real,width,0,0) = speq[1][2*(w2+1)-1];
     PIX(imaginary,width,0,0) = speq[1][2*(w2+1)];

     for (i=w2+2; i<=width; i++) {
        PIX(real,width,width+w2+2-i-1,0) = speq[1][2*i-1];
        PIX(imaginary,width,width+w2+2-i-1,0) = -speq[1][2*i];
     }


     for (j=1; j<h2; j++) {
        PIX(real,width,0,j) = PIX(real,width,0,height-j);
        PIX(imaginary,width,0,j) =  -PIX(imaginary,width,0,height-j);
     }

     /* replace all phasors by their complex conjugates    */
     /* we do this because to make the NR FFT routine      */
     /* compatible with the IEEE FFR routine used by Khoros */

     for (i=0; i<width; i++) {
       for (j=0; j<height; j++) {
          PIX(imaginary,width,i,j) =   -PIX(imaginary,width,i,j);
       }
     }

     if (debug) {

       /* generate log magnitude image */

       mag =  (float *) malloc(sizeof(float) * width * height);

       for (i=0; i<width; i++) {
         for (j=0; j<height; j++) {
            a =  PIX(real,width,i,j);
            b =  PIX(imaginary,width,i,j);

            PIX(mag,width,i,j) = (float) log_magnitude(a,b);
         }
       }

       //dump_float_image(mag, width, height);
       free(mag);
     }

   }
   else {

     /* Inverse FFT */


     /* replace all phasors by their complex conjugates    */
     /* we do this because to make the NR FFT routine      */
     /* compatible with the IEEE FFR routine used by Khoros */

     for (i=0; i<width; i++) {
       for (j=0; j<height; j++) {
          PIX(imaginary,width,i,j) =   -PIX(imaginary,width,i,j);
       }
     }

     /* load real and imaginary values into data. */

     for (i=1; i<=width; i++) {
       for (j=1; j<=h2; j++) {
          data[1][i][2*j-1] = 0;
          data[1][i][2*j]   = 0;
       }
     }


     for (i=1; i<=w2; i++) {
       for (j=1; j<=h2; j++) {
          data[1][i][2*j-1] = PIX(real, width, w2+i-1, h2+j-1);
          data[1][i][2*j]   = PIX(imaginary, width, w2+i-1, h2+j-1);
       }
     }

     for (i=w2+1; i<=width; i++) {
       for (j=1; j<=h2; j++) {
          data[1][i][2*j-1] = PIX(real, width, i-w2-1, h2+j-1);
          data[1][i][2*j]   = PIX(imaginary, width, i-w2-1, h2+j-1); 
       }
     }

     /* get the rest by symmetry */

     for (i=1; i<width; i++) {
       for (j=1; j<h2; j++) {
          PIX(real,width,width-i,height-j) = PIX(real,width,i,j);
          PIX(imaginary,width,width-i,height-j) = -PIX(imaginary,width,i,j);
       }
     }


     /* now fill in the aliased values */

     for (i=1; i<=w2; i++) {
        speq[1][2*(i)-1] = PIX(real,width,w2-i+1,0);
        speq[1][2*(i)] = -PIX(imaginary,width,w2-i+1,0);
     }

     speq[1][2*(w2+1)-1] = PIX(real,width,0,0);
     speq[1][2*(w2+1)] = PIX(imaginary,width,0,0);

     for (i=w2+2; i<=width; i++) {
        speq[1][2*i-1] = PIX(real,width,width+w2+2-i-1,0);
        speq[1][2*i] = -PIX(imaginary,width,width+w2+2-i-1,0);
     }

     for (j=1; j<h2; j++) {
        PIX(real,width,0,height-j) = PIX(real,width,0,j);
        PIX(imaginary,width,0,height-j) = -PIX(imaginary,width,0,j);
     }

   
     rlft3(data,speq,1,width,height,-1); /* backward FFT */
   
     for (i=0; i<width; i++) {
       for (j=0; j<height; j++) {
         PIX(image, width, i, j) = data[1][i+1][j+1];
       }
     }
    

     if (debug) {
       //dump_float_image(image, width, height);
     }
   }


   free_matrix(speq,1,1,1,2*width);
   free_f3tensor(data,1,1,1,width,1,height);


}

void pause(int milliseconds) {

  portable_timeb_ tb;

   long int s1, s2;
   long int ms1, ms2;
   long elapsed;

   portable_ftime(&tb); 
   s1=(long) tb.time; 
   ms1=tb.millitm;

   do {
     portable_ftime(&tb); 
     s2=(long) tb.time; 
     ms2=tb.millitm; 
     elapsed =(s2*1000+ms2)-(s1*1000+ms1);
   } while (elapsed < milliseconds);
}

 
/* -library_code_end */
 
 
 
