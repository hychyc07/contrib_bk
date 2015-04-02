// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Giorgio Metta and Francesco Orabona
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

/**
 * @file SalienceOperator.h
 * @brief definition of the salience computation class (for blobs).
 */

#ifndef _SALIENCEOPERATOR_H_
#define _SALIENCEOPERATOR_H_

#include <yarp/sig/Image.h>//#include <yarp/YARPImage.h>
#include <yarp/math/Math.h>//#include <yarp/YARPMath.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#include <iCub/ColorVQ.h>
#include <iCub/YARPBox.h>
#include <iCub/YARPIntegralImage.h>

#include <map>
#include <string>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;


#define Giotto1 0
#define Giotto2 1
#define CUST 20
#define FITIN   99
#define FITOUT 101
#define CODELENGHT 7


// Logpolar trans -- Notational conventions:
// x = ro*cos(eta/q)
// y = ro*sin(eta/q)
// ro = k1 + k2*lambda^(csi) csi > F
// r0 = csi					 csi <= F
// J(csi, eta) = lambda^csi * ln(lambda)/q (k1+k2*lambda^csi) outside the fovea
// J(csi, eta) = J(F, eta) within the fovea
// Jan 2004 -- by nat

// LATER: this code requires some serious clean up.
// LATER: check if any of this constants need to be modified to fit the new logpolar mapping.
//
namespace _logpolarParams {
    const int _xsize = 320;     // const int _xsize = 256;
    const int _ysize = 240;     // const int _ysize = 256;
    const int _srho = 152;
    const int _stheta = 240;    // const int _stheta = 252;
    const int _sfovea = 42;
    
    const int _xsizefovea = 128;
    const int _ysizefovea = 128;
    // this is the ratio between the full size cartesian image and the actual one
    const double _ratio = 0.25;		// 1/4
    
    // parameter of the transformation
    const double _q = _stheta/(2*3.1415926535897932384626433832795);
    const double _lambda = 1.02314422608633;
    const double _logLambda = log(_lambda);
    const double _k1 = (_sfovea - 0.5)+(_lambda)/(1-_lambda);
    const double _k2 = _lambda/(pow(_lambda,_sfovea)*(_lambda-1));
};

struct Image_Data {
    // Logarithm Index
    double Log_Index;
    bool Valid_Log_Index;

    // Zoom Level of the Remapped Image
    double Zoom_Level;

    // Ratio between the diameter of the image and the size of the smallest pixel
    int Resolution;
    double dres;

    //	int Fovea_Display_Mode; //0 Sawtooth (Raw); 1 Triangular; 2 Complete

    // Log Polar Metrics
    int Size_Rho;
    int Size_Theta;
    int Size_Fovea;
    int Size_LP;
    int Fovea_Type; //0->3 Giotto 2.0; 4->7 Giotto 2.1 //0;4 Sawtooth (Raw); 1;5 Triangular; 2;6 Complete
    int Pix_Numb;
    int Fovea_Display_Mode;

    // Remapped Cartesian Metrics
    int Size_X_Remap;
    int Size_Y_Remap;
    int Size_Img_Remap;

    // Original Cartesian Metrics
    int Size_X_Orig;
    int Size_Y_Orig;
    int Size_Img_Orig;

    // Color Depth of the Images
    int Orig_Planes;
    int Remap_Planes;
    int LP_Planes;

    // Orientation of the Cartesian Image
    bool Orig_LandScape;
    bool Remap_LandScape;

    int padding;

    float Ratio;  //Used just for naming purpose
};


/**
* Operator that manages the blobs and calculates the saliency of every blob
* based on the linear combination of top-down (search color) and bottom-up contribution (isolated blobs).
* @author Francesco Rea
*/

class SalienceOperator {
private:
    /**
    * pointer to the first blob
    */
    YARPBox *m_boxes;

    /**
    * size of the image
    */
    int imageSize;
    
    /**
    * pointer to the checkCutted
    */
    bool *_checkCutted;
    
    /**
    * pointer to index cutted
    */
    PixelInt *_indexCutted;

    /**
    * height of the images
    */
    int height;

    /**
    * width of the images
    */
    int width;

    /**
    * pointers to angleShift
    */
    double *_angShiftMap;

    /**
    * feature of the input image
    */
    Image_Data _img;
    
    /**
    * box which conteins the last max salience box
    */
    YARPBox box;
    
    /**
    * checks if the position passed as parameter is insied a precise blob
    * @param r row position in the logPolar space
    * @param c column position in the logPolar space
    * @param blobReference id number of the blob in the blobList
    * @return boolean value result of the check
    */
    bool checkInside(int r,int c, int blobReference);
    
public:
    /**
    * default constructor
    */
    SalienceOperator(){};

    /**
    * default destructor
    */
    ~SalienceOperator();

    /**
    * constructor
    * @param width1 dimension width
    * @param height1 dimension height
    */
    SalienceOperator(const int width1,const int height1);

    /**
    * resizes to the new value of width and height
    * @param width1 new width
    * @param height1 new height
    */
    void resize(const int width1, const int height1);

    /**
    * gets the tagged image of pixels,the R+G-, G*R-,B+Y-, the Red, blue and green Plans and creates a catalog of blobs
    * @param rg R+G- opponency colour image
    * @param rg R+G- opponency colour image
    * @param rg R+G- opponency colour image
    * @param r1 Red plan of the colour image
    * @param g1 Green plan of the opponency colour image
    * @param b1 Blue plan of the opponency colour image
    */
    void blobCatalog(ImageOf<PixelInt>& tagged,
                               ImageOf<PixelMono> &rg,
                               ImageOf<PixelMono> &gr,
                               ImageOf<PixelMono> &by,
                               ImageOf<PixelMono> &r1,
                               ImageOf<PixelMono> &g1,
                               ImageOf<PixelMono> &b1,
                               int last_tag);  //

    /**
    * conversion from log polar to cartesian of a point
    * @param irho rho ray of the polar point
    * @param itheta itheta angle of the polar point
    * @param ox x position of the point
    * @param oy y position of the point
    */
    void logPolar2Cartesian(int irho, int itheta, int& ox, int& oy); 

    /**
    * returns the XY center of a data image
    */
    int Get_XY_Center(double *xx, double *yy, int rho, int theta, Image_Data *par, double *Ang_Shift);

    /**
    * set parameters for Image_data
    * @param SXO size along x axis of the original image
    * @param SYO size of the original image along y axis
    * @param SXR size of the reconstruvtion image along x axis
    * @param SYR size of the resulting image along y axis
    * @param rho rho of logPolar conversion
    * @param theta theta of the logPolar conversion
    * @param fovea fovea size
    * @param resolution resolution of the image
    * @param LPMode logPolar modality (GIOTTO1, GIOTTO2, CUST)
    * @param ZoomLevel level of the zoom
    */
    Image_Data Set_Param(int SXO,
                     int SYO,
                     int SXR,
                     int SYR,
                     int rho,
                     int theta,
                     int fovea,
                     int resolution,
                     int LPMode, 
                     double ZoomLevel);

    /**
    * computes Index
    * @param Resolution resolution of the image
    * @param Fovea fovea size
    * @param SizeRho size of the rho value
    */
    double Compute_Index(double Resolution, int Fovea, int SizeRho);

    /**
    * draws visual quantized Colour Image
    * @param id input image (data)
    * @param tagged image composed by tags
    */
    void DrawVQColor(ImageOf<PixelBgr>& id, ImageOf<PixelInt>& tagged);

    /**
     * @brief function that returns the color of the fovea
     * @param redValue color of the blob (red channel)
     * @param greenValue color of the blob (green channel)
     * @param blueValue color of the blob (blue channel)
     */
    void getFoveaBlobColor(int &redValue, int &greenValue, int &blueValue);

    /**
    * gets x and y center
    * @param xx value of the x position that will be update
    * @param yy value of the y position that will be update
    * @param rho rho value fot the blob
    * @param theta theta position of the blob
    * @_img input image
    */
    void get_XY_Center(int* xx, int *yy,int irho,int itheta,ImageOf<PixelRgb> *_img);

    /**
    * returns in box the blob with the maximum saliency value
    * @param tagged the image of the tags
    * @param max_tag maximum value of the tag
    * @param box box of the fovea
    */
    void maxSalienceBlob(ImageOf<PixelInt>& tagged, int max_tag, YARPBox &box); //
     
    /**
    * draw the blobs present in the fovea area
    * @param output greyscale image representing the fovea
    * @param tagged the image of the tags
    */
    void drawFoveaBlob(ImageOf<PixelMono>& id, ImageOf<PixelInt>& tagged); //

    /**
    * draw the blobs present in the fovea area using extracted colors
    * @param output image in color representing the fovea
    * @param tagged the image of the tags
    */
    void drawColorFoveaBlob(ImageOf<PixelRgb>& id, ImageOf<PixelInt>& tagged); 

    /**
    * fuses all blobs in fovea proximity
    */
    void fuseFoveaBlob3(ImageOf<PixelInt>& tagged, char *blobList, PixelBgr var, int max_tag); //

    /**
    * function that calculates the centre of mass of the blob
    * @param in image composed by the tags
    * @param tag tag that the function would be looking for
    * @param x returned value for the x position of the blob
    * @param y returned value for the y position of the blob
    * @param mass mass of the blob
    */
    void centerOfMassAndMass(ImageOf<PixelInt> &in, PixelInt tag, int *x, int *y, double *mass); //

    /**
    * returns the mean colour of the selected blob
    */
    PixelBgr varBlob(ImageOf<PixelInt>& tagged, ImageOf<PixelMono> &rg, ImageOf<PixelMono> &gr, ImageOf<PixelMono> &by, int tag); //

    /**
    * computes the saliency value for all the blobs 
    * @param num_blob number of blobs
    * @last_tag last_tag
    */
    void ComputeSalienceAll(int num_blob, int last_tag); 

    /**
    * counts the number of the small blobs
    * @param tagged image composed of tags
    * @param blobList list of blobs
    * @param max_tag number of tagged blobs
    * @param min_size size of blob over the thresholding takes place
    */
    int countSmallBlobs(ImageOf<PixelInt>& tagged, char *blobList, int max_tag, int min_size);

    /**
    * merge blobs that are tagged with the same value
    * @param tagged the image of the tags
    * @param blobList list of the blobs
    * @param max_tag tag with the maximum value
    * @param numBlob number of blobs
    */
    void mergeBlobs(ImageOf<PixelInt>& tagged, char *blobList, int max_tag, int numBlob); //

    /**
    * calculates the MeanColours for every blob
    * @param last_tag number of blobs
    */
    void ComputeMeanColors(int last_tag); //

    /**
    * counts the number of spiky stimuli coming from every blob
    * @param id input image(data)
    * @param max_tag number of blobs
    * @param tagged image composed by tags
    */
    void countSpikes(ImageOf<PixelInt>& tagged, int max_tag, YARPBox &box);

    /**
    * draws the blobs in the scene painting the blobs with their mean colour
    * @param id input image(data)
    * @param tagged image composed by tags
    */
    void DrawMeanColorsLP(ImageOf<PixelBgr>& id, ImageOf<PixelInt>& tagged); //

    /**
    * draws the most salient blobs in the scene
    * @param id input image(data)
    * @param max_tag number of blobs
    * @param tagged image composed by tags
    */
    void DrawMaxSaliencyBlob(ImageOf<PixelMono>& id,int max_tag,ImageOf<PixelInt>& tagged);

    /**
    * draws the blobs which has strongest responses in the scene
    * @param id input image(data)
    * @param max_tag number of blobs
    * @param tagged image composed by tags
    */
    void DrawStrongestSaliencyBlob(ImageOf<PixelMono>& id,int max_tag,ImageOf<PixelInt>& tagged);

    /**
    * remove the blobs that are out of range
    * @param last_tag number of blobs
    * @param max_size maximum size of the blob
    * @param min_size minimum size of the blob
    */
    void RemoveNonValidNoRange(int last_tag, const int max_size, const int min_size);

    /**
    * draws the contrast LP and calculates saliency
    * @param rg mono image R+G-
    * @param gr mono image G+R-
    * @param by mono image B+Y-
    * @param dst resulting image of this drawing (contrast LP)
    * @param tagged image composed of tags
    * @param numBlob number of blobs
    * @param pBu bottom-up coefficient for tot saliency calculation
    * @param pTD top-down coefficient for tot saliency calculation
    * @param prg target red green colour
    * @param pgr target green red colour
    * @param pby target blue yellow colour
    */
    int DrawContrastLP(ImageOf<PixelMono>& rg, ImageOf<PixelMono>& gr,
        ImageOf<PixelMono>& by, ImageOf<PixelMono>& dst, ImageOf<PixelInt>& tagged,
        int numBlob, float pBU, float pTD,
        PixelMono prg, PixelMono pgr, PixelMono pby);

    /**
    * draws the contrast LP2 and calculates saliency
    * @param rg mono image R+G-
    * @param gr mono image G+R-
    * @param by mono image B+Y-
    * @param dst resulting image of this drawing (contrast LP)
    * @param tagged image composed of tags
    * @param numBlob number of blobs
    * @param pBu bottom-up coefficient for tot saliency calculation
    * @param pTD top-down coefficient for tot saliency calculation
    * @param prg target red green colour
    * @param pgr target green red colour
    * @param pby target blue yellow colour
    */
    int DrawContrastLP2(ImageOf<PixelMono>& rg, ImageOf<PixelMono>& gr,
                                  ImageOf<PixelMono>& by, ImageOf<PixelMono>& dst, ImageOf<PixelInt>& tagged,
                                  int numBlob, float pBU, float pTD,
                                  PixelMono prg, PixelMono pgr, PixelMono pby, PixelMono maxDest);

    /**
    * returns the blob with a specific number
    * @param num number of the selected blob
    */
    inline YARPBox & getBlobNum(int num){return m_boxes[num];}

    /**
    * removes the fovea blob
    * @param tagged image composed of tags
    */
    inline void removeFoveaBlob(ImageOf<PixelInt>& tagged) {m_boxes[tagged(0, 0)].valid=false;}

    /**
    * return the total area of the box passed as parameter
    * @param box reference to the box whose area will be returned
    */
    inline int TotalArea(YARPBox& box) { return (box.xmax - box.xmin + 1) * (box.ymax - box.ymin + 1); }

    /**
    * function that returns the list of blobs
    * @returns the list of extracted blobs
    */
    YARPBox* getBlobList();

    /** 
     * sets the percentage area.
     */
    inline bool setPercentageArea(double p) { pArea = p; return true; }

    /** 
     * get the percentage area
     */
    inline double getPercentageArea() const { return pArea; }

public:
    ColorVQ *colorVQ;               //color quantizator
    YARPIntegralImage *integralRG;  //integral image of R+G-
    YARPIntegralImage *integralGR;  //integral image of G+R-

    YARPIntegralImage *integralBY;  //integral image of B+Y-
    ImageOf<PixelBgr>* colorVQ_img; //colour quantization image
    ImageOf<PixelMono>* foveaBlob;  //one channel image representing the fovea blob
    ImageOf<PixelMono>* maxSalienceBlob_img; //image of the most salient blob

    double centroid_x;          //center of the max saliency blob, x coordinate
    double centroid_y;          //center of the max saliency blob, x coordinate
    double target_x;            //center of the blob that has strongest reinforcement, x coordinate in cartesian space
    double target_y;            //center of the blob that has strongest reinforcement, y coordinate in cartesian space
    int target_c;               //center of the blob that has strongest reinforcement, x coordinate in logPolar space
    int target_r;               //center of the blob that has strongest reinforcement, y coordinate in logPolar space
    double maxc;                //center of the max saliency blob, logpolar c coordinate
    double maxr;                //center of the max saliency blob, logpolar r coordinate
    std::map<std::string,int> spikeMap; //map which contains the number of spikes for a specific blob

private:
    double pArea;               //percentage of the blobdimension considered surrounding area   
};

#endif //_SALIENCEOPERATOR_H_


//----- end-of-file --- ( next line intentionally left blank ) ------------------
