// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Bjoern Browatzki
 * email:   bjoern.Browatzki@tuebingen.mpg.de
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
 * @file ObjSegThread.h
 * @brief this file contains the definition of an additional thread that does the job for the main module.
 */

#ifndef _OBJ_SEG_THREAD_H_
#define _OBJ_SEG_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/math/Math.h>
#include <iCub/iKin/iKinFwd.h>

#include "opencv2/core/core.hpp"
#include "opencv2/legacy/legacy.hpp"

#include <iostream>
#include <string>

#define MODE_RGB    0
#define MODE_YUV    1
#define MODE_HSV    2
#define MODE_LAB    3

#define CAM_LEFT    0
#define CAM_RIGHT   1

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

class ObjSegThread : public yarp::os::RateThread {
public: 

    class Gmm
    {
        private:
        int kernelSize;
        cv::Mat means;
        std::vector<cv::Mat> inv_covs;
        cv::Mat weights;
        public:
        typedef float DTYPE;
        void fromCvEM(const CvEM &em, int ksize);
        void save(const std::string &filename) const;
        void load(const std::string &filename);
        inline bool empty() { return inv_covs.empty(); }
        inline int nClusters()const { return inv_covs.size(); }
        inline const cv::Mat &getMeans()const { return means; }
        inline const cv::Mat &getWeights()const { return weights; }
        inline const std::vector<cv::Mat> &getInvCovs()const { return inv_covs; }
        inline int getKernelSize() const{return kernelSize;}
    };

    //inline cv::Mat matmul(const cv::Mat& m1, const cv::Mat &m2, cv::Mat &dst) const
    inline void matmul(float *m1, float *m2, float *dst, int m1rows, int m1cols,int m2rows, int m2cols ) const
    {
        typedef Gmm::DTYPE T; 

        T sum;
        //cv::Mat dst(m1.rows, m2.cols, CV_64F);
        //T *pm1 =(T*)(m1.data);
        //T *pm2 =(T*)(m2.data);
        //T *pdst =(T*)(dst.data);

        //int m1cols = m1.cols;
        //int m2cols = m2.cols;
        //int dstcols = dst.cols;

        int i,j,k;
        for (i = 0; i < m1rows; i++)
        {
            for (j = 0; j < m2cols; j++)
            {
                sum = 0;
                for (k = 0; k < m1cols; k++)
                {
                    sum += m1[i*m1cols+k] * m2[k*m2cols+j];
                }
                dst[i*m2cols+j] = sum;
            }
        }
    }
   /**
    * contructor.
    * @param threshold threshold for image filtering.
    */
    ObjSegThread(int period, ResourceFinder &rf_, 
            BufferedPort<ImageOf<PixelBgr> > *camImgPort,
            BufferedPort<ImageOf<PixelBgr> > *camRightImgPort,
            BufferedPort<ImageOf<PixelBgr> > *objImgPort_,
            BufferedPort<ImageOf<PixelBgr> > *disparityImgPort_,
            PolyDriver &_robotArm,
            PolyDriver &_robotHead,
            PolyDriver &_robotTorso,
            PolyDriver &_armCartDriver,
            IGazeControl *gazeCtrl_,
            Semaphore &sendMutex_)
    : RateThread(period), rf(rf_), camLeftPort(camImgPort), camRightPort(camRightImgPort), objImgPort(objImgPort_), disparityImgPort(disparityImgPort_), 
      robotArm(_robotArm), robotHead(_robotHead), robotTorso(_robotTorso), armCartDriver(_armCartDriver), gazeCtrl(gazeCtrl_), sendMutex(sendMutex_)
        {
        }


   /**
    * destructor.
    */
   ~ObjSegThread() {}

   bool threadInit();     
   void threadRelease();
   void run(); 

private:

    double probCluster(const cv::Mat &sample_32F, const cv::Mat &mu, const cv::Mat &inv_sigma) const;
    void extractPatches(const cv::Mat &img, int patchSize, std::vector<cv::Mat> &patches) const;
    cv::Mat createProbMap(const Gmm &m, const cv::Mat &img) const;
    cv::Mat removeBorder(const cv::Mat &img, const cv::Vec3b &color);
    void trainGmm(const std::vector<cv::Mat> &samples, Gmm &currentGmm, int kernelSizeGmm, int numClusters) const;
    float predict(const Gmm &gmm, const cv::Mat &sample, cv::Mat &probs) const;

    BufferedPort<ImageOf<PixelBgr> > *camLeftPort; // input image
    BufferedPort<ImageOf<PixelBgr> > *camRightPort; // input image
    BufferedPort<ImageOf<PixelBgr> > *objImgPort; // output image
    BufferedPort<ImageOf<PixelBgr> > *disparityImgPort; // input image from stereo vision

    ImageOf<PixelBgr> *dImg; // disparity image

    ResourceFinder  &rf;


    Semaphore       sendMutex;

    int             camera;
    int             mod;

    double          scaleFactor;
    bool            display;

    double          min_bbox_x;
    double          min_bbox_y;
    double          min_bbox_z;
    double          max_bbox_x;
    double          max_bbox_y;
    double          max_bbox_z;

    double          min_bbox_offset_x;
    double          min_bbox_offset_y;
    double          min_bbox_offset_z;
    double          max_bbox_offset_x;
    double          max_bbox_offset_y;
    double          max_bbox_offset_z;

    int             numClusters_obj;
    int             numClusters_bkg;
    int             kernelSizeGmm_bkg;
    int             kernelSizeGmm_obj;

    double          w_bkg;
    double          w_obj;
    double          w_edge;

    int             minSegmentSize;

    Gmm             gmmBkg;
    Gmm             gmmObj;

    int             blurLevel;
    int             maxSamples;  // max. number of training samples for GMM

    int             highDisparity;
    int             lowDisparity;

    IEncoders *encHead;
    IEncoders *encTorso;
    IEncoders *encArm;
    ICartesianControl *armCart;
    IGazeControl *gazeCtrl;
    
    /* Devices passed from module */
    PolyDriver &robotArm;
    PolyDriver &robotHead;
    PolyDriver &robotTorso;
    PolyDriver &armCartDriver;

    cv::Rect getObjRect(const cv::Mat &src, double handDist) const;

    void getArmSkeleton(std::vector<Vector> &jointCoords);


    void createBoundingBox3D(const Vector &handX, const Vector &handO, 
        double bbox_x, double bbox_y, double bbox_z, std::vector<Vector> &bbox) const;
    void drawObjBoundingBox3D(cv::Mat &img, const std::vector<Vector> &bbox, const cv::Scalar &color = CV_RGB(255,0,0)) const;
    cv::Mat createBoundingBoxMask(const cv::Mat &img, const std::vector<Vector> &bbox3D) const;
    void calcConvexHull2D(const std::vector<Vector> &bbox3D, cv::vector<cv::Point> &hull) const;
    cv::Rect getBoundingRect(const cv::Mat &img, const std::vector<Vector> &bbox3D) const;
    void preprocessImage(cv::Mat &img) const;
    cv::Mat getGraphCutLabelling(const cv::Mat &prob, const cv::Mat &prob_obj) const;
    Vector getArmJointPosition(int jnt) const;
    void postprocessImage(cv::Mat &objMask) const;
    cv::Mat findBackgroundPixels(const cv::Mat &img, const cv::Mat &bboxMask, const cv::Mat &disparity) const;
    cv::Mat findObjectPixels(const cv::Mat &img, const cv::Mat &bboxMask, const cv::Mat &disparity) const;
    void sendSegmentedImage(const cv::Mat &img);


    inline Vector translatePoint3D(
            const Vector &x, 
            const Vector &o, 
            const double dx,
            const double dy,
            const double dz) const
    {
        Vector x_trans = x;
        Vector nx = iCub::ctrl::axis2dcm(o).getCol(0).subVector(0,2);
        Vector ny = iCub::ctrl::axis2dcm(o).getCol(1).subVector(0,2);
        Vector nz = iCub::ctrl::axis2dcm(o).getCol(2).subVector(0,2);
        x_trans += nx * dx;
        x_trans += ny * dy;
        x_trans += nz * dz;
        return x_trans;
    }
};

#endif

//----- end-of-file --- ( next line intentionally left blank ) ------------------

