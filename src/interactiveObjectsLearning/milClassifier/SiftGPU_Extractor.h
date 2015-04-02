
#ifndef __SIFT_GPU_EXTRACTOR_H__
#define __SIFT_GPU_EXTRACTOR_H__

#include <vector>

#include <highgui.h>
#include <cv.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>


#include "GL/gl.h"

#if !defined(SIFTGPU_STATIC) && !defined(SIFTGPU_DLL_RUNTIME)
// SIFTGPU_STATIC comes from compiler
#define SIFTGPU_DLL_RUNTIME
// Load at runtime if the above macro defined
// comment the macro above to use static linking
#endif


#ifdef SIFTGPU_DLL_RUNTIME
   #include <dlfcn.h>
   #define FREE_MYLIB dlclose
   #define GET_MYPROC dlsym
#endif

#include "SiftGPU.h"


class SiftGPU_Extractor
{
private:
   ComboSiftGPU                        *combo;
   SiftMatchGPU                        *matcher;
   SiftGPU                             *sift;

public:
    SiftGPU_Extractor();
        
    int getFeatureNum();
    
    bool extractSift(IplImage *img);
    
    bool extractSift(IplImage *img, std::vector<SiftGPU::SiftKeypoint> *keypoints, std::vector<float> *descriptors, int feature_size=128);
    
    bool getFeatureVector(std::vector<SiftGPU::SiftKeypoint> *keypoints, std::vector<float> *descriptors, int feature_size=128);
};



#endif



