

#ifndef __HDF_EXTRACTOR__
#define __HDF_EXTRACTOR__


#include "Extractor.h"


#include <cv.h>
#include <highgui.h>





class HighDimensionalityFeatureExtractor: public Extractor
{
private:
    cv::Ptr<cv::FeatureDetector>            detector;
    cv::Ptr<cv::DescriptorExtractor>        extractor;

    string                                  detector_type;
    string                                  extractor_type;

    vector<cv::KeyPoint>                    keypoints;
    cv::Mat                                 descriptors;

    int                                     feature_size;

public:
    HighDimensionalityFeatureExtractor()
    {}

    HighDimensionalityFeatureExtractor(ResourceFinder &rf);

    virtual Extractor *create(ResourceFinder &rf) const {return new HighDimensionalityFeatureExtractor(rf);}

    virtual void releaseExtractor()
    {}

    virtual bool init_impl()
    {
        return true;
    }

    virtual bool save_impl();
    virtual bool load_impl();
    virtual bool extract_impl();
    virtual bool update_ini_impl();
};


#endif