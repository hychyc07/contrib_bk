

#ifndef __BOW_EXTRACTOR__
#define __BOW_EXTRACTOR__

#include "Extractor.h"

#include <cv.h>
#include <highgui.h>


class BagOfWordsExtractor: public Extractor
{
private:
    int                                 bow_feature_size;
    cv::Mat                             samples;
    cv::Mat                             centers;
    int                                 n_centers;
    int                                 max_bow_features;

    cv::Ptr<cv::DescriptorMatcher>      matcher;

    int                                 n_attempts;

    bool                                initialized;

    int                                 current_row;

    virtual bool init_impl()
    {
        return true;
    }

    virtual bool save_impl();
    virtual bool load_impl();
    virtual bool extract_impl();
    virtual bool update_ini_impl();

public:
    BagOfWordsExtractor()
    {}

    BagOfWordsExtractor(ResourceFinder &rf);

    virtual Extractor *create(ResourceFinder &rf) const {return new BagOfWordsExtractor(rf);}

    virtual void releaseExtractor()
    {}


    virtual bool dictionarize();

    virtual bool feedData(const Matrix &data);

};


#endif


