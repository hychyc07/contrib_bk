
#ifndef HMAXEXTRACTOR_H
#define HMAXEXTRACTOR_H

#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <cv.h>
#include <highgui.h>
#include <fstream>
//#include "hmaxModel.h"
#include <iostream>

#include "Extractor.h"
//#include "../../hmin/include/hmaxModel.h"

class hmaxModel;
struct dictionary;

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class HmaxExtractor : public Extractor
{
private:
    hmaxModel *model;
    dictionary *dict;
    float* lastComputedVector;
    bool canSave;

public:
    HmaxExtractor()
    {}

    HmaxExtractor(ResourceFinder &rf);

    virtual Extractor *create(ResourceFinder &rf) const {return new HmaxExtractor(rf);}


    virtual void releaseExtractor();
    virtual bool extract_impl();
    virtual bool init_impl()
    {
        return true;
    }

    virtual bool save_impl();
    virtual bool load_impl();
    virtual bool update_ini_impl();
};

#endif // HMAXEXTRAC    TOR_H
