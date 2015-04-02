

#ifndef __DUMMY_EXTRACTOR__
#define __DUMMY_EXTRACTOR__

#include "Extractor.h"
#include <math.h>


class DummyExtractor: public Extractor
{
public:
    DummyExtractor()
    {}

    DummyExtractor(ResourceFinder &rf)
        :Extractor(rf)
    {}

    virtual Extractor *create(ResourceFinder &rf) const {return new DummyExtractor(rf);}

    virtual void releaseExtractor()
    {
    }

    virtual bool init_impl()
    {
        return true;
    }

    virtual bool save_impl()
    {
        return true;
    }

    virtual bool load_impl()
    {
        return true;
    }

    virtual bool extract_impl()
    {
        fprintf(stdout,"Extracting!\n");
        fprintf(stdout,"Img timestamp: %f\n",this->ts_img);
        fprintf(stdout,"Lbl timestamp: %f\n",this->ts_labels);
        fprintf(stdout,"Delta times:   %f\n\n",fabs(this->ts_img-this->ts_labels));
        return true;
    }

    virtual bool update_ini_impl()
    {
        return true;
    }
};


#endif