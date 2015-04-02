
#ifndef __BASIC_EXTRACTOR__
#define __BASIC_EXTRACTOR__

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/RpcServer.h>


#include <yarp/sig/Matrix.h>
#include <yarp/sig/Image.h>

#include <vector>
#include <deque>

#include <string>
#include <fstream>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

#define TABS "\t\t\t\t"

#define STATE_EXTRACT           0
#define STATE_DICTIONARIZE      1

#define MODE_STREAM             0
#define MODE_LOAD               1


class Extractor
{
protected:
    int                 state;
    int                 mode;

    string              context_path;
    string              save_path;
    string              name;
    string              type;
    string              time_tag;

    fstream             f_dat;
    fstream             f_ini;

    int                 n_samples;
    int                 max_samples;

    int                 n_currently_loaded;

    Semaphore           mutex;

    //data for extraction
    Image               img;
    Bottle              labels;

    double              ts_img;
    double              ts_labels;

    //data for dictionary
    Matrix              feature_vector;
    int                 feature_size;

    bool                updated_img;
    bool                updated_labels;
    bool                updated_feature_vector;
    bool                updated_data;


    bool                dictionarized;

    vector<string>      class_list;

    bool                update_class_list(Bottle &labels);

    virtual bool        init_impl()=0;
    virtual bool        save_impl()=0;
    virtual bool        load_impl()=0;
    virtual bool        extract_impl()=0;
    virtual bool        update_ini_impl()=0;


    virtual bool        is_ready()
    {
        bool ready=false;
        ready=ready||(updated_data);
        ready=ready||(updated_feature_vector);
        ready=ready||(updated_img && updated_labels && mode==MODE_STREAM);
        return ready;
    }

public:
    Extractor()
    {}

    Extractor(ResourceFinder &_rf);

    virtual bool init(const string &experiment_name);
    virtual bool release();

    ~Extractor()
    {
    }

    virtual bool get_n_samples(int *_n_samples)
    {
        *_n_samples=n_samples;
        return true;
    }

    virtual bool get_feature_size(int *_feature_size)
    {
        *_feature_size=feature_size;
        return true;
    }

    virtual bool get_class_list(vector<string> *_class_list)
    {
        *_class_list=class_list;
        return true;
    }

    virtual bool set_max_samples(const int &_max_samples)
    {
        max_samples=_max_samples;
        return true;
    }

    virtual Extractor *create(ResourceFinder &rf) const =0;

    virtual void        releaseExtractor(){}

    virtual bool        save();
    virtual bool        load();                     //actually: che struttura dovrebbe avere il load?
    virtual bool        extract();
    virtual bool        update_ini();

    //create a dictionary from the data read so far
    virtual bool        dictionarize(){return true;}

    virtual bool        is_dictionarized(){return dictionarized;}

    bool setImage(Image *_img, const double &ts)
    {
        img=*_img;
        ts_img=ts;
        updated_img=true;

        return true;
    }

    bool setLabels(Bottle *_labels, const double &ts)
    {
        labels=*_labels;
        ts_labels=ts;
        updated_labels=true;

        return true;
    }

    virtual bool feedData(const Matrix &_feature_vector)
    {
        return true;
    }

    bool setFeatureVector(const Matrix &_feature_vector)
    {
        feature_vector=_feature_vector;
        updated_feature_vector=true;
        return true;
    }

    bool getFeatureVector(Matrix *_feature_vector)
    {
        if(!updated_feature_vector)
            return false;

        *_feature_vector=feature_vector;
        return true;
    }

    bool getLabels(Bottle *_labels)
    {
        if(!updated_feature_vector)
            return false;

        *_labels=labels;
        return true;
    }

    virtual bool        release_data()
    {
        updated_img=false;
        updated_labels=false;
        updated_feature_vector=false;
        updated_data=false;

        return true;
    }

};




#endif
