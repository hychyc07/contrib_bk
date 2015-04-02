
#include "HmaxExtractor.h"
#include "hmaxModel.h"

HmaxExtractor::HmaxExtractor(ResourceFinder &rf)
    :Extractor(rf)
{
    if(mode==MODE_STREAM)
    {
        dict = (dictionary *) malloc(sizeof(dictionary));
        dict->pfCount = rf.find("pfCount").asInt();
        dict->yxCountMax = rf.find("yxCountMax").asInt();
        dict->fCount = rf.find("fCount").asInt();
        dict->fSizes = (double*) malloc(dict->fCount * sizeof(double));
        dict->fVals  = (float*) malloc (dict->pfCount * dict->yxCountMax * dict->yxCountMax * dict->fCount * sizeof(float));

        //std::cout << "Opening " << rf.getContextPath() << "/" << rf.find("fSizes").asString() << std::endl;
        ifstream ifile((rf.getContextPath()+"/"+rf.find("fSizes").asString()).c_str());

        for(int s = 0; s < dict->fCount; s++) ifile >> dict->fSizes[s];
        ifile.close();

        //std::cout << "Opening " << rf.getContextPath() << "/" << rf.find("fSizes").asString()  << std::endl;
        ifile.open((rf.getContextPath()+"/"+rf.find("fVals").asString()).c_str());

        for(int s = 0; s < (dict->pfCount * dict->yxCountMax * dict->yxCountMax * dict->fCount); s++) ifile >> dict->fVals[s];
        ifile.close();

        model = new hmaxModel(rf.find("imgDepth").asInt(), dict);
        feature_size=model->FCount();
    }

    lastComputedVector = new float[feature_size];
    feature_vector.resize(1,feature_size);

    return;
}

bool HmaxExtractor::extract_impl()
{
    model->response((IplImage*)img.getIplImage(), lastComputedVector);

    for(int i=0; i<model->FCount(); i++)
        feature_vector[0][i]=static_cast<double>(lastComputedVector[i]);

    return true;
}

void HmaxExtractor::releaseExtractor()
{
    if(mode==MODE_STREAM)
    {
        delete model;
        delete dict->fSizes;
        delete dict->fVals;
        delete dict;
    }

    delete [] lastComputedVector;
}

bool HmaxExtractor::save_impl()
{
    if(!f_dat.is_open())
    {
        return false;
    }

    f_dat.write((char*)lastComputedVector, sizeof(float)*feature_size);
    return true;
}

bool HmaxExtractor::load_impl()
{
    if(!f_dat.is_open())
    {
        return false;
    }


    f_dat.read((char*)lastComputedVector, sizeof(float)*feature_size);
    for(int i=0; i<feature_size; i++)
        feature_vector[0][i]=lastComputedVector[i];

    return true;
}


bool HmaxExtractor::update_ini_impl()
{
    return true;
}


