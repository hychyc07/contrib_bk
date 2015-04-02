/*
 * Copyright (C) 2013 Elena Ceseracciu, iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 and later.
 *
 */

#include <map>
#include <vector>
#include "gurls++/gmat2d.h"
#include "gurls++/gvec.h"
#include <yarp/sig/Vector.h>

class TrainingDatabase //might become a thread...
{
    std::map<std::string, size_t> keys;
    std::map<size_t,  std::vector<yarp::sig::Vector> > data;
    size_t nDimensions;
    size_t newClassId;

    size_t countTotalNoSamples(const std::vector< std::string> &classNames);

    size_t getDataForClass(size_t classId, gurls::gMat2D<double> &X, size_t startingRow);


public:
    TrainingDatabase()
    {
        nDimensions=0;
        newClassId=0;
    }

    TrainingDatabase(size_t nDim):nDimensions(nDim)
    {
        newClassId=0;
    }

    std::vector<std::string> classList();

    bool save(const std::string dbName){return false;};

    bool load(const std::string dbName){return false;};
    
    bool clear();
    
    bool forget(const std::string className);

    bool getDataForClasses(const std::vector< std::string > classNames, gurls::gMat2D<double> &X, gurls::gMat2D<double> &y);

    bool getDataForAllClasses(gurls::gMat2D<double> &X, gurls::gMat2D<double> &y);

    bool addDataForClass(const std::string className, const yarp::sig::Vector &sample);

    bool addDataForClass(size_t classId, const yarp::sig::Vector &sample);
    
    bool setNDimensions(size_t nDims);

    size_t getNDimensions();

    int getIdForClass(const std::string className);

    std::string getNameForId(size_t classId);
};