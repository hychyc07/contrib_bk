/*
 * Copyright (C) 2013 Elena Ceseracciu, iCub Facility - Istituto Italiano di Tecnologia
 * CopyPolicy: Released under the terms of the GNU GPL v2.0 and later.
 *
 */

#include "TrainingDatabase.h"

size_t TrainingDatabase::countTotalNoSamples(const std::vector< std::string> &classNames)
{
    size_t totalNo=0;
    for (std::vector <std::string>::const_iterator classIt=classNames.cbegin(); classIt!= classNames.end(); ++classIt)
    {
        std::map<std::string, size_t>::const_iterator it;
        if ( (it= keys.find((*classIt)))== keys.end() )
            return false;
        if( data.size() <= it->second)
            return false;
        totalNo+=data.at(it->second).size();
    }
    return totalNo;
}

size_t TrainingDatabase::getDataForClass(size_t classId, gurls::gMat2D<double> &X, size_t startingRow)
{
    if (data.find(classId)!=data.end())
    {
        std::vector<yarp::sig::Vector> &classData=data.at(classId); 
        size_t nSamples=classData.size();
        for(size_t i=0; i<nSamples; ++i)
        {
            gurls::gVec<double> sample(classData.at(i).data() , classData.at(i).length());
            X.setRow(sample, startingRow+i);
        }
        return nSamples;
    }
    else return 0;
}

std::vector<std::string> TrainingDatabase::classList()
{
    std::vector<std::string> result;
    for (std::map<std::string, size_t>::const_iterator it=keys.cbegin(); it!=keys.cend(); ++it)
        result.push_back(it->first);
    return result;
}

bool TrainingDatabase::clear()
{  
    bool wasempty=keys.empty();
    keys.clear();
    data.clear();
    return wasempty;
}

bool TrainingDatabase::forget(const std::string className)
{
    std::map<std::string, size_t>::const_iterator it;
    if ( (it= keys.find(className))== keys.end() )
        return false;
    else
    {
        data.erase(it->second);
        keys.erase(it);
        return true;
    }
}

bool TrainingDatabase::getDataForClasses(const std::vector< std::string > classNames, gurls::gMat2D<double> &X, gurls::gMat2D<double> &y)
{
    size_t nClasses=classNames.size();
    size_t nSamples= countTotalNoSamples(classNames);
    X.resize(nSamples, nDimensions);
    y.resize(nSamples, nClasses);
    y=-1; 

    size_t startRow=0;
    size_t endRow=0;
    for (std::vector <std::string>::const_iterator classIt=classNames.cbegin(); classIt!= classNames.end(); ++classIt)
    {
        std::map<std::string, size_t>::const_iterator it;
        if ( (it= keys.find((*classIt)))== keys.end() )
            return false;
        size_t classId=it->second;
        if( data.size() < classId)
            return false;
        endRow+=getDataForClass(classId, X, startRow);
            
        for (size_t ct=startRow; ct<endRow; ++ct)
            y(ct,classId)=1;
        startRow=endRow;
    }
    return endRow>0;
}

bool TrainingDatabase::getDataForAllClasses(gurls::gMat2D<double> &X, gurls::gMat2D<double> &y)
{
    std::vector< std::string > classNames=classList();
    return getDataForClasses(classNames, X, y);
}

bool TrainingDatabase::addDataForClass(const std::string className, const yarp::sig::Vector &sample)
{
    if (className=="")
        return false;
    int id=getIdForClass(className);
    if(id<0)
    {
        id= (int) newClassId;
        keys.insert(std::pair<std::string, size_t>(className, newClassId));
        data.insert(std::pair<size_t,  std::vector<yarp::sig::Vector> >(newClassId, std::vector<yarp::sig::Vector>()));
        newClassId++;
    }
    return addDataForClass((size_t) id, sample);
}

bool TrainingDatabase::addDataForClass(size_t classId, const yarp::sig::Vector &sample)
{
    if (sample.length() != nDimensions)
    {
        if(nDimensions == 0)
            nDimensions = sample.length();
        else
            return false;
    }
    data[classId].push_back(sample);
    return true;
        
}
    
bool TrainingDatabase::setNDimensions(size_t nDims)
{
    nDimensions=nDims;
    return clear();
}

size_t TrainingDatabase::getNDimensions(){return nDimensions;}

int TrainingDatabase::getIdForClass(const std::string className)
{

    try
    {
        return keys.at(className);
    }
    catch(std::out_of_range){
        return -1;
    }

}

std::string TrainingDatabase::getNameForId(size_t classId)
{
    //ugh
    for (std::map<std::string, size_t>::const_iterator it=keys.begin(); it!=keys.end(); ++it)
        if (it->second ==classId)
            return it ->first;
    return "";
}
