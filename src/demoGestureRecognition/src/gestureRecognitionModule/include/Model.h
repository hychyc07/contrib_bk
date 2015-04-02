#include <deque>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <yarp/sig/Vector.h>

class Model
{

public:

    virtual void computeOutput(std::deque<yarp::sig::Vector> &features, yarp::sig::Vector &output)=0;
    virtual void trainModel() {};
    virtual int getNumActions()=0;
    virtual bool read()=0;
    virtual void write()=0;
    virtual void printModel()=0;
    virtual ~Model() {};

};