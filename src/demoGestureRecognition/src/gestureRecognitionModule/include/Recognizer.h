#include <string>
#include <vector>
#include <cmath>
#include <yarp/os/Thread.h>
#include <yarp/math/math.h>
#include <iCub/ctrl/filters.h>
#include <iostream>
#include <fstream>
class Recognizer 
{
    std::vector<yarp::sig::Vector> scores;
    yarp::sig::Vector num;
    yarp::sig::Vector den;
    yarp::sig::Vector variances;
    int buffer;
    int time;
    int start;
    bool init;
    bool rec;
    iCub::ctrl::Filter* filter;

    int getIndex();

public:
    Recognizer();
    int recognize(const yarp::sig::Vector& u, int& start, int& end);
    void setFilter(const yarp::sig::Vector& num, const yarp::sig::Vector& den);
    void saveFilteredScores(const yarp::sig::Vector &scores, std::string &path);
    ~Recognizer();
};