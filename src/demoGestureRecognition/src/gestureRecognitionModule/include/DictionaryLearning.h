#include <deque>
#include <stdio.h>
#include <sstream>
#include <vector>
#include <iostream>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <yarp/sig/Vector.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Property.h>
#include <yarp/math/Math.h>

class DictionaryLearning
{
    std::string dictionariesFilePath;
    yarp::sig::Matrix dictionary;
    yarp::sig::Matrix A;

    std::string group;

    int dictionarySize;
    int featuresize;
    double lamda;

    void subMatrix(const yarp::sig::Matrix& A, const yarp::sig::Vector& indexes, yarp::sig::Matrix& Atmp);
    void max(const yarp::sig::Vector& x, double& maxVal, int& index);
    bool read();
    void printMatrixYarp(const yarp::sig::Matrix& A);

public:

    DictionaryLearning(std::string dictionariesFilePath, std::string group);
    void computeDictionary(const yarp::sig::Vector& feature, yarp::sig::Vector& descriptor);
    void learnDictionary();
    ~DictionaryLearning();
};