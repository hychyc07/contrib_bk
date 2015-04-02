#ifndef __HELPERS_H_
#define __HELPERS_H_

#include <string>
#include <cmath>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

class Helpers
{
public:
    static double sign(const double value);
    static void min(const yarp::sig::Matrix &mat, yarp::sig::Vector &out);
    static void max(const yarp::sig::Matrix &mat, yarp::sig::Vector &out);
    static double mod(const double a, const double b);
    static yarp::sig::Vector extractSubVector(const yarp::sig::Vector &vect, const int i, const int j);
};

#endif
