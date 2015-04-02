#include <iCub/data3D/private/helpers.h>

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;

void Helpers::min(const Matrix &mat, yarp::sig::Vector &out)
{
    out.resize(mat.cols());
    double minx=10000;
    double miny=10000;
    double minz=10000;
    for (int i=0; i<mat.rows(); i++)
    {
        if (mat(i,0)<minx)
            minx=mat(i,0);
        if (mat(i,1)<miny)
            miny=mat(i,1);
        if (mat.cols()>2 && mat(i,2)<minz)
            minz=mat(i,2);
    }
    out[0]=minx;
    out[1]=miny;
    if (mat.cols()>2)
        out[2]=minz;
}

void Helpers::max(const Matrix &mat, yarp::sig::Vector &out)
{
    out.resize(mat.cols());
    double maxx=-10000;
    double maxy=-10000;
    double maxz=-10000;
    for (int i=0; i<mat.rows(); i++)
    {
        if (mat(i,0)>maxx)
            maxx=mat(i,0);
        if (mat(i,1)>maxy)
            maxy=mat(i,1);
        if (mat.cols()>2 && mat(i,2)>maxz)
            maxz=mat(i,2);
    }
    out[0]=maxx;
    out[1]=maxy;
    if (mat.cols()>2)
        out[2]=maxz;
}


double Helpers::mod(const double a, const double b)
{
    int result=floor(a/b);
    return (a-static_cast<float>(result)*b);
}

double Helpers::sign(const double value)
{
    double s=0.0;

    if (value>0)
        s=1.0;
    else if (value<0)
        s=-1.0;

    return s;
}

yarp::sig::Vector Helpers::extractSubVector(const yarp::sig::Vector &vect, const int i, const int j)
{
    yarp::sig::Vector out(2);
    out[0]=vect[i];
    out[1]=vect[j];
    return out;
}


