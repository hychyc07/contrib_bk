#ifndef __ICUB_HANDPOSEUTIL_H__
#define __ICUB_HANDPOSEUTIL_H__

#include <yarp/sig/Vector.h>     

using namespace yarp::sig;

class HandPoseUtil
{
    public:
    static Vector addOffset(const Vector &handX, const Vector &handO, 
            double gazeOffsetX, double gazeOffsetY, double gazeOffsetZ, bool isLeftHand);
    static Vector rotateVector(const Vector &v, const Vector &axis, double angle_rad);
    static Vector comp2ndNormal(const Vector &x, const Vector &n, double angle_deg);
    static void getGazeAngles(const Vector &eyePos, const Vector &handPos, const Vector &handOrient, 
            double &e, double &r, bool lefthand = false);
};

#endif
