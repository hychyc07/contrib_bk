#ifndef _GABORFILTER_H_
#define _GABORFILTER_H_
/***********************************************************************************************************************

Applies a set of gabor filters at each position in a single image.

***********************************************************************************************************************/
#include "main.h"
#include "layer.h"
#include "filter.h"


class GaborFilter : public Filter {
public:

    GaborFilter(int yxCount, float aspect, float lambda, float sigma, int fCount);
    //
    // Create a new filter of this type.
    //
    //    YXCOUNT - edge size of the filters in pixels.
    //
    //    ASPECT, LAMBDA, SIGMA - gabor filter parameters.
    //
    //    FCOUNT - number of different filters, i.e. the number of orientations.

    ~GaborFilter();

    /*----------------------------------------------------------------------------------------------------------------*/

    int FCount() const { return m_fCount; };

private:

    int    m_yxCount;
    int    m_fCount;
    float *m_gabors;

    float ComputeUnit(const Layer * const in[], float yc, float xc, int f) const;

};



#endif
