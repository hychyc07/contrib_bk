#ifndef _GMAXFILTER_H_
#define _GMAXFILTER_H_

#include "layer.h"
#include "filter.h"
/***********************************************************************************************************************

Performs a per-feature global maximum over position and scale in one or more input layers.

***********************************************************************************************************************/

class GMaxFilter : public Filter {
public:

    GMaxFilter(int sCount);
    //
    // Create a new filter of this type.
    //
    //    SCOUNT - number of scales over which to pool.

private:

    int m_sCount;

    float ComputeUnit(const Layer * const in[], float yc, float xc, int f) const;

};

/**********************************************************************************************************************/

#endif
