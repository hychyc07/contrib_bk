#ifndef _MAXFILTER_H
#define _MAXFILTER_H

/***********************************************************************************************************************

Performs a per-feature local maximum over position and scale in one or more input layers.

***********************************************************************************************************************/
#include "main.h"
#include "layer.h"
#include "filter.h"


class MaxFilter : public Filter {
public:

    MaxFilter(int sCount, int yxCount);
    //
    // Create a new filter of this type.
    //
    //    SCOUNT - number of scales over which to pool.
    //
    //    YXCOUNT - number of grid positions (in the largest scale) over which to pool.

private:

    int m_sCount;
    int m_yxCount;

    float ComputeUnit(const Layer * const in[], float yc, float xc, int f) const;

};



#endif
