#ifndef _NDPFITLER_H_
#define _NDPFILTER_H_
#include "filter.h"
#include "dictionary.h"
/***********************************************************************************************************************

Applies a set of learned feature templates at each position in a single layer.  The response to a given template at a
given position is computed using a normalized dot product.

***********************************************************************************************************************/

class NDPFilter : public Filter {
public:

    NDPFilter(bool zero, float thres, bool abs, const dictionary *dict);
    //
    // Create a new filter of this type.
    //
    //    ZERO - Should the input patch's mean be subtracted before computing its norm?
    //
    //    THRES - Minimum value of the norm to divide by.  The result of the dot product will be divided by
    //    max(norm, THRES * sqrt(N)), where N is the number of inputs.
    //
    //    ABS - Do we take the absolute value of the result?  If not, negative values will be clipped to zero.
    //
    //    DICT - a learned dictionary of feature templates, in the format output by hmax_s.SampleFeatures, with
    //    the additional requirement that each template's values have been pre-normalized to have mean 0 and
    //    sum-of-squares 1.

    /*----------------------------------------------------------------------------------------------------------------*/

    int FCount() const { return m_fCount; };

private:

    bool  m_zero;
    float m_thres;
    bool  m_abs;
    int   m_pfCount;
    int   m_yxCountMax;
    int   m_fCount;

    const double *m_yxCounts; // Edge size for each feature.
    const float  *m_vals;     // Feature values, size [PFCOUNT * YXCOUNTMAX * YXCOUNTMAX * FCOUNT].

    float ComputeUnit(const Layer * const in[], float yc, float xc, int f) const;

};

/**********************************************************************************************************************/

#endif
