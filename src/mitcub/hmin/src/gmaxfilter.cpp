#include "gmaxfilter.h"
GMaxFilter::GMaxFilter(int sCount) {

    m_sCount = sCount;

}

/**********************************************************************************************************************/

float GMaxFilter::ComputeUnit(const Layer * const in[], float yc, float xc, int f) const {

    float res = UNKNOWN;

    for (int s  = 0; s  < m_sCount      ; s ++) {
    for (int xi = 0; xi < in[s]->XSize(); xi++) {
    for (int yi = 0; yi < in[s]->YSize(); yi++) {

        float v = in[s]->Get(yi, xi, f);

        res = fmaxf(res, v);

    }
    }
    }

    return res;

}
