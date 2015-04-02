#include "ndpfilter.h"

NDPFilter::NDPFilter(bool zero, float thres, bool abs, const dictionary *dict) {

    m_zero  = zero;
    m_thres = thres;
    m_abs   = abs;

    m_yxCounts 	= dict->fSizes;
    m_vals  	= dict->fVals;

    m_pfCount    = dict->pfCount;
    m_yxCountMax = dict->yxCountMax;
    m_fCount     = dict->fCount;
}

/**********************************************************************************************************************/

float NDPFilter::ComputeUnit(const Layer * const in[], float yc, float xc, int f) const {

    int yxCount = (int)m_yxCounts[f];

    // Find our [YXCOUNT * YXCOUNT] sized receptive field in the input layer.

    int yi1, xi1, dummy;
    if (!in[0]->GetYRFNear(yc, yxCount, yi1, dummy)) return UNKNOWN;
    if (!in[0]->GetXRFNear(xc, yxCount, xi1, dummy)) return UNKNOWN;

    // Now iterate over template F and the receptive field.

    float res   = 0.0f;
    float sumv  = 0.0f;
    float sumv2 = 0.0f;

    for (int pf = 0         ; pf < m_pfCount; pf++     ) {
    for (int xi = xi1, j = 0; j  < yxCount  ; xi++, j++) {
    for (int yi = yi1, i = 0; i  < yxCount  ; yi++, i++) {

        float w = m_vals[((f * m_yxCountMax + j) * m_yxCountMax + i) * m_pfCount + pf];

        float v = in[0]->Get(yi, xi, pf);
        if (v == UNKNOWN) return UNKNOWN;

        res   += w * v;
        sumv  += v;
        sumv2 += v * v;

    }
    }
    }

    // Now compute the result.

    float n = m_pfCount * yxCount * yxCount;

    float norm;
    if (m_zero) {
        norm = sqrtf(sumv2 - sumv * sumv / n);
    } else {
        norm = sqrtf(sumv2);
    }

    norm = fmaxf(norm, m_thres * sqrtf(n));

    if (m_abs) {
        res = fabsf(res / norm);
    } else {
        res = fmaxf(res / norm, 0.0f);
    }

    return res;

}
