#include "maxfilter.h"
/**********************************************************************************************************************/

MaxFilter::MaxFilter(int sCount, int yxCount) {

    m_sCount  = sCount;
    m_yxCount = yxCount;

}

/**********************************************************************************************************************/

float MaxFilter::ComputeUnit(const Layer * const in[], float yc, float xc, int f) const {

    // Re-express YXCOUNT as a distance in real-valued retinal coordinates.

    float yr = in[0]->YSpace() * 0.5f * (float)m_yxCount;
    float xr = in[0]->XSpace() * 0.5f * (float)m_yxCount;

    // Now for each input layer (i.e. each scale) perform a local max over position for feature F.

    float res = UNKNOWN;

    for (int s = 0; s < m_sCount; s++) {

        int yi1, yi2, xi1, xi2;
        in[s]->GetYRFDist(yc, yr, yi1, yi2);
        in[s]->GetXRFDist(xc, xr, xi1, xi2);

        for (int xi = xi1; xi <= xi2; xi++) {
        for (int yi = yi1; yi <= yi2; yi++) {

            float v = in[s]->Get(yi, xi, f);

            res = fmaxf(res, v);

        }
        }

    }

    return res;

}
