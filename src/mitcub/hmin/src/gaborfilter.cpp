#include "gaborfilter.h"

/**********************************************************************************************************************/

GaborFilter::GaborFilter(int yxCount, float aspect, float lambda, float sigma, int fCount) {

    const float pi = 3.1415927410125732f;

    m_yxCount = yxCount;
    m_fCount  = fCount;

    m_gabors = (float *)malloc(yxCount * yxCount * fCount * sizeof(float));

    for (int f = 0; f < fCount; f++) {

        float *ptr;

        // First we generate the filter.

        float sum   = 0.0f;
        float sumsq = 0.0f;

        ptr = m_gabors + f * yxCount * yxCount;

        for (int j = 0; j < yxCount; j++) {
        for (int i = 0; i < yxCount; i++) {

            float jj = 0.5f * (float)(1 - yxCount) + (float)j;
            float ii = 0.5f * (float)(1 - yxCount) + (float)i;

            float theta = (float)f / (float)fCount * pi;

            float y = jj * sinf(theta) + ii * cosf(theta);
            float x = jj * cosf(theta) - ii * sinf(theta);

            float e;
            if (sqrtf(x * x + y * y) <= 0.5f * (float)yxCount) {
                e = expf(-(x * x + aspect * aspect * y * y) / (2.0f * sigma * sigma));
                e = e * cosf(2.0f * pi * x / lambda);
            } else {
                e = 0.0f;
            }

            *ptr++ = e;

            sum   += e;
            sumsq += e * e;

        }
        }

        // Now we normalize it to have mean 0 and total energy 1.

        float n = (float)(yxCount * yxCount); 
        float mean = sum / n;
        float stdv = sqrtf(sumsq - sum * sum / n);

        ptr = m_gabors + f * yxCount * yxCount;

        for (int j = 0; j < yxCount; j++) {
        for (int i = 0; i < yxCount; i++) {
            float e = *ptr;
            *ptr++ = (e - mean) / stdv;
        }
        }

    }

}

/**********************************************************************************************************************/

GaborFilter::~GaborFilter() {

    free(m_gabors);

}

/**********************************************************************************************************************/

float GaborFilter::ComputeUnit(const Layer * const in[], float yc, float xc, int f) const {

    // Find our [YXCOUNT * YXCOUNT] sized receptive field in the input layer.

    int yi1, yi2, xi1, xi2;
    if (!in[0]->GetYRFNear(yc, m_yxCount, yi1, yi2)) return UNKNOWN;
    if (!in[0]->GetXRFNear(xc, m_yxCount, xi1, xi2)) return UNKNOWN;

    // Now apply filter F to the receptive field.

    float res = 0.0f;
    float len = 0.0f;

    const float *ptr = m_gabors + f * m_yxCount * m_yxCount;

    for (int xi = xi1; xi <= xi2; xi++) {
    for (int yi = yi1; yi <= yi2; yi++) {

        float w = *ptr++;

        float v = in[0]->Get(yi, xi, 0);
        if (v == UNKNOWN) return UNKNOWN;

        res += w * v;
        len += v * v;

    }
    }

    res = fabsf(res);
    if (len > 0.0f) res /= sqrtf(len);

    return res;

}
