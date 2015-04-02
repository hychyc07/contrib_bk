#include "layer.h"
/**********************************************************************************************************************/

Layer::Layer(int ySize, int xSize, int fSize, float yStart, float ySpace, float xStart, float xSpace) {

    m_ySize  = ySize;
    m_xSize  = xSize;
    m_fSize  = fSize;
    m_yStart = yStart;
    m_ySpace = ySpace;
    m_xStart = xStart;
    m_xSpace = xSpace;

    m_array = (float *)malloc(ySize * xSize * fSize * sizeof(float));

}

/**********************************************************************************************************************/

Layer::Layer(int yxSize, int fSize, float yxStart, float yxSpace) {

    m_ySize  = yxSize;
    m_xSize  = yxSize;
    m_fSize  = fSize;
    m_yStart = yxStart;
    m_ySpace = yxSpace;
    m_xStart = yxStart;
    m_xSpace = yxSpace;

    m_array = (float *)malloc(yxSize * yxSize * fSize * sizeof(float));

}

/**********************************************************************************************************************/

Layer::~Layer() {

    free(m_array);

}



/**********************************************************************************************************************/

void Layer::SetLayer(const float *in) {

    //if (mxGetClassID(in) != mxSINGLE_CLASS) mexErrMsgTxt("input is not a single array");
    //if (mxGetNumberOfDimensions(in) > 3) mexErrMsgTxt("input has more than 3 dimensions");

    //int ny = (mxGetNumberOfDimensions(in) < 1) ? 1 : mxGetDimensions(in)[0];
    //int nx = (mxGetNumberOfDimensions(in) < 2) ? 1 : mxGetDimensions(in)[1];
    //int nf = (mxGetNumberOfDimensions(in) < 3) ? 1 : mxGetDimensions(in)[2];

    //if (ny != m_ySize) mexErrMsgTxt("input y size is different");
    //if (nx != m_xSize) mexErrMsgTxt("input x size is different");
    //if (nf != m_fSize) mexErrMsgTxt("input f size is different");

    memcpy(m_array, in, m_ySize * m_xSize * m_fSize * sizeof(float));

}

/**********************************************************************************************************************/

float *Layer::GetLayer() const {

    //mwSize size[3];
    //size[0] = m_ySize;
    //size[1] = m_xSize;
    //size[2] = m_fSize;

    float *out = (float *) malloc(m_ySize * m_xSize * m_fSize * sizeof(float));

    memcpy(out, m_array, m_ySize * m_xSize * m_fSize * sizeof(float));

    return out;

}



/**********************************************************************************************************************/

bool Layer::GetYRFNear(float c, int n, int &i1, int &i2) const {

    int j1, j2;
    RFNear(m_ySize, m_yStart, m_ySpace, c, n, i1, i2, j1, j2);

    return (i1 == j1) && (i2 == j2);

}

/**********************************************************************************************************************/

bool Layer::GetXRFNear(float c, int n, int &i1, int &i2) const {

    int j1, j2;
    RFNear(m_xSize, m_xStart, m_xSpace, c, n, i1, i2, j1, j2);

    return (i1 == j1) && (i2 == j2);

}

/**********************************************************************************************************************/

bool Layer::GetYRFDist(float c, float r, int &i1, int &i2) const {

    int j1, j2;
    RFDist(m_ySize, m_yStart, m_ySpace, c, r, i1, i2, j1, j2);

    return (i1 == j1) && (i2 == j2);

}

/**********************************************************************************************************************/

bool Layer::GetXRFDist(float c, float r, int &i1, int &i2) const {

    int j1, j2;
    RFDist(m_xSize, m_xStart, m_xSpace, c, r, i1, i2, j1, j2);

    return (i1 == j1) && (i2 == j2);

}

/**********************************************************************************************************************/

bool Layer::GetYRFNear(float c, int n, int &i1, int &i2, int &j1, int &j2) const {

    RFNear(m_ySize, m_yStart, m_ySpace, c, n, i1, i2, j1, j2);

    return (i1 <= i2);

}

/**********************************************************************************************************************/

bool Layer::GetXRFNear(float c, int n, int &i1, int &i2, int &j1, int &j2) const {

    RFNear(m_xSize, m_xStart, m_xSpace, c, n, i1, i2, j1, j2);

    return (i1 <= i2);

}

/**********************************************************************************************************************/

bool Layer::GetYRFDist(float c, float r, int &i1, int &i2, int &j1, int &j2) const {

    RFDist(m_ySize, m_yStart, m_ySpace, c, r, i1, i2, j1, j2);

    return (i1 <= i2);

}

/**********************************************************************************************************************/

bool Layer::GetXRFDist(float c, float r, int &i1, int &i2, int &j1, int &j2) const {

    RFDist(m_xSize, m_xStart, m_xSpace, c, r, i1, i2, j1, j2);

    return (i1 <= i2);

}

/**********************************************************************************************************************/

void Layer::RFNear(int t, float s, float d, float c, int n, int &i1, int &i2, int &j1, int &j2) {

    float dd = 1.0f / d;

    j1 = (int)ceilf((c - s) * dd - 0.5f * (float)n - 0.001f);
    j2 = j1 + n - 1;

    i1 = min(max(j1,  0), t    );
    i2 = min(max(j2, -1), t - 1);

}

/**********************************************************************************************************************/

void Layer::RFDist(int t, float s, float d, float c, float r, int &i1, int &i2, int &j1, int &j2) {

    float dd = 1.0f / d;

    j1 = (int)ceilf ((c - r - s) * dd - 0.001f);
    j2 = (int)floorf((c + r - s) * dd + 0.001f);

    i1 = min(max(j1,  0), t    );
    i2 = min(max(j2, -1), t - 1);

}
