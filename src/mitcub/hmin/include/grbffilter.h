/***********************************************************************************************************************

Applies a set of learned feature templates at each position in a single layer.  The response to a given template at a
given position is computed using a gaussian radial basis function.

***********************************************************************************************************************/

class GRBFFilter : public Filter {
public:

    GRBFFilter(int yxCountMin, float sigma, const mxArray *dict);
    //
    // Create a new filter of this type.
    //
    //    YXCOUNTMIN - edge size of the smallest feature in DICT.
    //
    //    SIGMA - Standard deviation of the gaussian applied to the distance between the template and the input patch.
    //
    //    DICT - a learned dictionary of feature templates, in the format output by hmax_ss.SparsifyDict.

    ~GRBFFilter();

    /*----------------------------------------------------------------------------------------------------------------*/

    int FCount() const { return m_fCount; };

private:

    int    m_yxCountMin;
    float  m_sigma;
    int    m_fCount;
    int    m_inCountMax;

    int   *m_yxCounts; // Edge size for each feature.
    int   *m_inCounts; // Number of inputs for each feature.

    float *m_vals;  // These all have size [INCOUNTMAX * FCOUNT].  They represent the components of each feature
    int   *m_yOffs; // template: the Y offset, X offset, and F value of each input, and the desired value for that
    int   *m_xOffs; // input.
    int   *m_pfs;

    float ComputeUnit(const Layer * const in[], float yc, float xc, int f) const;

};

/**********************************************************************************************************************/

GRBFFilter::GRBFFilter(int yxCountMin, float sigma, const mxArray *dict) {

    if (mxGetClassID(dict) != mxSTRUCT_CLASS) mexErrMsgTxt("invalid dictionary");
    if (mxGetNumberOfElements(dict) != 1) mexErrMsgTxt("invalid dictionary");

    m_yxCountMin = yxCountMin;
    m_sigma      = sigma;

    m_fCount     = mxGetN(mxGetField(dict, 0, "fVals"));
    m_inCountMax = mxGetM(mxGetField(dict, 0, "fVals"));

    m_yxCounts = (int   *)mxMalloc(               m_fCount * sizeof(int  ));
    m_inCounts = (int   *)mxMalloc(               m_fCount * sizeof(int  ));
    m_vals     = (float *)mxMalloc(m_inCountMax * m_fCount * sizeof(float));
    m_yOffs    = (int   *)mxMalloc(m_inCountMax * m_fCount * sizeof(int  ));
    m_xOffs    = (int   *)mxMalloc(m_inCountMax * m_fCount * sizeof(int  ));
    m_pfs      = (int   *)mxMalloc(m_inCountMax * m_fCount * sizeof(int  ));

    const double *fSizes = (double *)mxGetData(mxGetField(dict, 0, "fSizes")); // [FCOUNT]
    const float  *fVals  = (float  *)mxGetData(mxGetField(dict, 0, "fVals" )); // [INCOUNTMAX * FCOUNT]
    const float  *fMap   = (float  *)mxGetData(mxGetField(dict, 0, "fMap"  )); // [3 * INCOUNTMAX * FCOUNT]

    for (int f = 0; f < m_fCount; f++) {
        m_yxCounts[f] = (int)fSizes[f];
        m_inCounts[f] = 0;
        for (int i = 0; i < m_inCountMax; i++) {
            float val =      fVals[ f * m_inCountMax + i         ];
            int   pf  = (int)fMap [(f * m_inCountMax + i) * 3 + 0] - 1;
            int   yo  = (int)fMap [(f * m_inCountMax + i) * 3 + 1] - 1;
            int   xo  = (int)fMap [(f * m_inCountMax + i) * 3 + 2] - 1;
            if (pf < 0) break;
            m_inCounts[f] = i + 1;
            m_vals [f * m_inCountMax + i] = val;
            m_yOffs[f * m_inCountMax + i] = yo;
            m_xOffs[f * m_inCountMax + i] = xo;
            m_pfs  [f * m_inCountMax + i] = pf;
        }
    }

}

/**********************************************************************************************************************/

GRBFFilter::~GRBFFilter() {

    mxFree(m_yxCounts);
    mxFree(m_inCounts);
    mxFree(m_vals    );
    mxFree(m_yOffs   );
    mxFree(m_xOffs   );
    mxFree(m_pfs     );

}

/**********************************************************************************************************************/

float GRBFFilter::ComputeUnit(const Layer * const in[], float yc, float xc, int f) const {

    int yxCount = m_yxCounts[f];
    int inCount = m_inCounts[f];

    // Find our [YXCOUNT * YXCOUNT] sized receptive field in the input layer.

    int yi1, xi1, dummy;
    if (!in[0]->GetYRFNear(yc, yxCount, yi1, dummy)) return UNKNOWN;
    if (!in[0]->GetXRFNear(xc, yxCount, xi1, dummy)) return UNKNOWN;

    // Now apply template F to the receptive field.

    float res = 0.0f;

    for (int i = 0; i < inCount; i++) {

        float w  = m_vals [f * m_inCountMax + i];
        int   yi = m_yOffs[f * m_inCountMax + i] + yi1;
        int   xi = m_xOffs[f * m_inCountMax + i] + xi1;
        int   pf = m_pfs  [f * m_inCountMax + i];

        float v = in[0]->Get(yi, xi, pf);
        if (v == UNKNOWN) return UNKNOWN;

        float diff = v - w;
        res -= diff * diff;

    }

    float yxRatio = (float)yxCount / (float)m_yxCountMin;

    return expf(res / (2.0f * m_sigma * m_sigma * yxRatio * yxRatio));

}
