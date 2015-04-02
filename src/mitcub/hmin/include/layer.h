#ifndef _LAYER_H_
#define _LAYER_H_
/***********************************************************************************************************************

A layer is a three-dimensional array of units which collectively represent the activity of some set of features at
each location in a 2-D grid of points in retinal space.  A layer can be an input layer (containing an image) or the
result of a filtering operation performed on another layer (or layers).  Each unit in a layer is identified by three
zero-based integer indices: Y, X, and F.

The F index is the feature number.  In an input layer, the F dimension has size 1: the only feature is the pixel
intensity at that point.  Other layers have more features, for example, the responses to a set of gabor filters, or
a set of higher-order feature templates.

The Y and X indices identify each grid point in a layer.  In addition to these integer indices, which only have
meaning within the individual layer, each grid point is also associated with a real-valued (Y, X) position in retinal
space.  This real-valued coordinate system is consistent across layers, and all local operations, such as pooling and
convolution, are defined with reference to it.  A given unit finds its input units in other layers based on proximity
in retinal space.  Via appropriate relative arrangement of grid points between a layer and its input layer(s),
different types of pooling can be defined.  (See http://cbcl.mit.edu/jmutch/cns/doc/package.html#common.)

HMAX models are typically multi-scale.  Representing multiple scales requires one layer object per scale.

***********************************************************************************************************************/
#include "main.h"

class Layer {
public:

    Layer(int ySize, int xSize, int fSize, float yStart, float ySpace, float xStart, float xSpace);
    //
    // Allocate a new layer of size [YSIZE * XSIZE * FSIZE].  The remaining four parameters determine the (Y, X) grid
    // positions in the common, real-valued retinal coordinate system.  YSTART is the position of Y index 0, and YSPACE
    // determines the spacing between adjacent grid points along Y.  XSTART and XSPACE are similar.  (See example.cpp.)

    Layer(int yxSize, int fSize, float yxStart, float yxSpace);
    //
    // Same as above but assumes layers are square.

    ~Layer();

    /*----------------------------------------------------------------------------------------------------------------*/

    int   YSize () const { return m_ySize ; };
    int   XSize () const { return m_xSize ; };
    int   FSize () const { return m_fSize ; };
    float YSpace() const { return m_ySpace; };
    float XSpace() const { return m_xSpace; };
    //
    // These methods just allow access to some of the quantities provided in the constructor.

    /*----------------------------------------------------------------------------------------------------------------*/

    void  Set(int yi, int xi, int f, float val);
    float Get(int yi, int xi, int f) const;
    //
    // Set or retrieve the value of the unit at indices (YI, XI, F).

    /*----------------------------------------------------------------------------------------------------------------*/

    void     SetLayer(const float *in);
    float *GetLayer() const;
    //
    // Set or retrieve the values of all units in the layer.

    /*----------------------------------------------------------------------------------------------------------------*/

    float YCenter(int i) const;
    float XCenter(int i) const;
    //
    // For either the Y or X dimensions, return the real-valued position in common retinal space that corresponds to
    // integer index i.

    /*----------------------------------------------------------------------------------------------------------------*/

    bool GetYRFNear(float c, int n, int &i1, int &i2) const;
    bool GetXRFNear(float c, int n, int &i1, int &i2) const;
    //
    // For either the Y or X dimensions, find the N nearest indices to position C in the real-valued retinal coordinate
    // system.  The range of indices will be returned in I1 and I2.  If any of the found indices are outside the valid
    // range [0 YSIZE-1] or [0 XSIZE-1], only the valid part of the range will be returned in I1 and I2, and the
    // function's return value will be false.  If N valid indices can be returned, the return value will be true.

    /*----------------------------------------------------------------------------------------------------------------*/

    bool GetYRFDist(float c, float r, int &i1, int &i2) const;
    bool GetXRFDist(float c, float r, int &i1, int &i2) const;
    //
    // Similar to Get*RFNear above, except instead of finding the N nearest indices, we find all indices within
    // distance R of C, both specified in real-value retinal coordinates.  If any of the indices found are invalid,
    // the range in I1/I2 is truncated and the return value will be false, otherwise we return true.

    /*----------------------------------------------------------------------------------------------------------------*/

    bool GetYRFNear(float c, int   n, int &i1, int &i2, int &j1, int &j2) const;
    bool GetXRFNear(float c, int   n, int &i1, int &i2, int &j1, int &j2) const;
    bool GetYRFDist(float c, float r, int &i1, int &i2, int &j1, int &j2) const;
    bool GetXRFDist(float c, float r, int &i1, int &i2, int &j1, int &j2) const;
    //
    // Variants of Get*RFNear and Get*RFDist which return both the valid indices found (in I1/I2) and the entire range
    // found, whether valid or invalid, in J1/J2.  Returns false if the valid range (in I1/I2) is empty, otherwise
    // returns true.

private:

    int    m_ySize;
    int    m_xSize;
    int    m_fSize;
    float  m_yStart;
    float  m_ySpace;
    float  m_xStart;
    float  m_xSpace;
    float *m_array;

    static void RFNear(int t, float s, float d, float c, int   n, int &i1, int &i2, int &j1, int &j2);
    static void RFDist(int t, float s, float d, float c, float r, int &i1, int &i2, int &j1, int &j2);

};
/**********************************************************************************************************************/

inline void Layer::Set(int yi, int xi, int f, float val) {

    m_array[(f * m_xSize + xi) * m_ySize + yi] = val;

}

/**********************************************************************************************************************/
inline float Layer::Get(int yi, int xi, int f) const {

    return m_array[(f * m_xSize + xi) * m_ySize + yi];

}

/**********************************************************************************************************************/

inline float Layer::YCenter(int i) const {

    return m_yStart + (float)i * m_ySpace;

};

/**********************************************************************************************************************/

inline float Layer::XCenter(int i) const {

    return m_xStart + (float)i * m_xSpace;

};

#endif
