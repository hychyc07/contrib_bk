#ifndef _FILTER_H_
#define _FILTER_H_
/***********************************************************************************************************************

Defines the abstract Filter class.  Filters take one or more layers as input and produce one layer as output.

***********************************************************************************************************************/
#include "main.h"
#include "layer.h"

class Filter {
public:

    void ComputeLayer(const Layer * const in[], Layer *out) const;
    //
    // Run the filter on the input layer(s), storing values in the output layer.

private:

    virtual float ComputeUnit(const Layer * const in[], float yc, float xc, int f) const =0;
    //
    // Compute the value of a single unit.  This function is provided by each subclass, and will be called by
    // ComputeLayer once for each unit in the layer.
    //
    //    YC, XC - the output unit's coordinates in the real-valued retinal space.
    //
    //    F - the output unit's feature index.

};


#endif
