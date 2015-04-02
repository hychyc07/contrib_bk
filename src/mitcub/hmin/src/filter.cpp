#include "filter.h"

/**********************************************************************************************************************/

void Filter::ComputeLayer(const Layer * const in[], Layer *out) const {

    for (int f  = 0; f  < out->FSize(); f ++) {
    for (int xi = 0; xi < out->XSize(); xi++) {

        float xc = out->XCenter(xi);

        for (int yi = 0; yi < out->YSize(); yi++) {

            float yc = out->YCenter(yi);

            float val = ComputeUnit(in, yc, xc, f);
            out->Set(yi, xi, f, val);

        }

    }
    }

}
