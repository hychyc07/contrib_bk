#ifndef _MAIN_H_
#define _MAIN_H_
/***********************************************************************************************************************

The main header file.  Contains some essential definitions and also #includes all the other header files in this
directory.  This is the only file a model's ".cpp" file needs to directly #include.

***********************************************************************************************************************/


#include <math.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>

/**********************************************************************************************************************/

// Units in a layer can assume the value UNKNOWN for several reasons:
// - In an input layer, UNKNOWNs are used as padding when the input image has a different aspect ratio.
// - A non-input unit might be unable to compute its value for some reason, such as:
//   - Some of its inputs might be UNKNOWN.
//   - It might be trying to apply a large filter too close to an edge.

const float UNKNOWN = (-FLT_MAX);

inline int min(int a, int b) {
    return (a <= b) ? a : b;
}

inline int max(int a, int b) {
    return (a >= b) ? a : b;
}

inline float fminf(float a, float b) {
    return (a <= b) ? a : b;
}

inline float fmaxf(float a, float b) {
    return (a >= b) ? a : b;
}

#endif
