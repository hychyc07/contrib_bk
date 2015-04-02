// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright: 2011 RBCS & UC3M
 * Author: Juan G Victores
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#define MORE_THAN_THIS_YELLOW 1.0
#define MORE_THAN_THIS_RED 2.0

using namespace yarp::os;
using namespace yarp::sig;

int main(int argc, char *argv[]) {
    Network yarp;
    if (!yarp.checkNetwork()) {
        printf("No yarp network, bye!\n");
        return -1;
    }

    BufferedPort<Vector> inPort;
    BufferedPort<ImageOf<PixelRgb> > outPort;
    inPort.open("/colorfulOutput/vector:i");
    outPort.open("/colorfulOutput/img:o");
    PixelRgb green(0,255,0);
    PixelRgb yellow(255,255,0);
    PixelRgb red(255,0,0);
    while (true) {
        printf("waiting for input\n");
        Vector *input = inPort.read();
        if (input!=NULL) {
            ImageOf<PixelRgb>& img = outPort.prepare();
            img.resize(1,1);
            img(0,0) = green;
            for (int i=0; i<input->size(); i++) {
                printf("%f ",(*input)[i]);
                if ((*input)[i]>MORE_THAN_THIS_YELLOW) img(0,0) = yellow;
                if ((*input)[i]>MORE_THAN_THIS_RED) img(0,0) = red;
            }
            printf("\n");
            outPort.write();
        }
    }
    return 0;
}

