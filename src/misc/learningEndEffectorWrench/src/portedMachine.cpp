
#include "iCub/learningEndEffectorWrench/portedMachine.h"

#include "iCub/learningMachine/MachineCatalogue.h"  // for some reason become multiply defined if
#include "iCub/learningMachine/TransformerCatalogue.h"  // these two lines in header, maybe too "inline"??

/************************************************************************/
bool portedMachine::init(ResourceFinder &rf) {
    // initialize catalogue of machine factory
    registerMachines();
    // initialize catalogue of transformers
    registerTransformers();
    // get the name of the machine
    ConstString machine=rf.findFile("machine");
    // load the TRAIN machine portable
    printf("Loading TRAIN machine portable from file '%s'... ",machine.c_str());
    bool ok = mp.readFromFile(machine.c_str());
    if (ok) printf("ok!\n");
    else printf("failed :(\n");
    printf("TRAIN\n%s\n",mp.getWrapped().getInfo().c_str());
    // get the name of the transf
    ConstString transf=rf.findFile("transf");
    // load TRANSF machine portable
    printf("Loading TRANSF machine portable from file '%s'... ",transf.c_str());
    ok = tp.readFromFile(transf.c_str());
    if (ok) printf("ok!\n");
    else printf("failed :(\n");
    printf("TRANSF\n%s\n",tp.getWrapped().getInfo().c_str());
    return 0;
}

/************************************************************************/
Vector portedMachine::predict(const Vector v) {
    Vector transInput = tp.getWrapped().transform(v.subVector(0,4));
    Prediction prediction = mp.getWrapped().predict(transInput);
    return prediction.getPrediction();
}

