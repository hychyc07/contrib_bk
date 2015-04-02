README.txt
----------
Note: may not be updated, best check the code! (or email jcgvicto at ing dot uc3m dot es, xD)

A version of cartesianForce that we branch because we incorporate a dependency on the WBC library and therefore also on TAO dynamics.

tools:
* simTaoFltk: 5 and 7DOF graphical simulators (sim5 and sim7). Many examples if you ENABLE_experimental (ccmake)
* icubParams: just gets H and COMs from iDyn
* icubInertiasDirect: uses iDyn getCOM directly to move the inertia tersors to the joints
* icubInertias: here we try to express each inertia in the frame we use in the SAI .xml COM expression
