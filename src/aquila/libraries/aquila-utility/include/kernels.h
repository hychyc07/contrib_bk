void findBestMatchOnDevice(dim3 grid, dim3 block, float *outputs,  int *winner, int numOutputs);
void propogateInputOnDevice(dim3 grid, dim3 block, float *inputs, float *weights, float *outputs, int numInputs, int sequenceId, int numOutputs);
void updateWeightsOnDevice(dim3 grid, dim3 block, float *inputs, float *weights, int *winner, float sigma, int numInputs, int sequenceId, int numOutputs, int neighbourhoodSize, float initLearningRate,int numIterations, int currentIteration);
