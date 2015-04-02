//GPU kernel wrapper declarations
void resetDeltaWeightsOnDevice(dim3 grid, dim3 block, cudaStream_t stream, int numWeights, int numIONeurons, float *deltaWeight, float *individualError);
void setInitStatesOnDevice(dim3 grid, dim3 block, cudaStream_t stream, float initState, float *activity, int numNeurons, int numIONeurons, int numFastNeurons);
void resetParametersOnDevice(dim3 grid, dim3 block, cudaStream_t stream, int numNeurons, int maxSequenceSteps, float *delta, float *previousDelta, float *potential, float *previousPotential, float *error);
void updateWeightsOnDevice(dim3 grid, dim3 block, float learningRate, float momentum, float *weight, float *deltaWeight, float *previousDeltaWeight, int numWeights);
void forwardPassV1onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int sequenceOffset, float *activity, float *input, float *weight, float *previousPotential, float *error, float *potential, int *deltaT, int numNeurons, int numIONeurons);
void forwardPassV2onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int sequenceOffset, float *activity, float *input, float *weight, int numNeurons, int numIONeurons, float *buffer);
void forwardPassV21onDevice(dim3 grid, dim3 block, int smemSize, cudaStream_t stream, int step, int sequenceOffset, float *activity, float *input, float *buffer, float *potential, float *weight, float *previousPotential, float *error, int *deltaT, int numNeurons, int numIONeurons);
void backwardPassV1onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int sequenceOffset, int numNeurons, int numIONeurons, float *input, float *activity, float *delta, float *deltaWeight, float *previousDelta, float *error, float *individualError, int *deltaT, float *weight);
void backwardPassV11onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step,  int numNeurons, int numIONeurons,  float *activity, float *delta, float *previousDelta, int *deltaT, float *weight);
void backwardPassV2onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int sequenceOffset, int numNeurons, int numIONeurons, float *input, float *activity, float *delta, float *deltaWeight, float *previousDelta, float *error, float *individualError, int *deltaT, float *weight, float *buffer);
void backwardPassV21onDevice(dim3 grid, dim3 block, int smemSize, cudaStream_t stream, float *input, float *output, int numNeurons, int numIONeurons);
void backwardPassV3onDevice(dim3 grid, dim3 block, cudaStream_t stream, int step, int numNeurons, int numIONeurons, float *activity, float *delta, float *previousDelta, float *deltaWeight, int *deltaT, float *weight);
void sumDeltaWeightsP2PonDevice(dim3 grid, dim3 block, int numWeights, float *masterDeltaWeight, float *peerDeltaWeight);
void updateWeightsP2PonDevice(dim3 grid, dim3 block, int numWeights, float learningRate, float momentum, float *masterWeight, float *peerWeight, float *deltaWeight, float *previousDeltaWeight);
void sumErrorP2PonDevice(dim3 grid, dim3 block, float *masterError, float *peerError);
void reduceOnDevice(int size, dim3 grid, dim3 block, int smemSize, cudaStream_t stream, float *input, float *output, unsigned int n, bool nIsPow2);
