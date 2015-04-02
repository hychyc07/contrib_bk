#ifndef __ICUB_CALIBRATIONALGORITHMLIST_H__
#define __ICUB_CALIBRATIONALGORITHMLIST_H__

//ALGORITHM DEFINITION
//	Add a DEFINE(Your_Algorithm_ClassName) to ALGORITHM_TABLE. Don't forget the backslash.
#define ALGORITHM_TABLE(DEFINE) \
	DEFINE(DummyCA) \
	DEFINE(LeastSquareCA) \
	DEFINE(DataCollectorCA) \
	DEFINE(ActivationAreaCA) \
	DEFINE(CholeskyLeastSquareCA) \
	DEFINE(MeshIntersectionsCA) \
	DEFINE(EnvironmentCA) \
	DEFINE(ModuleFittingCA)


//ALGORITHM INCLUSION
//	Add a #include INCLUDE(Your_Algorithm_HeaderFileName_Without_Extension)
#include INCLUDE(DummyCA)
#include INCLUDE(LeastSquareCA)
#include INCLUDE(DataCollectorCA)
#include INCLUDE(ActivationAreaCA)
#include INCLUDE(CholeskyLeastSquareCA)
#include INCLUDE(MeshIntersectionsCA)
#include INCLUDE(EnvironmentCA)
#include INCLUDE(ModuleFittingCA)

#endif
