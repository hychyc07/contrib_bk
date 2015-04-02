#ifndef TBG_H
#define TBG_H

#include <iostream>
#include <fstream>
#include <math.h>

using namespace std;

namespace iCub{
namespace pmplib{
namespace core{

class TimeBaseGenerator
{
	private:
		double T_init;
		double T_dur;
		double SlopeRamp;
		double Gamma_max;
		double alpha;
		double offset;
		//ofstream o1;
	public:
		double csidot;
		TimeBaseGenerator();
		TimeBaseGenerator(double _T_init, double _T_dur, double _SlopeRamp, double _alpha);
		~TimeBaseGenerator();
		TimeBaseGenerator& operator=(const TimeBaseGenerator &tbg);
		double& getT_dur();
		double& getT_init();
		double& getSlopeRamp();
		double& getAlpha();
		void setT_dur(double _T_dur);
		void setT_init(double _T_init);
		void setSlopeRamp(double _SlopeRamp);
		void setAlpha(double _alpha);
		double calculateGamma(unsigned int time, bool offset=false);
		void scaleFactor();
};
}
}
}

#endif

