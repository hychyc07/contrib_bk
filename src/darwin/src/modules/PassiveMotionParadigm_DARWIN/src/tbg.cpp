#include "darwin/tbg.h"


TimeBaseGenerator::TimeBaseGenerator()
{
	T_dur = 1.0;
	T_init = 0.003;
	SlopeRamp = 0.001;
	alpha = 1.00001;
	scaleFactor();
	offset = 0.0;

	printf("--> Tbg object created with default parameters\n");
	//o1.open("Gamma_test.txt");
}

TimeBaseGenerator::TimeBaseGenerator (double _T_init, double _T_dur, double _SlopeRamp, double _alpha) : 
				  T_init(_T_init), T_dur(_T_dur), SlopeRamp(_SlopeRamp), alpha(_alpha)
{
	printf("--> Tbg object created\n");
}

TimeBaseGenerator::~TimeBaseGenerator()
{
	//o1.close();
}

// copy operator overloading
TimeBaseGenerator& TimeBaseGenerator::operator=(const TimeBaseGenerator &tbg)
{
	if (this != &tbg) // protect against self-copy
	{
		this->T_init = tbg.T_init;
		this->T_dur = tbg.T_dur;
		this->SlopeRamp = tbg.SlopeRamp;
		this->alpha = tbg.alpha;
		this->Gamma_max = tbg.Gamma_max;
	}

	return *this;
}

double TimeBaseGenerator::getT_dur()
{
	return T_dur;
};
	
double TimeBaseGenerator::getT_init()
{
	return T_init;
};
	
double TimeBaseGenerator::getSlopeRamp()
{
	return SlopeRamp;
};

double TimeBaseGenerator::getAlpha()
{
	return alpha;
};
	
void TimeBaseGenerator::setT_dur(double _T_dur)
{
	T_dur = _T_dur;
	scaleFactor();
	printf("--> T_dur value set to %4.3f\n",T_dur);
};
	
void TimeBaseGenerator::setT_init(double _T_init)
{
	T_init = _T_init;
	scaleFactor();
	printf("--> T_init value set to %4.3f\n",T_init);
};
	
void TimeBaseGenerator::setSlopeRamp(double _SlopeRamp)
{
	SlopeRamp = _SlopeRamp;
	scaleFactor();
	printf("--> SlopeRamp value set to %4.3f\n",SlopeRamp);
};

void TimeBaseGenerator::setAlpha(double _alpha)
{
	alpha = _alpha;
	scaleFactor();
	printf("--> Alpha value set to %f\n",alpha);
}

double TimeBaseGenerator::calculateGamma(unsigned int time, bool os)
{
	// [Time] = ms, [T_ramp] = [T_dur] = sec
	double T_ramp = time * SlopeRamp;
	double T_curr = (T_ramp - T_init)/ T_dur;
	double T_temp = T_init + T_dur - T_ramp;
	double T_window;
	if(T_temp > 0 && T_ramp >= T_init)
	{
		T_window = 1;
	}
	else
	{
		T_window = 0;
	}
	
	double csi=(6*pow(T_curr,5))-(15*pow(T_curr,4))+(10*pow(T_curr,3));  //6z^5-15z^4+10z^3
	csidot=(30*pow(T_curr,4))-(60*pow(T_curr,3))+(30*pow(T_curr,2)); //csi_dot=30z^4-60z^3+30z^2
	double Gamma = (csidot*T_window)/(T_dur *(alpha - (csi*T_window)));///Gamma_max; // Function maximum = 1
	//o1 << time << "\t" << Gamma+offset << endl;
	if (os)
	{
		if(time > (T_dur+T_init)-5)
			return Gamma;
		else
			return Gamma+offset;
	}
	else	return Gamma;
};

void TimeBaseGenerator::scaleFactor()
{
/*	double T_max = (T_dur -27*SlopeRamp)/T_dur;

	double csi_max =(6*pow(T_max,5))-(15*pow(T_max,4))+(10*pow(T_max,3));  //6z^5-15z^4+10z^3
	double csidot_max =(30*pow(T_max,4))-(60*pow(T_max,3))+(30*pow(T_max,2)); //csi_dot=30z^4-60z^3+30z^2
	Gamma_max = (csidot_max)/(T_dur*(1.0001 - csi_max));
	cout << "T_max: " << T_max << endl;
	cout << "Gamma_max :" << Gamma_max << endl;
*/
	double csi, csidot, Gamma, tmax;
	tmax = 0;
	Gamma_max = 0;
	//ofstream o1("G.txt");
	//ofstream o2("Csi.txt");
	//ofstream o3("Csidot.txt");
	for (int t = (int)T_init/SlopeRamp; t < (int)(T_dur/SlopeRamp); t++)
	{
		double ti = t*SlopeRamp/T_dur;
		csi=(6*pow(ti,5))-(15*pow(ti,4))+(10*pow(ti,3));  //6z^5-15z^4+10z^3
		csidot=(30*pow(ti,4))-(60*pow(ti,3))+(30*pow(ti,2)); //csi_dot=30z^4-60z^3+30z^2
		Gamma = (csidot)/(T_dur *(alpha - (csi)));
		if(Gamma_max < Gamma)
		{
			Gamma_max = Gamma;
			tmax = ti;
		}
		//o1 << t << "\t" << Gamma << endl;	
		//o2 << t << "\t" << csi << endl;
		//o3 << t << "\t" << csidot << endl;
	}
	//o1 << "GAMMA MAX: " << Gamma_max <<endl;
	//o1.close();
	//o2.close();
	//o3.close();
}
