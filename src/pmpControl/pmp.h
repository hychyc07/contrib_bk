class PassiveMotionParadigm
{
	private:
	
		double Jan[10];
		double JanL[10]; //initial Joint angles


		double x_ini; // Using PMP+ Symmetry So Right and Left Arm movements are
		// coordinated by the forward inverse model of the left arm only
		double y_ini;
		double z_ini; 

		/* double x_ini=-327; // Using PMP+ Symmetry So Right and Left Arm movements are
		// coordinated by the forward inverse model of the left arm only
		double y_ini=-318;
		double z_ini=77;*/ 
		double x_iniL; // Init Config / O/p of target generator
		double y_iniL;
		double z_iniL;

		double x_fin,y_fin,z_fin;
		double x_finL,y_finL,z_finL;

		double Gam_Arr[20000],Gam_Arry[20000],Gam_Arrz[20000],q1[20000],q2[20000],q3[20000],q4[20000],q5[20000];
		double q6[20000],q7[20000],q8[20000],q9[20000],q10[20000];

		double Gam_ArrL[20000],Gam_ArryL[20000],Gam_ArrzL[20000],q1L[20000],q2L[20000],q3L[20000],q4L[20000],q5L[20000];
		double q6L[20000],q7L[20000],q8L[20000],q9L[20000],q10L[20000];

		int ang1,ang2,ang3,ang4,ang5,ang6,ang7,ang8,ang9,ang10,konst;

		int ang1L,ang2L,ang3L,ang4L,ang5L,ang6L,ang7L,ang8L,ang9L,ang10L;
		int angCup,angT1,angT2,angT3,angI1,angI2,angM1,angM2,angRP,angCupL,angTL1,angTL2,angTL3,angIL1,angIL2,angML1,angML2,angRPL;

		double target[3],targetL[3];
		double X_pos[3],ffield[3],JoVel[20],X_posL[3],ffieldL[3];
		double *ptr;
		//char* objEcYar ="ball";
		double KFORCE,ITERATION,RAMP_KONSTANT,t_dur,KOMP_JANG,KOMP_WAISZT,KOMP_WAISZT2,J2H,J7H,J8H,J9H,J3H;
		float s[5000];
		int ParamStiff;
	public:
		PassiveMotionParadigm(int aParamStiff);
		~PassiveMotionParadigm();
		double* forward_Kinematics(double *u , int l);
		double* forward_KinematicsL(double *uL , int lef);
		double* forward_KinematicsLRH(double *uLRH , int lefRH);
		double* forcefield(double *w, double*v);
		double* forcefieldL(double *wL, double*vL);
		double* PMP(double *force,double *forceL);
		double Gamma_Int(double *Gar,int n);
		double Gamma(int _Time);
		void MotCon(double T1, double T2, double T3,double TL1, double TL2, double TL3);
		void MessagePass(int Activate);
		void MessageDevDriver(int Activate1);
		void initiCubUp();
		void CubGrazp1();
		void CubGrazp2();
		void CubRelease();
		void Grab();
		void Kompliance();
}; 