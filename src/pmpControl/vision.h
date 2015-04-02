class VisionSystem
{
	private:
		double a[3][3], det,l1[11],l2[11],Y1,Y2,Y3,Y4;
		double rw,rx,ImagX,ImagY,ImagZ;
		double Sconf[4][3],SconfT[3][4],C[3][3],point[5][4];
		float s[5000];
		int iLocX,iLocY,iLocXL,iLocYL;
	public:
		VisionSystem();
		~VisionSystem();
		double* Vision(int ObjectType);
		double Determinant(double **a,int n);
		void CoFactor(double **a,int n,double **b);
		void Transpose(double **a,int n);
		void ConvImg(double UUC1,double UUV1,double UUC2,double UUV2);
		void GCamSceneR(int ActCamsR);
		void GCamSceneL(int ActCamsL);
		void GCamSceneRred(int ActCamsRred);
		void GCamSceneLred(int ActCamsLred);
		void initHead(double headzero);
};