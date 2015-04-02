#pragma once

class AMem
{
	private:
	
	int Episodes[25][1000];
	int NumEpi;
	int **data; 
	int **WhubEp;
	int N;
	double Uini[1000];
	int VnetAct[1000];
	int WHub2Epim[1000][42];
	int WHub2EpimT[42][1000];
	int WAct2Epim[1000][9];
	int OHub[42];
	int MemID[6], OLap[6], NovelObID[6], NNovobs;
	int VSSP[42], SumVSSP;
	int Action[9];
	int IndexM[5],PEnd;
	int NRelPast[5][1000];
	int NowObAcSeq[5][1000];
	double HubTopDown[5][42],SumTopDown;
	int PlanPastExp[1000];
			
	public:
		AMem();
		~AMem();
		void MemControl();
        int TDMemCompHub(int Nrelev);
		int FindOverlap(int NW);
		void InitializeAM();
		void DumpAM();
		void MemMaint();
		int RememberPast();
		void RetrievalFromCue(int Nrelpos);
		int Random_Zel(int lim);
		void RipRealSequence(int nme);
		void PlanFromPastXP(int NWine, int Noverlapp);
		void MemComb(int NWiner);
		void XploreCombact(int Ola);
		
}; 