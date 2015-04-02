#pragma once

class Smallworld
{
	private:
    double Col[30],Word[30],Shape[30],ProvHub[42],OCHub[42],LocalMapAct[90],ipCol[3],ipShap[3],RdisActW[2][30];
	int MapstoProvHub[36][90],ProvHubtoMaps[90][36],ColW[30][3], WorW[30][120];
	int WordIn[2][120], inhib[30][30],NumWords;


	
	public:
		Smallworld(void);
		~Smallworld(void);
		void InitializeSW();
		void WordEncode(int numu);
		void GetLocalAct(int numW);
		void Retroactivate(int PropWC);
		void HubRep();

};
