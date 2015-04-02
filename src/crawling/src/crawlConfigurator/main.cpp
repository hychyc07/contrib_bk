#include <iostream>
#include <fstream>
#include <string>

using namespace std;

void replace_line_left_arm (string& line, string test)
{
	if (test=="Max ")         line = "Max	  10	160.8	80	106	 90	  0	40	60"; 
	if (test=="Min ")         line = "Min	-110	0	   -37	5.5	-90	-90	-20	0"; 
	if (test=="Currents")     line = "Currents      3000          3000          3000          3000          550           1000          1000          400";  
	if (test=="Pid0")         line = "Pid0  24000	50		40		1333	1333	13	0";		
	if (test=="Pid1")         line = "Pid1 	24000	50		40		1333	1333	13	0";		
	if (test=="Pid2")         line = "Pid2 	 8000	0	 	5		1333	1333	13	0";		
	if (test=="Pid3")         line = "Pid3	24000	20		40		1333	1333	13	0";		
	if (test=="Pid4")         line = "Pid4  200		1000	0		1333	1333	12	0";		
	if (test=="Pid5")         line = "Pid5	100		100		0	    1333	1333	12	0";		
	if (test=="Pid6")         line = "Pid6  100		100		0	    1333	1333	12	0";		
	if (test=="Pid7")         line = "Pid7	200		200		1	    1333	1333	4	0";		

	if (test=="TPid0")        line = "TPid0 -1500	0	0		1333	1333	18	0";		
	if (test=="TPid1")        line = "TPid1 -1500	0	0		1333	1333	18	0";		
	if (test=="TPid2")        line = "TPid2 -1500	0	0		1333	1333	18	0";		
	if (test=="TPid3")        line = "TPid3	-1600	0	0		1333	1333	16	0";		
	if (test=="TPid4")        line = "TPid4 -50		0	0		1333	1333	10	0";		

	if (test=="PositionHome") line = "PositionHome  -30	30	0	45	0	0	40	0";
	if (test=="VelocityHome") line = "VelocityHome  10	10	10	10	10	10	10	10";
	cout << " @@@" << line <<endl;
}
void replace_line_left_leg (string& line, string test)
{
	if (test=="Max ")         line = "Max 115	          90	        78	          15         	39            22"; 
	cout << " @@@" << line <<endl;
}
void replace_line_left_hand (string& line, string test)
{
	if (test=="Min ")         line = "Min    0     0     0     0     0     0     0    0";
	cout << " @@@" << line <<endl;
}
void replace_line_right_arm (string& line, string test)
{
	if (test=="Max ")         line = "Max	  10	160.8	80	106	 90	  0	40	60"; 
	if (test=="Min ")         line = "Min	-110	0	   -37	5.5	-90	-90	-20	0"; 
	if (test=="Currents")     line = "Currents      3000          3000          3000          3000          550           1000          1000          400";  
	if (test=="Pid0")         line = "Pid0  24000	50		40		1333	1333	13	0";		
	if (test=="Pid1")         line = "Pid1 	24000	50		40		1333	1333	13	0";		
	if (test=="Pid2")         line = "Pid2 	 8000	0	 	5		1333	1333	13	0";		
	if (test=="Pid3")         line = "Pid3	24000	20		40		1333	1333	13	0";		
	if (test=="Pid4")         line = "Pid4  200		1000	0		1333	1333	12	0";		
	if (test=="Pid5")         line = "Pid5	100		100		0	    1333	1333	12	0";		
	if (test=="Pid6")         line = "Pid6  100		100		0	    1333	1333	12	0";		
	if (test=="Pid7")         line = "Pid7	200		200		1	    1333	1333	4	0";		

	if (test=="TPid0")        line = "TPid0 1500	0	0		1333	1333	18	0";		
	if (test=="TPid1")        line = "TPid1 1500	0	0		1333	1333	18	0";		
	if (test=="TPid2")        line = "TPid2 1500	0	0		1333	1333	18	0";		
	if (test=="TPid3")        line = "TPid3	1600	0	0		1333	1333	16	0";		
	if (test=="TPid4")        line = "TPid4 50		0	0		1333	1333	10	0";	

	if (test=="PositionHome") line = "PositionHome  -30	30	0	45	0	0	40	0";
	if (test=="VelocityHome") line = "VelocityHome  10	10	10	10	10	10	10	10";
	cout << " @@@" << line <<endl;
}
void replace_line_right_leg (string& line, string test)
{
	if (test=="Max ")         line = "Max 115	          90	        78	          15         	39            22"; 
	cout << " @@@" << line <<endl;
}
void replace_line_right_hand (string& line, string test)
{
	if (test=="Min ")         line = "Min    0     0     0     0     0     0     0    0";
	cout << " @@@" << line <<endl;
}
void replace_line_torso (string& line, string test)
{
	if (test=="Pid6")         line = "Pid6  	18000	6000	 0	1333	1333	13	0";		
	if (test=="Pid7")         line = "Pid7		32000	6000	20	1333	1333	13	0";
	if (test=="Pid8")         line = "Pid8		32000	6000	20	1333	1333	13	0";
	if (test=="Pid9")         line = "Pid9		0		0		0	1333	1333	0	0";
	if (test=="PositionHome") line = "PositionHome    -32	0	0	0	-10	0	0	0  -10	0  ";
	if (test=="VelocityHome") line = "VelocityHome     10	10	10	10	10	10	10	10	10	10 ";
	cout << " @@@" << line <<endl;
}

//*******************************************************************************************
int main ()
{
	void (*pfunz)(string& , string ); 
	string line;
	string test;
	string file_in;
	string file_out;

	for (int i=0; i<7; i++) 
	{
		switch (i)
		{
			case 0: file_in="icub_left_arm.ini";   pfunz = &replace_line_left_arm; break;
			case 1: file_in="icub_left_hand.ini";  pfunz = &replace_line_left_hand;break;
			case 2: file_in="icub_left_leg.ini";   pfunz = &replace_line_left_leg;break;
			case 3: file_in="icub_right_arm.ini";  pfunz = &replace_line_right_arm;break;
			case 4: file_in="icub_right_hand.ini"; pfunz = &replace_line_right_hand;break;
			case 5: file_in="icub_right_leg.ini";  pfunz = &replace_line_right_leg;break;
			case 6: file_in="icub_head_torso.ini"; pfunz = &replace_line_torso;break;
		}

		fstream in(file_in.c_str(),ios::in);
		file_out = ".//crawling//" + file_in.substr(0, file_in.size()-4);
		file_out += "_crawling.ini"; //just change here
		fstream out(file_out.c_str(),ios::out);
		int count = 0;
		while( getline(in,line) )
		{
			cout << count++ << " ***" << line <<endl;

			test = "Max ";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Min ";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Currents";  if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid0";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid1";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid2";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid3";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid4";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid5";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid6";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid7";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid8";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "Pid9";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid0";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid1";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid2";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid3";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid4";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid5";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid6";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid7";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid8";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			test = "TPid9";		if (strncmp (line.c_str(),test.c_str(),test.size())==0)		{pfunz (line,test); }
			
			test = "PositionHome"; if (strncmp (line.c_str(),test.c_str(),test.size())==0)	{pfunz (line,test); }
			test = "VelocityHome"; if (strncmp (line.c_str(),test.c_str(),test.size())==0)  {pfunz (line,test); }
			out << line << endl;
		}
	}

	return 0;
}