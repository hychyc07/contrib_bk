// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Katrin Lohan
  * email: katrin.lohan@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <yarp/os/all.h>
#include </home/bluerabbit/iCubsvn/src/objectAura/src/simulatedTracker/include/iCub/simulatedTracker.h>

using namespace yarp::os;
using namespace std;

//simulatedTracker::simulatedTracker(){}

int main() {

    // Set up YARP
    Network yarp;

    BufferedPort<Bottle> inPort, outPort1, outPort2, outPort3, outPort4;
    inPort.open("/activateTracker/in");
    outPort1.open("/tracker1/out");
    outPort2.open("/tracker2/out");
    outPort3.open("/tracker3/out");
    outPort4.open("/tracker4/out");
    // read the message
    Bottle *in = inPort.read();
    if (in!=NULL) {
      printf("Received %s\n", in->toString().c_str());
    
    int c1=0;
    int i1=0;
    int t1=0;
    int c2=0;
    int i2=0;
    int t2=0;
    int c3=0;
    int i3=0;
    int t3=0;
    int c4=0;
    int i4=0;
    int t4=0;
    int a=0;
    string line1,line2,line3,line4;
    string timestp1, timestp2, timestp3, timestp4 ;
    string x1,x2,x3,x4;
    string y1,y2,y3,y4;    
    double td1, xd1, yd1, td2, xd2, yd2, td3, xd3, yd3, td4, xd4, yd4;
        while(true){
    ifstream myfile1 ("/home/bluerabbit/Desktop/annotation_ball_1");
    ifstream myfile2 ("/home/bluerabbit/Desktop/annotation_ball_2");  
    ifstream myfile3 ("/home/bluerabbit/Desktop/annotation_ball_3");
    ifstream myfile4 ("/home/bluerabbit/Desktop/annotation_hand_breite");  
      if (myfile1.is_open()&&myfile2.is_open()&&myfile3.is_open()&&myfile4.is_open())
      {
        while ( myfile1.good()||myfile2.good()||myfile3.good()||myfile4.good() )
        {
            
        
          getline (myfile1,line1);
          string s1=line1;
          getline (myfile2,line2);
          string s2=line2;
          getline (myfile3,line3);
          string s3=line3;
          getline (myfile4,line4);
          string s4=line4;        


        while(s1.find("\t") !=string::npos && c1 < 3){

            if(c1==0){
            timestp1=s1.substr(0,s1.find("\t"));
            td1=atof(timestp1.c_str());
            t1=s1.find("\t");
            s1.replace(s1.find("\t"),1, ";");
             }
            if(c1==1){
            x1=s1.substr(t1+1,s1.find("\t"));
            xd1=atof(x1.c_str());
            t1=s1.find("\t");
             s1.replace(s1.find("\t"),1, ";");
             }
            if(c1==2){
            y1 =s1.substr(t1+1,s1.find("\t"));  
            yd1=atof(y1.c_str());
            }
             c1=c1+1; 
            }
            c1 =0;
        
            line1=s1;
            
            //cout << td << endl;
            //cout << xd << endl;
            //cout << yd << endl;
           
            
                if(i1==0){
                s1="";}
            i1=i1+1;
            Bottle&out1 = outPort1.prepare();
            out1.clear();
            if(a=0){
                out1.addString("Start");
            }
            out1.addDouble(xd1);
            out1.addDouble(yd1);
            out1.addDouble(td1);           
            outPort1.write(true);  
    

////////////////////////////////////////////////////////////////////////////////////




             while(s2.find("\t") !=string::npos && c2 < 3){

            if(c2==0){
            timestp2=s2.substr(0,s2.find("\t"));
            td2=atof(timestp2.c_str());
            t2=s2.find("\t");
            s2.replace(s2.find("\t"),1, ";");
             }
            if(c2==1){
            x2=s2.substr(t2+1,s2.find("\t"));
            xd2=atof(x2.c_str());
            t2=s2.find("\t");
             s2.replace(s2.find("\t"),1, ";");
             }
            if(c2==2){
            y2 =s2.substr(t2+1,s2.find("\t"));  
            yd2=atof(y2.c_str());
            }
             c2=c2+1; 
            }
            c2 =0;
        
            line2=s2;
            
            //cout << td << endl;
            //cout << xd << endl;
            //cout << yd << endl;
           
            
                if(i2==0){
                s2="";}
            i2=i2+1;
            Bottle&out2 = outPort2.prepare();
            out2.clear();
            if(a=0){
                out2.addString("Start");
            }
            out2.addDouble(xd2);
            out2.addDouble(yd2);
            out2.addDouble(td2);
            outPort2.write(true);  
    

////////////////////////////////////////////////////////////////////////////////////


            while(s3.find("\t") !=string::npos && c3 < 3){

            if(c3==0){
            timestp3=s3.substr(0,s3.find("\t"));
            td3=atof(timestp3.c_str());
            t3=s3.find("\t");
            s3.replace(s3.find("\t"),1, ";");
             }
            if(c3==1){
            x3=s3.substr(t3+1,s3.find("\t"));
            xd3=atof(x3.c_str());
            t3=s3.find("\t");
             s3.replace(s3.find("\t"),1, ";");
             }
            if(c3==2){
            y3 =s3.substr(t3+1,s3.find("\t"));  
            yd3=atof(y3.c_str());
            }
             c3=c3+1; 
            }
            c3 =0;
        
            line3=s3;
            
            //cout << td << endl;
            //cout << xd << endl;
            //cout << yd << endl;
           
            
                if(i3==0){
                s3="";}
            i3=i3+1;
            Bottle&out3 = outPort3.prepare();
            out3.clear();
            if(a=0){
                out3.addString("Start");
            }
            out3.addDouble(xd3);
            out3.addDouble(yd3);
            out3.addDouble(td3);
            outPort3.write(true);








/////////////////////////////////////////////////////////////////////////////////////////////////////////////////





            while(s4.find("\t") !=string::npos && c4 < 3){

            if(c4==0){
            timestp4=s4.substr(0,s4.find("\t"));
            td4=atof(timestp4.c_str());
            t4=s4.find("\t");
            s4.replace(s4.find("\t"),1, ";");
             }
            if(c4==1){
            x4=s4.substr(t4+1,s4.find("\t"));
            xd4=atof(x4.c_str());
            t4=s4.find("\t");
             s4.replace(s4.find("\t"),1, ";");
             }
            if(c4==2){
            y4 =s4.substr(t4+1,s4.find("\t"));  
            yd4=atof(y4.c_str());
            }
             c4=c4+1; 
            }
            c4 =0;
        
            line4=s4;
            
            //cout << td << endl;
            //cout << xd << endl;
            //cout << yd << endl;
           
            
                if(i4==0){
                s4="";}
            i4=i4+1;
            Bottle&out4 = outPort4.prepare();
            out4.clear();
            if(a=0){
                out4.addString("Start");
            }
            out4.addDouble(xd4);
            out4.addDouble(yd4);
            out4.addDouble(td4);
            outPort4.write(true);



                                    

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

            a=a+1;

            }
            
        myfile1.close();
        myfile2.close();
        myfile3.close();
        myfile4.close();
        }
    }
    }
    else cout << "Unable to open file"; 

    return 0;
}
