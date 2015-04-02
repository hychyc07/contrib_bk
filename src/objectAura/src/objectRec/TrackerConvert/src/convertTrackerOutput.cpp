//Copyright: (C) 2012 RobotCub Consortium Authors: Katrin Solveig Lohan
#include <yarp/os/all.h>
#include <stdio.h>
#include <iostream>
#include <string.h>
// h1= star h2= moon h3 =arrow
// r1= circle r2=cross r3=hart
// startx, starty : first pos for ordering 
// 


using namespace yarp::os;
using namespace std;

int main(int argc, char *argv[]) {
  Network yarp;

  BufferedPort<Bottle> inPort;
  bool tracker = inPort.open("/tracker/in");
  BufferedPort<Bottle> outPort;
  bool out = outPort.open("/pos/out");
  tracker = tracker; 
  //pos =pos;
  if (!tracker || !out) {
    fprintf(stderr, "Failed to create ports.\n");
    fprintf(stderr, "Maybe you need to start a nameserver (run 'yarpserver' \n");
    return 1;
  }


  while(true) {
	
    // read the message
	cout << "waiting for input" << endl;
    Bottle *in = inPort.read();
	Bottle &out = outPort.prepare();
	 if (in!=NULL) {
            cout << "got " << in->toString().c_str() << endl;
			string tmp;
			double startx=0.0;
			double starty=0.0;
			double object0range, object0imx, object0imy, object1range, object1imx, object1imy, object2range, object2imx, object2imy, object3range, object3imx, object3imy, object4range, object4imx, object4imy, object5range, object5imx, object5imy;
			bool object0 =false;
			bool object1 =false;
			bool object2 =false; 
			bool object3 =false; 
			bool object4 =false;
			bool object5 =false;
			bool human1 = false;
			bool human2 = false;
			bool human3 = false;
			bool robot1 = false;
			bool robot2 = false;
			bool robot3 = false;
			int pos1, pos2, pos3;

			
            for (int i=0; i<in->size(); i=i+27) {
                tmp = in->get(i).asString();
				out.clear();
				 if(tmp == "circle"){
					object0=true;
					object0range = in->get(i+20).asDouble();
					object0imx = in->get(i+22).asDouble();
					object0imy = in->get(i+23).asDouble();
					robot1=true;
					cout << "range " << object0range << endl;
					cout << "imx " << object0imx << endl;
					cout << "imy " << object0imy << endl;
					} 
				if(tmp=="star"){
					object1=true;
					human1=true;
					object1range = in->get(i+20).asDouble();
					object1imx = in->get(i+22).asDouble();
					object1imy = in->get(i+23).asDouble();
					cout << "range " << object1range << endl;
					cout << "imx " << object1imx << endl;
					cout << "imy " << object1imy << endl;
					} 
				if(tmp=="moon"){
					object2=true;
					human2=true;
					object2range = in->get(i+20).asDouble();
					object2imx = in->get(i+22).asDouble();
					object2imy = in->get(i+23).asDouble();
					cout << "range " << object2range << endl;
					cout << "imx " << object2imx << endl;
					cout << "imy " << object2imy << endl;
					} 
				if(tmp=="cross"){
					object3 =true;
					robot3=true;
					object3range = in->get(i+20).asDouble();
					object3imx = in->get(i+22).asDouble();
					object3imy = in->get(i+23).asDouble();
					cout << "range " << object3range << endl;
					cout << "imx " << object3imx << endl;
					cout << "imy " << object3imy << endl;
					} 
				if(tmp=="hart"){
					object4 =true;
					robot2=true;
					object4range = in->get(i+20).asDouble();
					object4imx = in->get(i+22).asDouble();
					object4imy = in->get(i+23).asDouble();
					cout << "range " << object4range << endl;
					cout << "imx " << object4imx << endl;
					cout << "imy " << object4imy << endl;					
					} 
				 if(tmp=="arrow"){
					object5=true;
					human3=true;
					object5range = in->get(i+20).asDouble();
					object5imx = in->get(i+22).asDouble();
					object5imy = in->get(i+23).asDouble();
					cout << "range " << object5range << endl;
					cout << "imx " << object5imx << endl;
					cout << "imy " << object5imy << endl;
					} 
				if(human1 & human2 & human3  ){
						double distance1x= startx-object1imx;
						double distance1y= starty-object1imy;
						double distance2x= startx-object2imx;
						double distance2y= starty-object2imy;
						double distance3x= startx-object5imx;
						double distance3y= starty-object5imy;
						if (distance1x< distance2x & distance1x< distance3x & distance1y < distance2y & distance1y < distance3y){
							pos1=1; //star on one
						}
						if (distance2x< distance1x & distance2x< distance3x & distance2y < distance1y & distance2y < distance3y){
							pos1=2; //moon on one
						}
						if (distance3x< distance2x & distance3x< distance1x & distance3y < distance2y & distance3y < distance1y){
							pos1=3; //arrow on one
						}	
						if (pos1 == 1 & distance2x< distance3x & distance2y < distance3y){
							pos2 = 2; 
							pos3 = 3; //star moon arrow
							out.addDouble(pos1);
							out.addDouble(object0imx);
							out.addDouble(object0imy);
							out.addDouble(object0range);
							out.addDouble(pos2);
							out.addDouble(object3range);
							out.addDouble(object3range);
							out.addDouble(object3range);
							out.addDouble(pos3);
							out.addDouble(object4range);
							out.addDouble(object4range);
							out.addDouble(object4range);
						}		
						if (pos1 == 1 & distance3x< distance2x & distance3y < distance2y){
							pos2 = 3;
							pos3 = 2; //star arrow moon
							out.addDouble(pos1);
							out.addDouble(object0imx);
							out.addDouble(object0imy);
							out.addDouble(object0range);
							out.addDouble(pos2);
							out.addDouble(object4range);
							out.addDouble(object4range);
							out.addDouble(object4range);
							out.addDouble(pos3);
							out.addDouble(object3range);
							out.addDouble(object3range);
							out.addDouble(object3range);
						}	
						if (pos1 == 2 & distance2x< distance3x & distance2y < distance3y){
							pos2 = 1; 
							pos3 = 3; //moon star arrow
							out.addDouble(pos1);
							out.addDouble(object3imx);
							out.addDouble(object3imy);
							out.addDouble(object3range);
							out.addDouble(pos2);
							out.addDouble(object0range);
							out.addDouble(object0range);
							out.addDouble(object0range);
							out.addDouble(pos3);
							out.addDouble(object4range);
							out.addDouble(object4range);
							out.addDouble(object4range);
						}		
						if (pos1 == 2 & distance3x< distance2x & distance3y < distance2y){
							pos2 = 3;
							pos3 = 1; //moon arrow star
							out.addDouble(pos1);
							out.addDouble(object3imx);
							out.addDouble(object3imy);
							out.addDouble(object3range);
							out.addDouble(pos2);
							out.addDouble(object4range);
							out.addDouble(object4range);
							out.addDouble(object4range);
							out.addDouble(pos3);
							out.addDouble(object0range);
							out.addDouble(object0range);
							out.addDouble(object0range);
						}	
						if (pos1 == 3 & distance2x< distance3x & distance2y < distance3y){
							pos2 = 1;
							pos3 = 2; //arrow moon star
							out.addDouble(pos1);
							out.addDouble(object4imx);
							out.addDouble(object4imy);
							out.addDouble(object4range);
							out.addDouble(pos2);
							out.addDouble(object0range);
							out.addDouble(object0range);
							out.addDouble(object0range);
							out.addDouble(pos3);
							out.addDouble(object3range);
							out.addDouble(object3range);
							out.addDouble(object3range);
						}		
						if (pos1 == 3 & distance3x< distance2x & distance3y < distance2y){
							pos2 = 2;
							pos3 = 1; //arrow star moon
							out.addDouble(pos1);
							out.addDouble(object4imx);
							out.addDouble(object4imy);
							out.addDouble(object4range);
							out.addDouble(pos2);
							out.addDouble(object3range);
							out.addDouble(object3range);
							out.addDouble(object3range);
							out.addDouble(pos3);
							out.addDouble(object0range);
							out.addDouble(object0range);
							out.addDouble(object0range);
						}	
					cout << "All Human object are detected!" << pos1 << pos2 << pos3 << endl;			
				}
				else{
					cout << "Not all Human object are detected!" << endl;
				}
							cout << "total " << tmp << endl;
							cout << "object " << object0 << object1 << object2 << object3<< object4 << object5 << endl;
							
				}
			outPort.write();
        }

	
    }
    }
 

