// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
  
/**
 * @file main.cpp
 * @brief main code for the tutorial module.
 */


#include <string>
#include <iostream>
#include <fstream> 
#include <cstring>
#include <cstdlib>

#include <yarp/os/all.h>

using namespace yarp::os;
using namespace std;

int main(int argc, char * argv[])
{
    
    Network yarp;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("trajectVisualize.ini");    //overridden by --from parameter
    rf.setDefaultContext("morphoGenApp/conf");    //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);  
    yarp::os::ConstString path = rf.findPath("trajPoints.txt");
    printf("File found! path %s \n", path.c_str());

    
    yarp::os::BufferedPort<yarp::os::Bottle> output;
    output.open("/trajectVisualize/cmd:o");
    Network::connect("/trajectVisualize/cmd:o","/iCubGui/objects");
    


    string line;
    char *ptrLine;
    string word;
    

    ifstream trajFile (path.c_str());
    if (trajFile.is_open()) {
        while ( trajFile.good() ) {
            getline (trajFile,line);
            cout << "line: "<<line << endl;
            if(line.empty()) {
                printf("empty line \n");
                break;
            }
            char text[line.length()];
            char * pch;
            double value[3];

            int i = 0;
            pch = strtok ((char*)line.c_str()," ");            
            printf("%s \n", pch);
            value[i] = atof(pch);            
            
            i++;
            
            
            while (pch != NULL) {               
                pch = strtok (NULL, " ");                
                
                //value[i] = stod(pch);
                if(pch !=NULL) {
                    printf("%s \n", pch);
                    value[i] = atof(pch);                
                }
                i++;
            }

            printf("Preparing the port \n");
            yarp::os::Bottle& b = output.prepare();
            b.clear();
            
            //trajPoint.addString("addpoint");
            //trajPoint.addString("trajA");
            //trajPoint.addDouble(x);
            //trajPoint.addDouble(y);
            //trajPoint.addDouble(z);
            
            b.addString("addpoint");
            b.addString("trajA");
            b.addDouble(value[0] * 1000);
            b.addDouble(value[1] * 1000);
            b.addDouble((value[2] + 0.35) * 1000 );
            
            //trajPoint = b.addList();
            output.write();

            Time::delay(0.1);

            
        }
        trajFile.close();
    }
    else cout << "Unable to open file"; 
    

    
 
    //module.runModule(rf);
    return 0;
}


