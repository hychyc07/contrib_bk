#include <sys/types.h>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <algorithm>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <sys/stat.h>

#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/ImageFile.h>
#include <yarp/os/all.h>



using namespace std;

using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

int getdir (string dir, vector<string> &files, vector<double> & times)
{
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(dir.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << dir << endl;
        return errno;
    }


    struct stat fileInfo;
    string logFile = dir + "/data.log";
    if (!stat(logFile.c_str(),&fileInfo)) {
        printf("found logfile %s\n", logFile.c_str());
	    ifstream* inputFile = new ifstream(logFile.c_str());
        string line;
        double time;
        string fileName;
        int dummy;
    	while ( !inputFile->eof() ) {	
	        getline (*inputFile,line);
    		stringstream ss(line); // Insert the string into a stream
    		ss >> dummy;
    		ss >> time;
		    ss >> fileName;
            files.push_back(fileName);
            times.push_back(time);
		}
    }
    else {
        string fname;

        std::string::size_type idx;
        while ((dirp = readdir(dp)) != NULL) {
            fname = string(dirp->d_name);
            idx = fname.rfind('.');

            if(idx != std::string::npos && fname.substr(idx+1) == "ppm") {
                files.push_back(fname);
            }
        }
        std::sort(files.begin(), files.end());
    }
    closedir(dp);
    return 0;
}

int main(int argc, char* argv[])
{
    Network yarp;
    ResourceFinder rf;

    rf.setVerbose(true);
    rf.configure("ICUB_ROOT",argc,argv);

    string name = rf.check("name", Value("/imageloader"), "name").asString().c_str();
    string dir = rf.check("folder", Value("."), "directory").asString().c_str();
    double fps = rf.check("fps", Value(0.0), "fps").asDouble();

    BufferedPort<ImageOf<PixelRgb> > port;

    port.open((name + "/out").c_str());

    vector<string> files = vector<string>();
    vector<double> times = vector<double>();

    getdir(dir,files, times);
    if (times.size() != files.size()) {
        if (times.size() == 0) {
            if (fps == 0) {
                printf("error: no fps provided and no log file found\n");
                return 0;
            }
            for (int k=0; k<files.size(); k++) 
                times.push_back(k/fps);
        }
    }
            

    double start = Time::now();
    double offset = times[0];
    for (unsigned int i = 0;i < files.size();i++) {
        ImageOf<PixelRgb>& img = port.prepare();
        yarp::sig::file::read(img, (dir + files[i]).c_str());
        Time::delay((times[i]-offset) - (Time::now()-start));
        port.write();
    }

    return 0;
}


