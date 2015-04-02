/*
 * main.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

/**
 * @mainpage SpeechFingerprinter
 *
 * This module creates fingerprints for sound files. This should be used to identify
 * keywords in sentences.
 *
 * @section Installation
 * @subsection Dependencies
 * -# fftw3
 *  - Install this via the ubuntu package management:@n
 *    @c sudo @c apt-get @c install @c libfftw3-dev
 *  - Or for windows go to this page:@n
 *    http://www.fftw.org/install/windows.html
 * -# musicretr-1.0
 *  - Go to the @c ext directory of this module and run the install.sh script @n
 *   @c ./install.sh @c \<install_prefix\> @n
 *   This will download the sources, patch, build and install them to the
 *   specified install_prefix.@n
 *  - You can also run the cmake scripts yourself if you want:
 *    - Create a build directory and run from that:
 *    @n@c cmake ..
 *    @n@c make
 *    - Go to @c create_lib directory inside the build directory and run:
 *    @n@c cmake .
 *    @n@c make
 *    @n@c make install
 * -# Make sure that both can be found by pkg-config
 * @subsection Building
 * -# Run cmake
 *
 * @section Usage
 * -# The following parameters must be given in order for the module to run:
 *  - @e --audio Specify an audio file (.wav) or a text file with a list of audio files
 *  separated by a newline.
 *  - @e --workdir Specify a directory where the descriptor and database files should be
 *  stored. The descriptor files will be named after the input audio files. If there already
 *  exist files with that name the new file will have the current system time in its name.
 *  If @e --overwrite is given, the old files will be overwritten.
 *  - @e --filter Path to the filter file. This can be found under: @n
 *   @c /musicretr-1.0_install_dir/etc/musicretr-1.0/boostdescr3.txt
 *
 * @author Christian Dondrup
 *
 */

#include <yarp/os/all.h>
#include <stdio.h>
#include <iostream>
#include "iCub/SFInterface.h"


using namespace yarp::os;
using namespace std;


int main(int argc, char * argv[]){

	Network yarp;

	SFInterface module;

	ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("");      //overridden by --from parameter
    rf.setDefaultContext("");           //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

	module.runModule(rf);
	return 0;
}




