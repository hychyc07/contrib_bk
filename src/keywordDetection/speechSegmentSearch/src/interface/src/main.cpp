/*
 * main.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

/**
 * @mainpage SpeechSegmentSearch
 *
 * This module checks single words (produced by speechSegmenter) against the fingerprint
 * database (produced by the speechFingerprinter).
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
 *  - @e --workdir Specify a directory where the descriptor and database files create by
 *  the speechFingerprinter module are stored.
 *  - @e --filter Path to the filter file. This can be found under: @n
 *   @c /musicretr-1.0_install_dir/etc/musicretr-1.0/boostdescr3.txt
 *  - @e --em The expectation maximization parameters file. Can be found in:@n
 *   @c /musicretr-1.0_install_dir/etc/musicretr-1.0/emparameters.bin
 *
 * As an output you will have the name of the matching files and a score both on the stdio
 * and the output port. The lower the score the better the match. Results will be ordered
 * concerning their scores.
 *
 * @author Christian Dondrup
 */

#include <yarp/os/all.h>
#include <stdio.h>
#include <iostream>
#include "iCub/SSInterface.h"


using namespace yarp::os;
using namespace std;


int main(int argc, char * argv[]){

	Network yarp;

	SSInterface module;

	ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("");      //overridden by --from parameter
    rf.setDefaultContext("");           //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);

	module.runModule(rf);
	return 0;
}




