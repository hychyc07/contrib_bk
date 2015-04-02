/*
 * main.cpp
 *
 *  Created on: Jun 4, 2012
 *      Author: Christian Dondrup
 */

/**
 * @mainpage SpeechSegmenter
 *
 * This module segments a audio file (.wav) of speech into words or the shortest possible
 * sentences in the case of continuous speech. It is used to generate keywords which are then
 * checked against the fingerprint database.
 *
 * @section Installation
 * @subsection Dependencies
 * -# fftw3
 *  - Install this via the ubuntu package management:@n
 *    @c sudo @c apt-get @c install @c libfftw3-dev
 *  - Or for windows go to this page:@n
 *    http://www.fftw.org/install/windows.html
 * -# libsndfile
 *  - Install this via the ubuntu package management:@n
 *    @c sudo @c apt-get @c install @c libsndfile-dev
 *  - Or for windows got to this page:@n
 *    http://www.mega-nerd.com/libsndfile/#Download
 * @subsection Building
 * -# Run cmake
 *
 * @section Usage
 * @subsection nparameters Necessary parameters
 * -# @e --audio A audio file (.wav) containing speech or a text file with a list of audio
 * files separated by a newline.
 * -# @e --workdir The directory where the filtered original and the segments should be stored.
 * The filtered original will have the same name as the original file with @c _filtered at the end
 * and the segment files will have Seg-[0-9] in there name. Files with the same name will be
 * overwritten.
 * @subsection hparameters Helpful parameters
 * These parameters are necessary to run the program but can influence the result a lot:
 * -# Filters:@n
 * The following parameters are used to construct the band pass filter which is supposed
 * to filter unwanted noise from the signal. Every frequency between high pass and low
 * pass cutoff will be passed through the filter. So you have to make sure that the low
 * pass cutoff frequency is higher then the high pass frequency. Otherwise you will have
 * no signal left.
 *  - @e --lowpass This is the cutoff frequency for the low pass filter which is used to
 *  create the band pass filter. Every frequency above this threshold will be eliminated
 *  - @e --highpass This is the cutoff frequency for the high pass filter which is used to
 *  create the band pass filter. Every frequency below this threshold will be eliminated
 * -# Segmentation parameters:@n
 * These parameters are used to improve the segmentation.
 *  - @e --lambda This is the minimal size of one speech segment. This is messured in speech
 *  windows of about 25ms with 25% overlap so lambda = 1 sets the minimal size of one speech segment to 25ms.
 *  This is meant to filter out short sound bursts.
 *  - @e --delta This is the maximal inter-segment distance between two speech segments. Every
 *  segment that has a neighbouring segment with a distance of less then delta will be grouped
 *  with this segment to one speech segment. Like lambda delta is messured in wondows of 25ms
 *  of sound with 25% overlap. So delta = 3 means a maximal distance of 62.5ms
 *  - @e --verbose prints some helpful output to the stdio in order to choose the right
 *  parameters.
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




