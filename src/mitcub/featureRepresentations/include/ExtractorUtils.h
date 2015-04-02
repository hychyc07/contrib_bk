



#ifndef __EXTRACTOR_UTILS__
#define __EXTRACTOR_UTILS__

#include <string>
#include <yarp/os/ResourceFinder.h>


void initResourceFinder(yarp::os::ResourceFinder *rf, const std::string &context_path, const std::string &name);

std::string generate_time_tag();


#endif

