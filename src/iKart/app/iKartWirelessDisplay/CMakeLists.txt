#
# Copyright (C) 2012 RobotCub Consortium
# Author: Vadim Tikhanoff vadim.tikhanoff@iit.it
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#

set(appname iKartWirelessDisplay)

icub_app(${appname})

file(GLOB conf ${CMAKE_CURRENT_SOURCE_DIR}/conf/*.ini)

file(GLOB pictures ${CMAKE_CURRENT_SOURCE_DIR}/pictures/*.bmp)

icub_app_install(${appname}
        FILES ${conf}
        DESTINATION conf)

icub_app_install(${appname} 
                 FILES ${pictures}
                 DESTINATION pictures)

