//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, <Martin Peniak - www.martinpeniak.com>																																					//
//All rights reserved.																																															//
//																																																				//
//Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:																//
//																																																				//
// - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.																				//
// - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.	//
//																																																				//
//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR	//
//A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT	//
//LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR	//
//TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.																//
//                                                                                                                                                                                                              //
//The views and conclusions contained in the software and documentation are those of the authors and should not be interpreted                                                                                  //
//as representing official policies,either expressed or implied, of the FreeBSD Project.                                                                                                                        //
//##############################################################################################################################################################################################################//

#include "settings.h"

/*!
 * \brief Constructor.
 */
Settings::Settings() : QDialog()
{
    //initialise GPU defaults
    gpu.count = 0;
    gpu.multi = false;
    gpu.active = false;
    gpu.initialised = false;

    for(int i=0; i<MAX_GPU_DEVICES; i++)
    {
        if(i==0)
        {
            gpu.selected[i] = 0; //default device
        }
        else
        {
            gpu.selected[i] = -1; //other devices are unselected by default
        }
    }

    QShortcut *closeDialog = new QShortcut(this);
    closeDialog->setKey(QKeySequence("CTRL+Q"));
    closeDialog->setContext(Qt::WindowShortcut);
    QObject::connect(closeDialog, SIGNAL(activated()), this, SLOT(close()));
}

/*!
 * \brief Destructor.
 */
Settings::~Settings()
{
}

/*!
 * \brief Saves parameters.
 * \note This is a virtual function that can be overridden.
 */
void Settings::save()
{
}
