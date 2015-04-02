//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, Anthony Morse																																											//
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

#include "trackerSettings.h"
#include "ui_trackerSettings.h"

/*!
 * \brief Constructor.
 */
TrackerSettings::TrackerSettings() : Settings(), ui(new Ui::TrackerSettings)
{
    ui->setupUi(this);
    setWindowTitle("Settings");
}

/*!
 * \brief Destructor.
 */
TrackerSettings::~TrackerSettings()
{
    delete ui;
}

/*!
 * \brief Sets parameters.
 * \param[in] simulationMode - simulation mode flag
 * \param[in] threshold - movement sensitivity threshold
 */
void TrackerSettings::setParameters(int simulationMode, int threshold)
{
    ui->horizontalSlider_threshold->setValue(threshold);
    ui->checkBox_simulationMode->setChecked(simulationMode);
    QObject::connect(ui->checkBox_simulationMode, SIGNAL(stateChanged(int)), this, SIGNAL(simulationModeChanged(int)));
    QObject::connect(ui->horizontalSlider_threshold, SIGNAL(valueChanged(int)), this, SIGNAL(thresholdChanged(int)));
}
