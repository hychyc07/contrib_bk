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

#include "somSettings.h"
#include "ui_somSettings.h"

/*!
 * \brief Constructor.
 */
SOMSettings::SOMSettings() : Settings(), ui(new Ui::SOMSettings)
{
    initialise();
    ui->setupUi(this);
    setWindowTitle("Settings");
}

/*!
 * \brief Destructor.
 */
SOMSettings::~SOMSettings()
{
    delete ui;
}

/*!
 * \brief Initialisation.
 */
void SOMSettings::initialise()
{
    gpu.multi = false;
    index = 5;
    for(int i=2; i<1001;i++)
    {
        outputValues[i-2] = (int)pow((float)i,2);
    }
}

/*!
 * \brief Sets the value of outputs to the nearest power of two value.
 * \param[in] value - spin box output value
 */
void SOMSettings::setOutputSizeValue(int value)
{
    if(value>=outputValues[index])
    {
        index++;
    }
    else
    {
        index--;
    }

    QObject::disconnect(ui->spinBox_numOutputs, SIGNAL(valueChanged(int)), this, SLOT(setOutputSizeValue(int)));
    ui->spinBox_numOutputs->setValue(outputValues[index]);
    QObject::connect(ui->spinBox_numOutputs, SIGNAL(valueChanged(int)), this, SLOT(setOutputSizeValue(int)));
}

/*!
 * \brief Sets parameters.
 * \param[in] learningRate - learning rate
 * \param[in] numSubIterations - number of sub-iterations
 * \param[in] numOutputs - number of outputs
 * \param[in] iterationPause - iteration pause
 */
void SOMSettings::setParameters(float learningRate, int numSubIterations, int numOutputs, int iterationPause)
{
    ui->doubleSpinBox_learningRate->setValue((double)learningRate);
    ui->spinBox_subIterations->setValue(numSubIterations);
    ui->spinBox_numOutputs->setValue(numOutputs);
    ui->spinBox_iterationPause->setValue(iterationPause);
}

/*!
 * \brief Saves parameters.
 * \note This is an overridden virtual function.
 * \note This function is normally called when user accepts settings dialog.
 * \note Once called, this function emits signal with updated parameters, which is typically sent to module.
 */
void SOMSettings::save()
{
    emit parametersSaved((float)ui->doubleSpinBox_learningRate->value(),
                         ui->spinBox_subIterations->value(),
                         ui->spinBox_numOutputs->value(),
                         ui->spinBox_iterationPause->value());
}
