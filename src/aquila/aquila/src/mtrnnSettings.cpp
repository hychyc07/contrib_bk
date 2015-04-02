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

#include "mtrnnSettings.h"
#include "ui_mtrnnSettings.h"

/*!
 * \brief Constructor.
 */
MTRNNSettings::MTRNNSettings() : Settings(), ui(new Ui::MTRNNSettings)
{
    ui->setupUi(this);
    setWindowTitle("Settings");
    gpu.multi = true;
}

/*!
 * \brief Destructor.
 */
MTRNNSettings::~MTRNNSettings()
{
    delete ui;
}

/*!
 * \brief Sets parameters.
 * \param[in] learningRate - learning rate
 * \param[in] momentum - momentum
 * \param[in] weightRange - initial range of weights
 * \param[in] threshold - error threshold
 * \param[in] iterations - number of iterations
 * \param[in] seed - random number generator seed
 * \param[in] numFastNeurons - number of fast neurons
 * \param[in] numSlowNeurons - number of slow neurons
 * \param[in] ioDeltaT - input-output delta-t value
 * \param[in] fastDeltaT - fast delta-t value
 * \param[in] slowDeltaT - slow delta-t value
 * \param[in] plotUpdateInterval - update interval of the error plot
 */
void MTRNNSettings::setParameters(float learningRate, float momentum, float weightRange, float threshold, int iterations, int seed, int numFastNeurons, int numSlowNeurons, int ioDeltaT, int fastDeltaT, int slowDeltaT, int plotUpdateInterval)
{
    ui->doubleSpinBox_learningRate->setValue(learningRate);
    ui->doubleSpinBox_momentum->setValue(momentum);
    ui->doubleSpinBox_weightRange->setValue(weightRange);
    ui->doubleSpinBox_threshold->setValue(threshold);
    ui->spinBox_iterations->setValue(iterations);
    ui->spinBox_seed->setValue(seed);
    ui->spinBox_fastNeurons->setValue(numFastNeurons);
    ui->spinBox_slowNeurons->setValue(numSlowNeurons);
    ui->spinBox_ioDeltaT->setValue(ioDeltaT);
    ui->spinBox_fastDeltaT->setValue(fastDeltaT);
    ui->spinBox_slowDeltaT->setValue(slowDeltaT);
    ui->spinBox_plotUpdateInterval->setValue(plotUpdateInterval);
}

/*!
 * \brief Saves parameters.
 * \note This is an overridden virtual function.
 * \note This function is normally called when user accepts settings dialog.
 * \note Once called, this function emits signal with updated parameters, which is typically sent to module.
 */
void MTRNNSettings::save()
{
    emit parametersSaved(ui->doubleSpinBox_learningRate->value(),
                         ui->doubleSpinBox_momentum->value(),
                         ui->doubleSpinBox_weightRange->value(),
                         ui->doubleSpinBox_threshold->value(),
                         ui->spinBox_iterations->value(),
                         ui->spinBox_seed->value(),
                         ui->spinBox_fastNeurons->value(),
                         ui->spinBox_slowNeurons->value(),
                         ui->spinBox_ioDeltaT->value(),
                         ui->spinBox_fastDeltaT->value(),
                         ui->spinBox_slowDeltaT->value(),
                         ui->spinBox_plotUpdateInterval->value());
}
