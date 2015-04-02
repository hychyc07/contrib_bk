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

#ifndef SOM_MAP_H
#define SOM_MAP_H

#include "plot3d.h"
#include "mainWindow.h"
#include "ui_mainWindow.h"
#include "ui_som.h"
#include "somSettings.h"
#include "somInterface.h"

namespace Ui
{
    class SOM;
}

/*! SOM graphical user interface */
class SOM : public GUI
{
    Q_OBJECT

public:
    SOM(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString serverName, int instanceID, int tabID=-1);
    ~SOM();

protected:
    void createFileMenu();
    void createSaveSubmenu();

protected slots:
    void about();
    void updateGUI();

private:
    Ui::SOM *ui;
    SOMInterface *intrfc;
    SOMSettings *settings;

    QSignalMapper *saveSignalMapper;
    QMenu *saveSubMenu;
    QAction *saveAct[2];
    QAction *trainAct;
    QAction *viewAct;

    float learningRate;
    float trainingTime;
    float highestDataPoint;
    float lowestDataPoint;
    bool viewingMode;
    bool mapLoaded;

	void setupGUI();
    void showMapProperties(int numOutputs, int vectorDimensions, int subIterationsUsed, double learningRateUsed);

private slots:
    void initRenderModeComboBox();
    void inputMappingChanged();
    void startTraining();
    void viewMap();
    void save(int targetID);
    void saveTrainingTime(float time);
    void saveDataPoints(float low, float hi);
    void initInputMappingComboBoxes(int numInputs);
    void setPlotInputs(QVector<float> map);
};

#endif//SOM_MAP_H
