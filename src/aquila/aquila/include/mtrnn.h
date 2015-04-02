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

#ifndef MTRNN_H
#define MTRNN_H

#include <QWidget>
#include <QStringList>
#include "mainWindow.h"
#include "ui_mainWindow.h"
#include "ui_mtrnn.h"
#include "mtrnnSettings.h"
#include "mtrnnInterface.h"
#include "plot2d.h"

namespace Ui
{
	class MTRNN;
}

/*! MTRNN graphical user interface */
class MTRNN : public GUI
{
    Q_OBJECT

public:
    MTRNN(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString serverName, int instanceID, int tabID=-1);
    ~MTRNN();

private:
    Ui::MTRNN *ui;   
    MTRNNInterface *intrfc;
    MTRNNSettings *settings;

    QSignalMapper *saveSignalMapper;
    QMenu *saveSubMenu;
    QAction *trainAct;
    QAction *saveAct[3];

    int numControlNeurons;
    int numLinguisticNeurons;
    int numVisionNeurons;
    int numActionNeurons;
    float maxError;
    float minError;
    float maxValue;
    float minValue;

	void setupGUI();
    void resetErrorLimits();
    void initPlot();

protected slots:
    void about();
    void updateGUI();

protected:
    void createFileMenu();
    void createSaveSubmenu();

private slots:
    void save(int targetID);
    void save(QVector<float> network);
    void saveErrors(QVector<float> trainingErrors);
    bool loadTrainingData(QString trainingFile);
    void startTraining();
    void updatePlot(int step, float error);
};

#endif//MTRNN_H
