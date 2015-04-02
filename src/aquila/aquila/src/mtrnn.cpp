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

#include <QtUiTools>
#include <QFileDialog>
#include <QShortcut>
#include "mtrnn.h"
#include "ui_mtrnn.h"
#include "ui_mtrnnSettings.h"

#define MAX_SEQUENCES 100

/*!
 * \brief Constructor.
 * \param[in] pMainWindow - pointer to MainWindow
 * \param[in] binaryName - name of module executable
 * \param[in] moduleTitle - title of this module
 * \param[in] server - server where the module runs
 * \param[in] instanceID - module identification number
 * \param[in] tabID - module tab index
 */
MTRNN::MTRNN(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString serverName, int instanceID, int tabID) : GUI(pMainWindow, binaryName, moduleTitle, serverName, instanceID, tabID), ui(new Ui::MTRNN)
{
	if(moduleConnected)
	{
		setupGUI();
	}
}

/*!
 * \brief Destructor.
 */
MTRNN::~MTRNN()
{
    delete ui;
}

/*!
 * \brief Sets up graphical user interface.
 */
void MTRNN::setupGUI()
{
	//set up user interface
    ui->setupUi(this);

    //initialise interface and settings
    intrfc = new MTRNNInterface(this);
    settings = new MTRNNSettings();

    //point base pointers to child pointers
    interfaceBase = intrfc;
    settingsBase = settings;

    //start listening for incomming messages
    intrfc->start();

    //connect signals/slots
    qRegisterMetaType< QVector<float> >("QVector<float>");
    qRegisterMetaType< QVector<float> >("QVector<QStringList>");
    QObject::connect(this, SIGNAL(abortRequested()), intrfc, SLOT(sendAbortRequest()));
	QObject::connect(this, SIGNAL(tabSelected()), this, SLOT(show()));
    QObject::connect(intrfc, SIGNAL(progressReceived(int)), this, SLOT(updateProgressBar(int)));
    QObject::connect(intrfc, SIGNAL(errorReceived(int, float)), this, SLOT(updatePlot(int,float)));
    QObject::connect(intrfc, SIGNAL(networkReceived(QVector<float>)), this, SLOT(save(QVector<float>)));
    QObject::connect(intrfc, SIGNAL(errorsReceived(QVector<float>)), this, SLOT(saveErrors(QVector<float>)));
    QObject::connect(intrfc, SIGNAL(gpuListReceived(QVector<QStringList>)), this, SLOT(setGpuList(QVector<QStringList>)));
    QObject::connect(intrfc, SIGNAL(statusReceived(int)), this, SLOT(statusChanged(int)));
    QObject::connect(intrfc, SIGNAL(parametersReceived(float,float,float,float,int,int,int,int,int,int,int,int)), settings, SLOT(setParameters(float,float,float,float,int,int,int,int,int,int,int,int)));
    QObject::connect(settings->ui->pushButton_ok, SIGNAL(clicked()), settings, SLOT(accept()));
    QObject::connect(settings, SIGNAL(accepted()), settings, SLOT(save()));
    QObject::connect(settings->ui->pushButton_cancel, SIGNAL(clicked()), settings, SLOT(reject()));
    QObject::connect(settings, SIGNAL(parametersSaved(float,float,float,float,int,int,int,int,int,int,int,int)), intrfc, SLOT(sendParameters(float,float,float,float,int,int,int,int,int,int,int,int)));

	//set flags
	useModuleMenu = false;

    //get the parameters from the module
    intrfc->sendParametersRequest();

    //initialise graphs
    initPlot();

    //check if the modeule's tab is currently selected
    checkFocus(mainWindow->tabWidget->currentIndex());
}

/*!
 * \brief Updates graphical user interface.
 * \note This is an overridden virtual function.
 */
void MTRNN::updateGUI()
{
    if(status==0) //initialised
    {
        abortAct->setVisible(false);
        saveSubMenu->setEnabled(false);
    }
    else if(status==1) //training in progress
    {
		running = true;
        
		trainAct->setVisible(false);
        saveSubMenu->setEnabled(false);
        settingsAct->setEnabled(false);
        abortAct->setVisible(true);
        deviceSubMenu->setEnabled(false);

        mainWindow->statusBar()->showMessage("Training started",2000);
		qDebug(" - %s_gui(%i): trainig started", binary.toStdString().c_str(), instance);
    }
    else if(status==2) //training finished
    {
		running = false;
        
		trainAct->setVisible(true);
        settingsAct->setEnabled(true);
        abortAct->setVisible(false);
        deviceSubMenu->setEnabled(true);
        saveSubMenu->setEnabled(true);
        saveAct[0]->setEnabled(true);
        saveAct[2]->setEnabled(true);

        mainWindow->statusBar()->showMessage("Training finished",2000);
		qDebug(" - %s_gui(%i): trainig finished", binary.toStdString().c_str(), instance);
    }
}

/*!
 * \brief Creates file menu.
 * \note This is an overridden virtual function.
 */
void MTRNN::createFileMenu()
{
    abortAct = new QAction("&Abort", this);
    abortAct->setShortcut(QKeySequence("Ctrl+A"));
    abortAct->setStatusTip("Abort training process");
    QObject::connect(abortAct, SIGNAL(triggered()), this, SIGNAL(abortRequested()));

    trainAct = new QAction("&New", this);
    trainAct->setShortcut(QKeySequence("Ctrl+N"));
    trainAct->setStatusTip("Train");
    QObject::connect(trainAct, SIGNAL(triggered()), this, SLOT(startTraining()));

    quitAct = new QAction("&Quit", this);
    quitAct->setShortcut(QKeySequence("Ctrl+Q"));
    quitAct->setStatusTip("Quit Aquila");
    QObject::connect(quitAct, SIGNAL(triggered()), mainWindow, SLOT(quit()));

    fileMenu = mainWindow->menuBar()->addMenu("&File");
    fileMenu->addAction(trainAct);
    fileMenu->addAction(abortAct);
    createSaveSubmenu();
    fileMenu->addSeparator();
    fileMenu->addAction(quitAct);
}

/*!
 * \brief Creates save menu.
 */
void MTRNN::createSaveSubmenu()
{
    //create a new submenu and signal mapper
    saveSubMenu = fileMenu->addMenu("&Save");
    saveSignalMapper = new QSignalMapper(saveSubMenu);
    saveSubMenu->clear();

    //save neural networks action
    saveAct[0] = new QAction("Network", this);
    saveAct[0]->setStatusTip("Save neural network");
    saveSignalMapper->setMapping(saveAct[0], 0);
    QObject::connect(saveAct[0], SIGNAL(triggered()), saveSignalMapper, SLOT(map()));

    //save plot picture action
    saveAct[1] = new QAction("Plot", this);
    saveAct[1]->setStatusTip("Save plot");
    saveSignalMapper->setMapping(saveAct[1], 1);
    QObject::connect(saveAct[1], SIGNAL(triggered()), saveSignalMapper, SLOT(map()));

    //save errors action
    saveAct[2] = new QAction("Errors", this);
    saveAct[2]->setStatusTip("Save errors");
    saveSignalMapper->setMapping(saveAct[2], 2);
    QObject::connect(saveAct[2], SIGNAL(triggered()), saveSignalMapper, SLOT(map()));

    //add actions and connect signal mapper
    QObject::connect(saveSignalMapper, SIGNAL(mapped(int)), this, SLOT(save(int)));
    saveSubMenu->addAction(saveAct[0]);
    saveSubMenu->addAction(saveAct[1]);
    saveSubMenu->addAction(saveAct[2]);
}

/*!
 * \brief Shows message box with the information about module.
 * \note This is an overridden virtual function.
 */
void MTRNN::about()
{
    QMessageBox::about(this, "Multiple Time-scales Recurent Neural Network", "This module implements multiple time-scales recurent neural network and back-propagation through time training.");
}

/*!
 * \brief Initialises plot.
 */
void MTRNN::initPlot()
{
    resetErrorLimits();

    ui->plot->setInteractions(QCustomPlot::iRangeDrag | QCustomPlot::iRangeZoom);
    ui->plot->setRangeDrag(Qt::Horizontal|Qt::Vertical);
    ui->plot->setRangeZoom(Qt::Horizontal|Qt::Vertical);
    ui->plot->setupFullAxesBox();
    
	ui->plot->xAxis->setLabel("iterations");
    ui->plot->xAxis->setPadding(20);

	ui->plot->yAxis->setLabel("error");
    ui->plot->yAxis->setPadding(15);
    ui->plot->yAxis->setNumberPrecision(6);
    
	ui->plot->addGraph();
    ui->plot->graph()->setLineStyle(QCPGraph::lsLine);
    ui->plot->graph()->setPen(QPen(Qt::black));
    ui->plot->graph()->clearData();
}

/*!
 * \brief Updates plot with new error.
 * \param[in] step - current step
 * \param[in] error - error at current step
 */
void MTRNN::updatePlot(int step, float error)
{
    ui->plot->graph()->addData(step, error);

    if(error>maxError && step>10)
    {
        maxError = error;
    }

    if(error<minError)
    {
        minError = error;
    }

    ui->plot->xAxis->setRange(0, step);
    ui->plot->yAxis->setRange(minError, maxError);
    ui->plot->replot();
}

/*!
 * \brief Resets error limits.
 */
void MTRNN::resetErrorLimits()
{
    maxError = 0.0;
    minError = 999999.0;
}

/*!
 * \brief Saves neural network, plot image or data.
 * \param[in] targetID - save target
 * \note This function is called when user clicked on any of the save elements in the save sub-menu.
 */
void MTRNN::save(int targetID)
{
    QString fileName;
    QString path;

    switch(targetID)
    {
    case 0:
        qDebug(" - %s_gui(%i): saving neural network", binary.toStdString().c_str(), instance);

        if(!remoteMode) //save the network directly
        {
            intrfc->sendSaveRequest(QFileDialog::getSaveFileName(mainWindow, "Choose a file to save the neural network", QDir::currentPath()+"/untitled.txt", "Text Files (*.txt)"));
        }
        else //first ask for a remote host for the network weights and then save them
        {
            intrfc->sendNetworkRequest();
        }
        break;

    case 1:
        fileName = QFileDialog::getSaveFileName(this, "Save Plot", QDir::currentPath()+"/untitled.png","Images (*.png *.bmp *.jpg *.jpg);;PDF(*.pdf)");

        if(fileName.endsWith(".pdf"))
        {
            ui->plot->savePdf(fileName);
        }
        else if(fileName.endsWith(".png"))
        {
            ui->plot->savePng(fileName);
        }
        else if(fileName.endsWith(".bmp"))
        {
            ui->plot->saveBmp(fileName);
        }
        else if(fileName.endsWith(".jpg"))
        {
            ui->plot->saveJpg(fileName);
        }
        break;

    case 2:
        path = QFileDialog::getSaveFileName(mainWindow, "Choose a file to save the errors", QDir::currentPath()+"/untitled.txt", "Text Files (*.txt)");
        QFile saveFile(path);
        QTextStream out(&saveFile);
        char buf[50];

        if(!saveFile.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            qCritical(" - %s_gui(%i): failed to open %s", binary.toStdString().c_str(), instance, saveFile.fileName().toStdString().c_str());
        }
        else
        {
            //write the errors
            for(int i=0; i<ui->plot->graph()->data()->size(); i++)
            {
                sprintf(buf,"%.6f\n", (float)ui->plot->graph()->data()->value(i).value);
                out<<buf;
            }
            saveFile.close();
            qDebug(" - %s_gui(%i): errors were saved to: %s", binary.toStdString().c_str(), instance, path.toStdString().c_str());
        }
        break;
    }
}

/*!
 * \brief Saves neural network that was received from module runnning on a remote server.
 * \param[in] network - neural network
 * \note This function is called when a neural network is received from module.
 */
void MTRNN::save(QVector<float> network)
{
    QString path = QFileDialog::getSaveFileName(mainWindow, "Choose a file to save the neural network", QDir::currentPath()+"/untitled.txt", "Text Files (*.txt)");
    QFile saveFile(path);
    QTextStream out(&saveFile);
    char buf[50];

    if(!saveFile.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qCritical(" - %s_gui(%i): failed to open %s", binary.toStdString().c_str(), instance, saveFile.fileName().toStdString().c_str());
    }
    else
    {        
        out<<"[SETTINGS]\n";
        if(settings->gpu.active)
        {
            out<<"execution mode\t\tGPU\n";
        }
        else
        {
            out<<"execution mode\t\tCPU \n";
        }

        out<<"\n[ARCHITECTURE]\n";
        out<<"control neurons\t\t"<<numControlNeurons<<"\n";
        out<<"linguistic neurons\t"<<numLinguisticNeurons<<"\n";
        out<<"vision neurons\t\t"<<numVisionNeurons<<"\n";
        out<<"action neurons\t\t"<<numActionNeurons<<"\n";
        out<<"fast neurons\t\t"<<settings->ui->spinBox_fastNeurons->value()<<"\n";
        out<<"slow neurons\t\t"<<settings->ui->spinBox_slowNeurons->value()<<"\n";

        out<<"\n[TRAINING]\n";
        out<<"final error\t\t"<<(float)ui->plot->graph()->data()->value(ui->plot->graph()->data()->size()-1).value<<"\n";
        out<<"learning rate\t\t"<<settings->ui->doubleSpinBox_learningRate->value()<<"\n";
        out<<"momentum\t\t"<<settings->ui->doubleSpinBox_momentum->value()<<"\n";
        out<<"weight range\t\t"<<settings->ui->doubleSpinBox_weightRange->value()<<"\n";
        out<<"threshold\t\t"<<settings->ui->doubleSpinBox_threshold->value()<<"\n";
        out<<"iterations\t\t"<<settings->ui->spinBox_iterations->value()<<"\n";
        out<<"io delta-t\t\t"<<settings->ui->spinBox_ioDeltaT->value()<<"\n";
        out<<"fast delta-t\t\t"<<settings->ui->spinBox_fastDeltaT->value()<<"\n";
        out<<"slow delta-t\t\t"<<settings->ui->spinBox_slowDeltaT->value()<<"\n";
        out<<"original min\t\t"<<minValue<<"\n";
        out<<"original max\t\t"<<maxValue<<"\n";
        out<<"seed\t\t\t"<<settings->ui->spinBox_seed->value()<<"\n";

        //total number of neurons
        int numNeurons = numControlNeurons +
                         numLinguisticNeurons +
                         numVisionNeurons +
                         numActionNeurons +
                         settings->ui->spinBox_fastNeurons->value() +
                         settings->ui->spinBox_slowNeurons->value();

        out<<"\n[WEIGHTS]\n";
        for(int i=0; i<numNeurons; i++) //connections from
        {
            for(int j=0; j<numNeurons+1; j++) //connections to, +1 is for bias weights stored in the last column
            {
                sprintf(buf,"%.6f", network.at((j*numNeurons)+i));
                out<<buf;
                if(j<numNeurons)
                {
                    out<<",";
                }
            }
            out<<"\n";
        }

        saveFile.close(); 
        qDebug(" - %s_gui(%i): neural network was saved to %s", binary.toStdString().c_str(), instance, path.toStdString().c_str());
    }
}

/*!
 * \brief Saves neural network that was received from module runnning on a remote server.
 * \param[in] trainingErrors - errors from the whole training
 * \note This function is called after module finished and sent out all erros so that plot can be updated with all errors
 * \note and not just those received during the training process as these are usually sent only at every few iterations.
 */
void MTRNN::saveErrors(QVector<float> trainingErrors)
{
    ui->plot->graph()->clearData();

    for(int i=0; i<trainingErrors.size(); i++)
    {
        ui->plot->graph()->addData((double)i, trainingErrors.at(i));
    }

    ui->plot->replot();
}

/*!
 * \brief Starts neural network training.
 */
void MTRNN::startTraining()
{
    resetErrorLimits();

    QString path = QFileDialog::getOpenFileName(mainWindow, "Choose a seqence file for training", QDir::currentPath(),"Text Files (*.txt)");

    if(!remoteMode) //module runs on localhost and therefore it is able to load selected training file directly
    {
        //wipe data and replot
        ui->plot->graph()->clearData();
        ui->plot->replot();
        intrfc->sendTrainingRequest(path);
        mainWindow->showProgressBar();
    }
    else //module runs on a remotehost and therefore it is necessary to load the selected training file and send the data via port to the module
    {
        if(loadTrainingData(path))
        {
            intrfc->sendTrainingRequest();
            qDebug(" - %s_gui(%i): training data was successfully loaded from %s", binary.toStdString().c_str(), instance, path.toStdString().c_str());
        }
        else
        {
            qWarning(" - %s_gui(%i): training data could not be loaded from %s", binary.toStdString().c_str(), instance, path.toStdString().c_str());
        }
    }
}

/*!
 * \brief Loads training data.
 * \param[in] trainingFile - file containing training sequences
 */
bool MTRNN::loadTrainingData(QString trainingFile)
{
    QVector<int> sequenceSteps;
    QVector<float> sequenceData;
    yarp::os::ResourceFinder rf;
    yarp::os::Property config;
    yarp::os::Bottle bSample;
    yarp::os::Bottle *pS;
    int sequenceWidth = 0;
    int maxSequenceSteps = 0;
    int totalSequenceSteps = 0;
    maxValue = 0.0f;
    minValue = 99999.0f;

    //find the training file
    if(config.fromConfigFile(rf.findFile(trainingFile.toStdString().c_str())))
    {
        //load the architecture specification
        bSample = config.findGroup("ARCHITECTURE");
        if(!bSample.isNull())
        {
            numControlNeurons = bSample.find("controlNeurons").asInt();
            numLinguisticNeurons = bSample.find("linguisticNeurons").asInt();
            numVisionNeurons = bSample.find("visionNeurons").asInt();
            numActionNeurons = bSample.find("actionNeurons").asInt();
        }
        else
        {
            qCritical(" - %s_gui(%i): [ARCHITECTURE] group is missing", binary.toStdString().c_str(), instance);
            return false;
        }

        //load sequences
        char buf[20];
        bSample.clear();
        for(int i=0; i<MAX_SEQUENCES; i++)
        {
            sprintf(buf,"SEQUENCE_%i", i);
            bSample = config.findGroup(buf);
            if(bSample.isNull())
            {
                //at least the first sequence header must be found
                if(i==0)
                {
                    qCritical(" - %s_gui(%i): [SEQUENCE_0] group is missing", binary.toStdString().c_str(), instance);
                    return false;
                }
            }
            else
            {
                //save the sequence width
                if(i==0)
                {
                    sequenceWidth = bSample.get(i+1).asList()->size();
                }

                //save the sequence length information
                sequenceSteps.push_back(bSample.size()-1);
                totalSequenceSteps+=bSample.size()-1;

                 //current sequence is the new longest sequence
                if(bSample.size()-1 > maxSequenceSteps)
                {
                    maxSequenceSteps = bSample.size()-1;
                }

                //get the training data
                for(int j=0; j<sequenceSteps.at(i); j++)
                {
                    //get the whole row
                    pS = bSample.get(j+1).asList();

                    //add this sequence step to data
                    for(int k=0; k<sequenceWidth; k++)
                    {
                        float value = (float)pS->get(k).asDouble();
                        sequenceData.push_back(value);

                        //find maximum and minimum values
                        if(value > maxValue)
                        {
                            maxValue = value;
                        }

                        if(value < minValue)
                        {
                            minValue = value;
                        }
                    }
                }
            }
        }

        //send all the parameters and data to the output port so that the remote module can initialised
        intrfc->sendTrainingFileData(numControlNeurons, numLinguisticNeurons, numVisionNeurons, numActionNeurons, sequenceSteps.size(), sequenceWidth, minValue, maxValue, sequenceSteps, sequenceData, maxSequenceSteps, totalSequenceSteps);
        return true;
    }
    else
    {
        return false;
    }
}
