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

#include "som.h"
#include "ui_som.h"
#include "ui_somSettings.h"

/*!
 * \brief Constructor.
 * \param[in] pMainWindow - pointer to MainWindow
 * \param[in] binaryName - name of module executable
 * \param[in] moduleTitle - title of this module
 * \param[in] server - server where the module runs
 * \param[in] instanceID - module identification number
 * \param[in] tabID - module tab index
 */
SOM::SOM(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString serverName, int instanceID, int tabID) : GUI(pMainWindow, binaryName, moduleTitle, serverName, instanceID, tabID), ui(new Ui::SOM)
{
	if(moduleConnected)
	{
		setupGUI();
	}
}

/*!
 * \brief Destructor.
 */
SOM::~SOM()
{  
    delete ui;
}

/*!
 * \brief Sets up graphical user interface.
 */
void SOM::setupGUI()
{
	//set up user interface
    ui->setupUi(this);

    //initialise interface and settings
    intrfc = new SOMInterface(this);
    settings = new SOMSettings();

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
	QObject::connect(this, SIGNAL(tabSelected()), ui->plot3d, SLOT(enableRendering()));
	QObject::connect(this, SIGNAL(tabDeselected()), ui->plot3d, SLOT(disableRendering()));
    QObject::connect(ui->checkBox_visualiseLearningProgress, SIGNAL(stateChanged(int)), intrfc, SLOT(sendVisualiseLearning(int)));
    QObject::connect(ui->comboBox_x, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));
    QObject::connect(ui->comboBox_y, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));
    QObject::connect(ui->comboBox_z, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));
    QObject::connect(intrfc, SIGNAL(gpuListReceived(QVector<QStringList>)), this, SLOT(setGpuList(QVector<QStringList>)));
    QObject::connect(intrfc, SIGNAL(mapReceived(QVector<float>)), this, SLOT(setPlotInputs(QVector<float>)));
    QObject::connect(intrfc, SIGNAL(progressReceived(int)), this, SLOT(updateProgressBar(int)));
    QObject::connect(intrfc, SIGNAL(numInputsReceived(int)), this, SLOT(initInputMappingComboBoxes(int)));
    QObject::connect(intrfc, SIGNAL(statusReceived(int)), this, SLOT(statusChanged(int)));
    QObject::connect(intrfc, SIGNAL(trainingTimeReceived(float)), this, SLOT(saveTrainingTime(float)));
    QObject::connect(intrfc, SIGNAL(dataPointLimitsReceived(float,float)), this, SLOT(saveDataPoints(float,float)));
    QObject::connect(intrfc, SIGNAL(parametersReceived(float,int,int,int)), settings, SLOT(setParameters(float,int,int,int)));
    QObject::connect(settings->ui->pushButton_ok, SIGNAL(clicked()), settings, SLOT(accept()));
    QObject::connect(settings, SIGNAL(accepted()), settings, SLOT(save()));
    QObject::connect(settings->ui->pushButton_cancel, SIGNAL(clicked()), settings, SLOT(reject()));
    QObject::connect(settings->ui->spinBox_numOutputs, SIGNAL(valueChanged(int)), settings, SLOT(setOutputSizeValue(int)));
    QObject::connect(settings, SIGNAL(parametersSaved(float,int,int,int)), intrfc, SLOT(sendParameters(float,int,int,int)));

	//set flags
    mapLoaded = false;
    viewingMode = false;
	useModuleMenu = false;

    //get the parameters from the module
    intrfc->sendParametersRequest();

    //initialise cobmobox used for changing rendering mode of the 3d plot
    initRenderModeComboBox();

    //check if the modeule's tab is currently selected
    checkFocus(mainWindow->tabWidget->currentIndex());
}

/*!
 * \brief Updates graphical user interface.
 * \note This is an overridden virtual function.
 */
void SOM::updateGUI()
{
    if(status==0) //initialised
    {
        abortAct->setVisible(false);
        saveSubMenu->setEnabled(false);
    }
    else if(status==1) //training in progress
    {
		running = true;
        mapLoaded = false;

        trainAct->setVisible(false);
        viewAct->setEnabled(false);
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
        mapLoaded = true;
        viewingMode = false;

        trainAct->setVisible(true);
        viewAct->setEnabled(true);
        settingsAct->setEnabled(true);
        abortAct->setVisible(false);
        deviceSubMenu->setEnabled(true);
        saveSubMenu->setEnabled(true);
        saveAct[0]->setEnabled(true);
        saveAct[1]->setEnabled(true);

        mainWindow->statusBar()->showMessage("Training finished", 2000);
		qDebug(" - %s_gui(%i): trainig finished", binary.toStdString().c_str(), instance);
    }
}

/*!
 * \brief Creates file menu.
 * \note This is an overridden virtual function.
 */void SOM::createFileMenu()
{
    abortAct = new QAction("&Abort", this);
    abortAct->setShortcut(QKeySequence("Ctrl+A"));
    abortAct->setStatusTip("Abort training process");
    QObject::connect(abortAct, SIGNAL(triggered()), this, SIGNAL(abortRequested()));

    trainAct = new QAction("&New", this);
    trainAct->setShortcut(QKeySequence("Ctrl+N"));
    trainAct->setStatusTip("Train a new self-organising map");
    QObject::connect(trainAct, SIGNAL(triggered()), this, SLOT(startTraining()));

    viewAct = new QAction("&View", this);
    viewAct->setShortcut(QKeySequence("Ctrl+V"));
    viewAct->setStatusTip("View an existing self-organising map");
    QObject::connect(viewAct, SIGNAL(triggered()), this, SLOT(viewMap()));

    quitAct = new QAction("&Quit", this);
    quitAct->setShortcut(QKeySequence("Ctrl+Q"));
    quitAct->setStatusTip("Quit Aquila");
    QObject::connect(quitAct, SIGNAL(triggered()), mainWindow, SLOT(quit()));

    fileMenu = mainWindow->menuBar()->addMenu("&File");
    fileMenu->addAction(trainAct);
    fileMenu->addAction(abortAct);
    fileMenu->addAction(viewAct);
    createSaveSubmenu();
    fileMenu->addSeparator();
    fileMenu->addAction(quitAct);
}

/*!
 * \brief Creates save menu.
 */
void SOM::createSaveSubmenu()
{
    //create a new submenu and signal mapper
    saveSubMenu = fileMenu->addMenu("&Save");
    saveSignalMapper = new QSignalMapper(saveSubMenu);
    saveSubMenu->clear();

    //save neural networks action
    saveAct[0] = new QAction("Map", this);
    saveAct[0]->setStatusTip("Save self-organising map");
    saveSignalMapper->setMapping(saveAct[0], 0);
    QObject::connect(saveAct[0], SIGNAL(triggered()), saveSignalMapper, SLOT(map()));

    //save plot action
    saveAct[1] = new QAction("Plot", this);
    saveAct[1]->setStatusTip("Save plot");
    saveSignalMapper->setMapping(saveAct[1], 1);
    QObject::connect(saveAct[1], SIGNAL(triggered()), saveSignalMapper, SLOT(map()));

    //add actions and connect signal mapper
    QObject::connect(saveSignalMapper, SIGNAL(mapped(int)), this, SLOT(save(int)));
    saveSubMenu->addAction(saveAct[0]);
    saveSubMenu->addAction(saveAct[1]);
}

/*!
 * \brief Shows message box with the information about module.
 * \note This is an overridden virtual function.
 */
void SOM::about()
{
    QMessageBox::about(this, "Self-organising map", "This module implements self-organising map and its visualisation.");
}

/*!
 * \brief Initialises combo box for selection of rendering mode.
 */
void SOM::initRenderModeComboBox()
{
    QStringList renderingModes;
    renderingModes <<"Filled"<<"Wireframe"<<"Points";
    ui->comboBox_renderingMode->addItems(renderingModes);
    QObject::connect(ui->comboBox_renderingMode, SIGNAL(currentIndexChanged(int)), ui->plot3d, SLOT(setSurfaceRenderMode(int)));
}

/*!
 * \brief Sets plot inputs.
 * \param[in] map - self-organasing map data
 */
void SOM::setPlotInputs(QVector<float> map)
{
    if(running && ui->plot3d->initialised)
    {
        ui->plot3d->updateData(map, ui->comboBox_x->currentIndex(), ui->comboBox_y->currentIndex(), ui->comboBox_z->currentIndex());
    }
    else
    {
        ui->plot3d->setData(map, (int)sqrt((float)settings->ui->spinBox_numOutputs->value()), ui->comboBox_x->currentIndex(), ui->comboBox_y->currentIndex(), ui->comboBox_z->currentIndex());
    }
}

/*!
 * \brief Initialises combo box for selection of inputs to be mapped to self-organising map.
 * \param[in] numInputs - number of self-organising map inputs
 */
void SOM::initInputMappingComboBoxes(int numInputs)
{
    QObject::disconnect(ui->comboBox_x, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));
    QObject::disconnect(ui->comboBox_y, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));
    QObject::disconnect(ui->comboBox_z, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));

    settings->numInputs = numInputs;

    ui->comboBox_x->clear();
    ui->comboBox_y->clear();
    ui->comboBox_z->clear();

    //add vector IDs
    for(int i=0; i<settings->numInputs; i++)
    {
        ui->comboBox_x->addItem(QString::number(i));
        ui->comboBox_y->addItem(QString::number(i));
        ui->comboBox_z->addItem(QString::number(i));
    }

    //set defaults
    ui->comboBox_x->setCurrentIndex(0);
    ui->comboBox_y->setCurrentIndex(1);

    if(settings->numInputs>2)
    {
        ui->comboBox_z->setCurrentIndex(2);
    }
    else
    {
        ui->comboBox_z->setCurrentIndex(1);
    }

    showMapProperties(settings->ui->spinBox_numOutputs->value(), settings->numInputs, settings->ui->spinBox_subIterations->value(), settings->ui->doubleSpinBox_learningRate->value());

    QObject::connect(ui->comboBox_x, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));
    QObject::connect(ui->comboBox_y, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));
    QObject::connect(ui->comboBox_z, SIGNAL(currentIndexChanged(int)), this, SLOT(inputMappingChanged()));
}

/*!
 * \brief Starts training.
 */
void SOM::startTraining()
{
    QString path = QFileDialog::getOpenFileName(mainWindow, "Choose a seqence file for training", QDir::currentPath(),"Text Files (*.txt)");

    if(!remoteMode)
    {
        ui->plot3d->initialised = false;
        intrfc->sendTrainingRequest(path);
        mainWindow->showProgressBar();
    }
    else
    {
        yarp::os::ResourceFinder rf;
        yarp::os::Property config;
        QVector<double> w;
        int samples;
        int inputs;

        if(config.fromConfigFile(rf.findFile(path.toStdString().c_str())))
        {
            yarp::os::Bottle &bSample = config.findGroup("SAMPLES");

            if(bSample.isNull())
            {
                qCritical(" - %s_gui(%i): [SAMPLES] group is missing", binary.toStdString().c_str(), instance);
            }
            else
            {
                ui->plot3d->initialised = false;

                samples = bSample.size() - 1;
                inputs = bSample.get(1).asList()->size();
                float *tmp_w = new float[inputs*samples];

                //get samples
                for(int i=0; i<samples; i++)
                {
                    yarp::os::Bottle *pS = bSample.get(i+1).asList();
                    for(int j=0; j<inputs; j++)
                    {
                        float value = pS->get(j).asDouble();
                        tmp_w[i+(j*samples)] = value;
                    }
                }

                for(int i=0; i<inputs*samples; i++)
                {
                    w.append(tmp_w[i]);
                }

                intrfc->sendTrainingRequest(samples, inputs, w);
                mainWindow->showProgressBar();
            }
        }
    }
}

/*!
 * \brief Views self-organising map.
 */
void SOM::viewMap()
{
    QString path = QFileDialog::getOpenFileName(mainWindow, "Choose a self-organising map to view", QDir::currentPath(), "Text Files (*.txt)");

    yarp::os::ResourceFinder rf;
    yarp::os::Property config;
    yarp::os::Value *value;
    int subIterationsUsed;
    double learningRateUsed;

    if(config.fromConfigFile(rf.findFile(path.toStdString().c_str())))
    {
        yarp::os::Bottle &info = config.findGroup("INFORMATION");
        if(!info.isNull())
        {
            if(info.check("learningRate", value))
            {
                learningRateUsed = value->asDouble();
            }

            if(info.check("subIterations", value))
            {
                subIterationsUsed = value->asInt();
            }
        }

        yarp::os::Bottle &bSample = config.findGroup("WEIGHTS");

        if(bSample.isNull())
        {
            qCritical(" - %s_gui(%i): [WEIGHTS] group is missing", binary.toStdString().c_str(), instance);
        }
        else
        {
            int outputs = bSample.size() - 1;
            int inputs = bSample.get(1).asList()->size();
            float *tmp_w = new float[inputs*outputs];
            QVector<float> w;

            //get weights
            for(int i=0; i<outputs; i++)
            {
                yarp::os::Bottle *pS = bSample.get(i+1).asList();
                for(int j=0; j<inputs; j++)
                {
                    float value = pS->get(j).asDouble();
                    tmp_w[i+(j*outputs)] = value;
                }
            }

            for(int i=0; i<inputs*outputs; i++)
            {
                w.append(tmp_w[i]);
            }

            //update settings
            settings->ui->spinBox_numOutputs->blockSignals(true);
            settings->ui->spinBox_numOutputs->setValue(outputs);
            settings->ui->spinBox_numOutputs->blockSignals(false);
            settings->ui->doubleSpinBox_learningRate->setValue(learningRateUsed);
            settings->ui->spinBox_subIterations->setValue(subIterationsUsed);
            settings->accept();

            initInputMappingComboBoxes(inputs);
            setPlotInputs(w);
            saveSubMenu->setEnabled(true);
            //disable saving since we are just viewing map from a file
            saveAct[0]->setDisabled(true);
            mapLoaded = true;
            viewingMode = true;
        }
    }
}

/*!
 * \brief Saves self-organising map or plot image.
 * \param[in] targetID - save target
 * \note This function is called when user clicked on any of the save elements in the save sub-menu.
 */
void SOM::save(int targetID)
{
    switch (targetID)
    {
    case 0:
        qDebug(" - %s_gui(%i): saving map", binary.toStdString().c_str(), instance);
        
        if(!remoteMode)
        {
            intrfc->sendSaveRequest(QFileDialog::getSaveFileName(mainWindow, "Choose a file to save the self organising map", QDir::currentPath()+"/untitled.txt", "Text Files (*.txt)"));
        }
        else
        {
            QString path = QFileDialog::getSaveFileName(mainWindow, "Choose a file to save the self organising map",QDir::currentPath()+"/untitled.txt", "Text Files (*.txt)");
            QFile saveFile(path);
            QTextStream out(&saveFile);
            char buf[50];

            if(!saveFile.open(QIODevice::WriteOnly | QIODevice::Text))
            {
                qCritical(" - %s_gui(%i): failed to open %s", binary.toStdString().c_str(), instance, saveFile.fileName().toStdString().c_str());
            }
            else
            {
                out<<"[INFORMATION]\n";
                if(settings->gpu.active) out<<"GPU "<<settings->gpu.selected[0]<<"\n";
                else	out<<"CPU \n";
                out<<"trainingTime "<<trainingTime<<" ms"<<"\n";
                out<<"learningRate "<<settings->ui->doubleSpinBox_learningRate->value()<<"\n";
                out<<"subIterations "<<settings->ui->spinBox_subIterations->value()<<"\n";
                out<<"outputs "<<settings->ui->spinBox_numOutputs->value()<<"\n";
                out<<"highestSourceDataValue "<<highestDataPoint<<"\n";
                out<<"lowestSourceDataValue "<<lowestDataPoint<<"\n";
                out<<"[WEIGHTS]\n";

                //write the weights of the currently loaded self-organising map
                for(int j=0; j<settings->ui->spinBox_numOutputs->value(); j++)
                {
                    for(int i=0; i<settings->numInputs; i++)
                    {
                        sprintf(buf,"%.6f", ui->plot3d->weights[(i*settings->ui->spinBox_numOutputs->value())+j]);
                        out<<buf;
                        if(i<settings->numInputs-1)
                        {
                            out<<"\t";
                        }
                    }
                    out<<"\n";
                }
                saveFile.close();
                qDebug(" - %s_gui(%i): map was saved to %s", binary.toStdString().c_str(), instance, path.toStdString().c_str());
            }
        }
        break;

    case 1:
        qDebug(" - %s_gui(%i): saving plot as image", binary.toStdString().c_str(), instance);
        QString fileName = QFileDialog::getSaveFileName(this, "Save Plot", QDir::currentPath()+"/untitled.png","Images (*.png *.bmp *.jpg *.jpg)");
        QImage image = ui->plot3d->grabFrameBuffer();
        image.save(fileName);
        qDebug(" - %s_gui(%i): plot was saved to %s", binary.toStdString().c_str(), instance, fileName.toStdString().c_str());
        break;
    }
}

/*!
 * \brief Shows self-organising map properties on the 3dplot.
 * \param[in] numOutputs - number of outputs
 * \param[in] vectorDimensions - number of dimensions of the input vector
 * \param[in] subIterationsUsed - number of sub-iterations used during for the training
 * \param[in] learningRateUsed - learning rate used for the training
 */
void SOM::showMapProperties(int numOutputs, int vectorDimensions, int subIterationsUsed, double learningRateUsed)
{
    QStringList information;
    information<<QString("outputs: ")+QString::number(numOutputs)
               <<QString("dimensions: ")+QString::number(vectorDimensions)
               <<QString("learning rate: ")+QString::number(learningRateUsed)
               <<QString("sub-iterations: ")+QString::number(subIterationsUsed);

    ui->plot3d->setInformation(information);
}

/*!
 * \brief Sets plot inputs when input mapping changes.
 */
void SOM::inputMappingChanged()
{
    setPlotInputs(ui->plot3d->weights);
}

/*!
 * \brief Saves self-organising map training time.
 * \param[in] time - training time
 */
void SOM::saveTrainingTime(float time)
{
    trainingTime = time;
}

/*!
 * \brief Saves original self-organising map data points.
 * \param[in] low - lowest data point
 * \param[in] hi - highest data point
 */
void SOM::saveDataPoints(float low, float hi)
{
    lowestDataPoint = low;
    highestDataPoint = hi;
}
