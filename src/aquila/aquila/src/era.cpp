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

#include "era.h"
#include "ui_era.h"
#include "ui_eraSettings.h"
#include "ui_eraNetworkMonitor.h"

/*!
 * \brief Constructor.
 * \param[in] pMainWindow - pointer to MainWindow
 * \param[in] binaryName - name of module executable
 * \param[in] moduleTitle - title of this module
 * \param[in] server - server where the module runs
 * \param[in] instanceID - module identification number
 * \param[in] tabID - module tab index
 */
ERA::ERA(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString server, int instanceID, int tabID) : GUI(pMainWindow, binaryName, moduleTitle, server, instanceID, tabID), ui(new Ui::ERA)
{
	if(moduleConnected)
	{
		setupGUI();
	}
}

/*!
 * \brief Destructor.
 */
ERA::~ERA()
{
    delete ui;
}

/*!
 * \brief Sets up graphical user interface.
 */
void ERA::setupGUI()
{
	//set up user interface
    ui->setupUi(this);

    //initialise interface, settings and network monitor
    intrfc = new ERAInterface(this);
    settings = new ERASettings();
    networkMonitor = new ERANetworkMonitor();

    //point base pointers to child pointers
    interfaceBase = intrfc;
    settingsBase = settings;

    //start listening for incomming messages
    intrfc->start();

    //connect signals/slots
    qRegisterMetaType< QVector<float> >("QVector<float>");
    qRegisterMetaType< QVector<float> >("QVector<QStringList>");
	QObject::connect(this, SIGNAL(tabSelected()), this, SLOT(show()));
    QObject::connect(intrfc, SIGNAL(gpuListReceived(QVector<QStringList>)), this, SLOT(setGpuList(QVector<QStringList>)));
    QObject::connect(intrfc, SIGNAL(statusReceived(int)), this, SLOT(statusChanged(int)));
    QObject::connect(intrfc, SIGNAL(parametersReceived(int)), settings, SLOT(setParameters(int)));
    QObject::connect(intrfc, SIGNAL(dataReceived(int, double*, double*)), networkMonitor, SLOT(updatePlot(int, double*, double*)));
    QObject::connect(settings->ui->pushButton_ok, SIGNAL(clicked()), settings, SLOT(accept()));
    QObject::connect(settings, SIGNAL(accepted()), settings, SLOT(save()));
    QObject::connect(settings->ui->pushButton_cancel, SIGNAL(clicked()), settings, SLOT(reject()));
    QObject::connect(settings, SIGNAL(simulationModeChanged(int)), intrfc, SLOT(sendSimulationMode(int)));

    //get the parameters from the module
    intrfc->sendParametersRequest();

    //initialise viewports
    initialiseViewports();

    //check if the modeule's tab is currently selected
    checkFocus(mainWindow->tabWidget->currentIndex());
}

/*!
 * \brief Updates graphical user interface.
 * \note This is an overridden virtual function.
 */
void ERA::updateGUI()
{
    if(status==0) //initialised
    {
        startAction->setEnabled(true);
        stopAction->setEnabled(false);
    }
    else if(status==1) //ERA running
    {
		running = true;

        startAction->setEnabled(false);
        stopAction->setEnabled(true);
        deviceSubMenu->setEnabled(false);
		
        ui->leftCam->start();

        mainWindow->statusBar()->showMessage("ERA started", 2000);
        qDebug(" - %s_gui(%i): started", binary.toStdString().c_str(), instance);
    }
    else if(status==2) //ERA not running
    {
		running = false;
        
		startAction->setEnabled(true);
        stopAction->setEnabled(false);
        deviceSubMenu->setEnabled(true);

        ui->leftCam->stop();

        mainWindow->statusBar()->showMessage("ERA stopped", 2000);
		qDebug(" - %s_gui(%i): stopped", binary.toStdString().c_str(), instance);
    }
}

/*!
 * \brief Initialises viewports.
 */
void ERA::initialiseViewports()
{
    ui->leftCam->initialise(QString("/era/")+QString::number(instance)+QString("/cam/left:o"), server+QString("/era/")+QString::number(instance)+QString("/cam/left:i"));
}

/*!
 * \brief Shows message box with the information about module.
 * \note This is an overridden virtual function.
 */
void ERA::about()
{
    QMessageBox::about(this, "ERA", "This is the ERA module by Anthony Morse");
}

/*!
 * \brief Creates view menu.
 * \note This is an overridden virtual function.
 */
void ERA::createViewMenu()
{
    viewMenu = mainWindow->menuBar()->addMenu(tr("&View"));
    mainWindow->addViewServerAction(viewMenu);

    networkMonitorAction = new QAction("Network &Monitor", this);
    networkMonitorAction->setShortcut(QKeySequence("ALT+M"));
    networkMonitorAction->setStatusTip("Diplay Network Monitor");
    QObject::connect(networkMonitorAction, SIGNAL(triggered()), networkMonitor, SLOT(show()));

    viewMenu->addAction(networkMonitorAction);
}

