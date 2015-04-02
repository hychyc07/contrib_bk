//##############################################################################################################################################################################################################//
//Aquila - An Open-Source GPU-Accelerated Toolkit for Cognitive and Neuro-Robotics Research																														//
//																																																				//
//Copyright (c) <2012>, YourName                                                                                                                                                                                //
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

#include "blank.h"
#include "ui_blank.h"
#include "ui_blankSettings.h"

/*!
 * \brief Constructor.
 * \param[in] pMainWindow - pointer to MainWindow
 * \param[in] binaryName - name of module executable
 * \param[in] moduleTitle - title of this module
 * \param[in] server - server where the module runs
 * \param[in] instanceID - module identification number
 * \param[in] tabID - module tab index
 */
Blank::Blank(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString server, int instanceID, int tabID) : GUI(pMainWindow, binaryName, moduleTitle, server, instanceID, tabID), ui(new Ui::Blank)
{
	if(moduleConnected)
	{
		setupGUI();
	}
}

/*!
 * \brief Destructor.
 */
Blank::~Blank()
{
    delete ui;
}

/*!
 * \brief Sets up graphical user interface.
 */
void Blank::setupGUI()
{
	//set up user interface
    ui->setupUi(this);

    //initialise interface and settings
    intrfc = new BlankInterface(this);
    settings = new BlankSettings();

    //point base pointers to child pointers
    interfaceBase = intrfc;
    settingsBase = settings;

    //start listening for incomming messages
    intrfc->start();

    //connect signals/slots
    qRegisterMetaType<QVector<float> >("QVector<float>");
    qRegisterMetaType< QVector<float> >("QVector<QStringList>");
	QObject::connect(this, SIGNAL(tabSelected()), this, SLOT(show()));
    QObject::connect(intrfc, SIGNAL(gpuListReceived(QVector<QStringList>)), this, SLOT(setGpuList(QVector<QStringList>)));
    QObject::connect(intrfc, SIGNAL(statusReceived(int)), this, SLOT(statusChanged(int)));
    QObject::connect(intrfc, SIGNAL(parametersReceived(/*YourParameters*/)), settings, SLOT(setParameters(/*YourParameters*/)));
    QObject::connect(settings->ui->pushButton_ok, SIGNAL(clicked()), settings, SLOT(accept()));
    QObject::connect(settings, SIGNAL(accepted()), settings, SLOT(save()));
    QObject::connect(settings->ui->pushButton_cancel, SIGNAL(clicked()), settings, SLOT(reject()));

    //get the parameters from the module
    intrfc->sendParametersRequest();

    //check if the modeule's tab is currently selected
    checkFocus(mainWindow->tabWidget->currentIndex());
}

/*!
 * \brief Updates graphical user interface.
 * \note This is an overridden virtual function.
 */
void Blank::updateGUI()
{
    if(status==0) //initialised
    {
        startAction->setEnabled(true);
        stopAction->setEnabled(false);
    }
    else if(status==1) //blank running
    {
		running = true;

        startAction->setEnabled(false);
        stopAction->setEnabled(true);
        deviceSubMenu->setEnabled(false);
		
        mainWindow->statusBar()->showMessage("Blank started",2000);
        qDebug(" - %s_gui(%i): started", binary.toStdString().c_str(), instance);
    }
    else if(status==2) //blank not running
    {
		running = false;
        
		startAction->setEnabled(true);
        stopAction->setEnabled(false);
        deviceSubMenu->setEnabled(true);

        mainWindow->statusBar()->showMessage("Blank stopped", 2000);
		qDebug(" - %s_gui(%i): stopped", binary.toStdString().c_str(), instance);
    }
}

/*!
 * \brief Shows message box with the information about module.
 * \note This is an overridden virtual function.
 */
void Blank::about()
{
    QMessageBox::about(this, "Blank", "YourDescription.");
}
