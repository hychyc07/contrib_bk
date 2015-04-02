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

#include "esn.h"
#include "ui_esn.h"
#include "ui_esnSettings.h"

/*!
 * \brief Constructor.
 * \param[in] pMainWindow - pointer to MainWindow
 * \param[in] binaryName - name of module executable
 * \param[in] moduleTitle - title of this module
 * \param[in] server - server where the module runs
 * \param[in] instanceID - module identification number
 * \param[in] tabID - module tab index
 */
ESN::ESN(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString server, int instanceID, int tabID) : GUI(pMainWindow, binaryName, moduleTitle, server, instanceID, tabID), ui(new Ui::ESN)
{
	if(moduleConnected)
	{
		setupGUI();
	}
}

/*!
 * \brief Destructor.
 */
ESN::~ESN()
{
    delete ui;
}

/*!
 * \brief Sets up graphical user interface.
 */
void ESN::setupGUI()
{
	//set up user interface
    ui->setupUi(this);

    //initialise interface and settings
    intrfc = new ESNInterface(this);
    settings = new ESNSettings();

    //point base pointers to child pointers
    interfaceBase = intrfc;
    settingsBase = settings;

    //start listening for incomming messages
    intrfc->start();

    //connect signals/slots
    qRegisterMetaType<QVector<float> >("QVector<float>");
    qRegisterMetaType< QVector<float> >("QVector<QStringList>");
	QObject::connect(this, SIGNAL(tabSelected()), this, SLOT(show()));
	QObject::connect(intrfc, SIGNAL(statusReceived(int)), this, SLOT(statusChanged(int)));
    QObject::connect(intrfc, SIGNAL(gpuListReceived(QVector<QStringList>)), this, SLOT(setGpuList(QVector<QStringList>)));
    QObject::connect(intrfc, SIGNAL(parametersReceived(int)), settings, SLOT(setParameters(int)));
    QObject::connect(intrfc, SIGNAL(networkActivityReceived(QVector<float>)), this, SLOT(displayESN(QVector<float>)));
    QObject::connect(settings->ui->pushButton_ok, SIGNAL(clicked()), settings, SLOT(accept()));
    QObject::connect(settings, SIGNAL(accepted()), settings, SLOT(save()));
    QObject::connect(settings->ui->pushButton_cancel, SIGNAL(clicked()), settings, SLOT(reject()));
    QObject::connect(settings, SIGNAL(parametersSaved(int)), intrfc, SLOT(sendParameters(int)));

    //get the parameters from the module
    intrfc->sendParametersRequest();

    //check if the modeule's tab is currently selected
    checkFocus(mainWindow->tabWidget->currentIndex());
}

/*!
 * \brief Updates graphical user interface.
 * \note This is an overridden virtual function.
 */
void ESN::updateGUI()
{
    if(status==0) //initialised
    {
        startAction->setEnabled(true);
        stopAction->setEnabled(false);
    }
    else if(status==1) //esn running
    {
		running = true;

        startAction->setEnabled(false);
        stopAction->setEnabled(true);
        deviceSubMenu->setEnabled(false);

        mainWindow->statusBar()->showMessage("ESN started", 2000);
        qDebug(" - %s_gui(%i): ESN started", binary.toStdString().c_str(), instance);
    }
    else if(status==2) //esn not running
    {
		running = false;
        
		startAction->setEnabled(true);
        stopAction->setEnabled(false);
        deviceSubMenu->setEnabled(true);

        mainWindow->statusBar()->showMessage("ESN stopped", 2000);
        qDebug(" - %s_gui(%i): ESN stoped", binary.toStdString().c_str(), instance);
    }
}

/*!
 * \brief Displays dialog showing basic information about module.
 */
void ESN::about()
{
    QMessageBox::about(this, "ESN", "This module implements echo state networks.");
}

/*!
 * \brief Displays ESN activity.
 */
void ESN::displayESN(QVector<float> activity)
{
    //work out how big the image should be
    int dimSize = (int)sqrt((float)activity.size());

    //declare an image
    QImage esnImage(dimSize, dimSize, QImage::Format_RGB32);

    //draw the image
    for(int i=0; i<dimSize; i++)
    {
        for(int j=0; j<dimSize; j++)
        {
            if(activity.at((i*dimSize)+j)>0)
            {
                esnImage.setPixel(i,j,qRgb(255 * activity.at((i*dimSize)+j), 0, 0));
            }
            else
            {
                esnImage.setPixel(i,j,qRgb(0, 0, 255 * -activity.at((i*dimSize)+j)));
            }
        }
    }

    //display the image
    ui->view->setImage(esnImage);
}
