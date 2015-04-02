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

#include <QtNetwork/QHostInfo>
#include "settings.h"

/*!
 * \brief Constructor
 * \note This constructor tries to start a new process and launch module. If this fails at firs time, it will try again.
 * \note This second trial is currently a workaround for QProcess failing to start a module on OSX on the first time.
 * \note The reason for this remains unclear, possibly an OSX issue with either yarprun or QProcess.
 * \param[in] pMainWindow - pointer to MainWindow
 * \param[in] binaryName - name of module executable
 * \param[in] moduleTitle - title of this module
 * \param[in] server - server where the module runs
 * \param[in] instanceID - module identification number
 * \param[in] tabID - module tab index
 */
GUI::GUI(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString serverName, int instanceID, int tabID)
{
	bool success = startModule(binaryName, serverName, instanceID);
    if(!success)
	{
		module->close();
		qCritical(" - %s_gui(%i): trying to relaunch QProcess and connect again", binaryName.toStdString().c_str(), instanceID);
		success = startModule(binaryName, serverName, instanceID);	
	}
    if(success) initialise(pMainWindow, binaryName, moduleTitle, serverName, instanceID, tabID);
}

/*!
 * \brief Destructor.
 */
GUI::~GUI()
{
}

/*!
 * \brief Starts module.
 * \param[in] binaryName - module executable name
 * \param[in] serverName - name of server running module
 * \param[in] instanceID - module instance identification number
 * \return true on success
 */
bool GUI::startModule(QString binaryName, QString serverName, int instanceID)
{
    QString program;
    QStringList arguments;

    //start module
    program = "yarprun";
    arguments<<"--on"<<serverName.toStdString().c_str()<<
               "--as"<<QString(binaryName + QString("_") + QString::number(instanceID)).toStdString().c_str()<<
               "--cmd"<<QString(binaryName + QString(" --id ") + QString::number(instanceID)).toStdString().c_str();
    module = new QProcess(this);
    module->start(program, arguments);

    //port name used for checking whether module module started successfully
    QString portName = QString("/") + binaryName + QString("/") + QString::number(instanceID) + QString(":o");

    //try to esablish connection with the module
    return timeoutConnectTo(binaryName, portName, instanceID);
}

/*!
 * \brief Checks for successful start of module.
 * \param[in] binaryName - module executable name
 * \param[in] portName - module port name
 * \param[in] instanceID - module instance identification number
 * \return true on success
 */
bool GUI::timeoutConnectTo(QString binaryName, QString portName, int instanceID)
{
    int timeout = 0;
	moduleConnected = false;

    //try to connect the ports
    while(!moduleConnected)
    {
        qDebug(" - %s_gui(%i): trying to connect to %s (trial %i out of %i)", binaryName.toStdString().c_str(), instanceID, portName.toStdString().c_str(), timeout, MAX_TIMEOUT_ATTEMPTS);
        yarp::os::Time::delay(0.1);
        if(!moduleConnected)
        {
            if(yarp::os::Network::exists(portName.toStdString().c_str(), true))
            {
                qDebug(" - %s_gui(%i): connection established", binaryName.toStdString().c_str(), instanceID);
                moduleConnected = true;
            }
            else
            {
                moduleConnected = false;
            }
        }

        timeout++;
        if(timeout==MAX_TIMEOUT_ATTEMPTS)
        {
            qCritical(" - %s_gui(%i): unable to establish connection", binaryName.toStdString().c_str(), instanceID);
			break;
        }
    }

    return moduleConnected;
}

/*!
 * \brief Initialises module's graphical user interface.
 * \param[in] pMainWindow - pointer to MainWindow
 * \param[in] binaryName - module executable name
 * \param[in] moduleTitle - module title
 * \param[in] serverName - name of server running module
 * \param[in] instanceID - module instance identification number
 */
void GUI::initialise(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString serverName, int instanceID, int tabID)
{
    setAttribute(Qt::WA_DeleteOnClose);
	mainWindow = pMainWindow;
    instance = instanceID;
    binary = binaryName;
	title = moduleTitle;
    server = serverName;
    status = 0;
    progress = -1;
    selected = false;
	running = false;
    useModuleMenu = true;

	#ifdef NTDDI_WIN7
    win7ProgressBarId = -1;
	#endif

    //set remote mode flag and set tab tag
    if(server==QString("/aquila/server/") + QHostInfo::localHostName())
    {
        remoteMode = false;

        if(instance!=0)
        {
            tag = title + QString(" (") + QString::number(instance) + QString(")");
        }
        else
        {
            tag = title;
        }

        qDebug(" - %s_gui(%i): running in local mode", binary.toStdString().c_str(), instance);
    }
    else
    {
        remoteMode = true;
        QString id = server.section("/", 3, 3);

        if(instance!=0)
        {
            tag = title + QString(" (") + QString::number(instance) + QString(",") + id + QString(")");
        }
        else
        {
            tag = title + QString(" (0,") + id + QString(")");
        }

        qDebug(" - %s_gui(%i): running in remote mode", binary.toStdString().c_str(), instance);
    }

    if(tabID>=0)
    {
        //specific tab requested
        mainWindow->tabWidget->insertTab(tabID, this, tag);
    }
    else
    {
        mainWindow->tabWidget->addTab(this, tag);
    }

    //connect module-non-specific slots/signals
    QObject::connect(mainWindow, SIGNAL(quitting()), this, SLOT(closeModule()));
    QObject::connect(mainWindow->tabWidget, SIGNAL(currentChanged(int)), this, SLOT(checkFocus(int)));
    QObject::connect(mainWindow->tabWidget, SIGNAL(removeRequested(int)), this, SLOT(remove(int)));
    QObject::connect(mainWindow->tabWidget, SIGNAL(duplicateRequested(int)), this, SLOT(duplicate(int)));
}

/*!
 * \brief Removes module.
 * \param[in] tabID - module tab identification number
 */
void GUI::remove(int tabID)
{
    if(tabID==mainWindow->tabWidget->indexOf(this) && !mainWindow->tabWidget->removed)
	{
        mainWindow->tabWidget->removed = true;
        mainWindow->tabWidget->removeTab(tabID);
        closeModule();
	}
}

/*!
 * \brief Duplicates module.
 * \param[in] tabID - module tab identification number
 */
void GUI::duplicate(int tabID)
{
    if(tabID==mainWindow->tabWidget->indexOf(this))
	{
        mainWindow->addModule(binary, server);
	}
}

/*!
 * \brief Shows menu.
 */
void GUI::show()
{
	createMenu();
	updateGUI();
    updateProgressBar(progress);
}

/*!
 * \brief Checks if the module tab is currently selected.
 * \note This function emits signals that broadcast if module is selected or not.
 * \param[in] tabID - module tab identification number
 */
void GUI::checkFocus(int tabID)
{
    if(tabID==mainWindow->tabWidget->indexOf(this))
    {
		selected = true;
		emit tabSelected();
    }
    else
    {
		selected = false;
        emit tabDeselected();
    }
}

/*!
 * \brief Updates module based on the new status.
 * \param[in] newStatus - new status received from module
 */
void GUI::statusChanged(int newStatus)
{
    status = newStatus;
    updateGUI();
    updateProgressBar(progress);
	updateProgressBarClients();
}

/*!
 * \brief Creates menu.
 * \note This is a virtual function that can be overridden.
 */
void GUI::createMenu()
{
    //clear anything that is currently in the menu
    mainWindow->menuBar()->clear();

    //add menu
    createFileMenu();
    createEditMenu();
    createViewMenu();
    if(useModuleMenu)
    {
        createModuleMenu();
    }
    mainWindow->createToolsMenu();
    createHelpMenu();

    //request GPU list from module, which will subsequently update device menu
    interfaceBase->sendGpuListRequest();
}

/*!
 * \brief Creates file menu.
 * \note This is a virtual function that can be overridden.
 */
void GUI::createFileMenu()
{
    quitAct = new QAction("&Quit", this);
    quitAct->setShortcut(QKeySequence("Ctrl+Q"));
    quitAct->setStatusTip("Quit Aquila");
    QObject::connect(quitAct, SIGNAL(triggered()), mainWindow, SLOT(quit()));

    fileMenu = mainWindow->menuBar()->addMenu("&File");
    fileMenu->addSeparator();
    fileMenu->addAction(quitAct);
    fileMenu->hide();
}

/*!
 * \brief Creates edit menu.
 * \note This is a virtual function that can be overridden.
 */
void GUI::createEditMenu()
{
    settingsAct = new QAction("&Options", this);
    settingsAct->setShortcut(QKeySequence("Ctrl+O"));
    settingsAct->setStatusTip("Change module settings");
    QObject::connect(settingsAct, SIGNAL(triggered()), settingsBase, SLOT(show()));

    editMenu = mainWindow->menuBar()->addMenu("&Edit");
    editMenu->addAction(settingsAct);
}

/*!
 * \brief Creates view menu.
 * \note This is a virtual function that can be overridden.
 */
void GUI::createViewMenu()
{
    viewMenu = mainWindow->menuBar()->addMenu(tr("&View"));
    mainWindow->addViewServerAction(viewMenu);
}

/*!
 * \brief Creates device sub-menu.
 * \note This is a virtual function that can be overridden.
 */
void GUI::createDeviceSubmenu()
{
    //create sub-menu and initialse signal mapper
    deviceSubMenu = editMenu->addMenu("&Device");
    deviceSignalMapper = new QSignalMapper(deviceSubMenu);
    settingsBase->gpu.count = settingsBase->gpu.list.size();

    //create actions
    for(int i=0; i<settingsBase->gpu.count+1; i++)
    {
        if(i==settingsBase->gpu.count) //CPU
        {
            deviceAct[i] = new QAction(mainWindow->getServerCpuType(server), this);
            deviceAct[i]->setCheckable(true);
            deviceAct[i]->setStatusTip(QString("Change to CPU"));
            deviceSignalMapper->setMapping(deviceAct[i], i);
            QObject::connect(deviceAct[i], SIGNAL(triggered()), deviceSignalMapper, SLOT(map()));
        }
        else //GPU
        {
            deviceAct[i] = new QAction(settingsBase->gpu.list.at(i).at(0).toStdString().c_str(), this);
            deviceAct[i]->setCheckable(true);
            deviceAct[i]->setStatusTip(QString("Change to ")+settingsBase->gpu.list.at(i).at(0));
            deviceSignalMapper->setMapping(deviceAct[i], i);
            QObject::connect(deviceAct[i], SIGNAL(triggered()), deviceSignalMapper, SLOT(map()));
        }
    }

    //connect actions to signal mapper
    QObject::connect(deviceSignalMapper, SIGNAL(mapped(int)), this, SLOT(setGpu(int)));

    //add actions to the sub-menu
    for(int i=0; i<settingsBase->gpu.count; i++)
    {
        deviceSubMenu->addAction(deviceAct[i]);
    }
    deviceSubMenu->addSeparator();
    deviceSubMenu->addAction(deviceAct[settingsBase->gpu.count]);

    //check active devices
    for(int i=0; i<settingsBase->gpu.count+1; i++)
    {
        if(i==settingsBase->gpu.count)//CPU
        {
            if(!settingsBase->gpu.active)
            {
                deviceAct[i]->setChecked(true);
            }
        }
        else
        {
            if(settingsBase->gpu.selected[i] > -1)
            {
                deviceAct[i]->setChecked(true);
            }
            else
            {
                deviceAct[i]->setChecked(false);
            }
        }
    }
}

/*!
 * \brief Creates module menu.
 * \note This is a virtual function that can be overridden.
 */
void GUI::createModuleMenu()
{
    startAction = new QAction("&Start", this);
    startAction->setShortcut(QKeySequence("CTRL+S"));
    startAction->setStatusTip("Start module");
    QObject::connect(startAction, SIGNAL(triggered()), interfaceBase, SLOT(sendStartRequest()));

    stopAction = new QAction("&Stop", this);
    stopAction->setShortcut(QKeySequence("CTRL+S"));
    stopAction->setStatusTip("Stop module");
    QObject::connect(stopAction, SIGNAL(triggered()), interfaceBase, SLOT(sendStopRequest()));

    moduleMenu = mainWindow->menuBar()->addMenu(title);
    moduleMenu->addAction(startAction);
    moduleMenu->addAction(stopAction);
}

/*!
 * \brief Creates help menu.
 * \note This is a virtual function that can be overridden.
 */
void GUI::createHelpMenu()
{
    aboutAct[0] = new QAction(tr("&About"), this);
    aboutAct[0]->setStatusTip(tr("About Aquila"));
    QObject::connect(aboutAct[0], SIGNAL(triggered()), mainWindow, SLOT(about()));

    aboutAct[1] = new QAction(QString("About ") + title, this);
    aboutAct[1]->setStatusTip(QString("About ") + title + QString(" module"));
    QObject::connect(aboutAct[1], SIGNAL(triggered()), this, SLOT(about()));

    helpMenu = mainWindow->menuBar()->addMenu("Help");
    helpMenu->addAction(aboutAct[0]);
    helpMenu->addAction(aboutAct[1]);
}

/*!
 * \brief Sets GPU list and subsequently calls a function that constructs device sub-menu.
 * \param[in] list - list of GPU devices present on server running module
 * \note This function is called when interface receives a list of GPU devices from module.
 */
void GUI::setGpuList(QVector<QStringList> gpuList)
{
    int index = mainWindow->getServerIndex(server);

    if(gpuList.size()>0)
    {
        mainWindow->server.at(index)->gpuInfo.clear();
        for(int i=0; i<gpuList.size(); i++)
        {
            for(int j=1; j<gpuList.at(i).size(); j++)
            {
                mainWindow->server.at(index)->gpuInfo.append(gpuList.at(i).at(j) + QString("\n"));
            }
            if(i<gpuList.size()-1)
            {
                mainWindow->server.at(index)->gpuInfo.append("\n\n");
            }
        }
    }
    else
    {
        mainWindow->server.at(index)->gpuInfo.clear();
        mainWindow->server.at(index)->gpuInfo.append("This server does not appear to have any GPU devices");
    }

    settingsBase->gpu.list = gpuList;
    if(!settingsBase->gpu.initialised)
    {
        settingsBase->gpu.active = (settingsBase->gpu.list.size() > 0);
        settingsBase->gpu.initialised = true;
    }

    qDebug(" - %s_gui(%i): available GPU devices:", binary.toStdString().c_str(), instance);
    for(int i=0; i<settingsBase->gpu.list.size(); i++)
    {
        qDebug("\t%s", settingsBase->gpu.list.at(i).at(0).toStdString().c_str());
    }

    createDeviceSubmenu();
}

/*!
 * \brief Sets GPU based on user selection.
 * \param[in] deviceID - GPU device identification number
 */
void GUI::setGpu(int deviceID)
{
    int requestedDevices = 0;
    int device = -1;
    QVector<int> devices;

    if(deviceID!=settingsBase->gpu.count) //GPU selected
    {
        //deselect CPU if previously selected
        if(deviceAct[settingsBase->gpu.count]->isChecked())
        {
            deviceAct[settingsBase->gpu.count]->setChecked(false);
        }

        if(settingsBase->gpu.multi) //multi-GPU selection
        {
            if(!deviceAct[deviceID]->isChecked())
            {
                deviceAct[deviceID]->setChecked(false);
                settingsBase->gpu.selected[deviceID] = -1;
            }
            else
            {
                deviceAct[deviceID]->setChecked(true);
                settingsBase->gpu.selected[deviceID] = deviceID;
            }

            //find out number of selcted devices
            for(int i=0; i<settingsBase->gpu.count; i++)
            {
                if(settingsBase->gpu.selected[i]!=-1)
                {
                    requestedDevices++;

                    //store the device id in case we run in a single-gpu mode
                    device = settingsBase->gpu.selected[i];
                    devices.push_back(settingsBase->gpu.selected[i]);
                }
            }
        }
        else //single-GPU selection
        {
            //deselect all
            for(int i=0; i<settingsBase->gpu.count; i++)
            {
                deviceAct[i]->setChecked(false);
                settingsBase->gpu.selected[i] = -1;
            }

            //check and initialise selected
            deviceAct[deviceID]->setChecked(true);
            settingsBase->gpu.selected[deviceID] = deviceID;
            device = settingsBase->gpu.selected[deviceID];
            requestedDevices = 1;
        }

        settingsBase->gpu.active = true;

        if(requestedDevices>1) //multi-GPU mode
        {
            interfaceBase->sendGpuIDs(devices);
        }
        else //single GPU mode
        {
            interfaceBase->sendGpuID(device);
        }
    }
    else //CPU selected
    {
        //deselect any previously-selected GPU
        for(int i=0; i<settingsBase->gpu.count; i++)
        {
            deviceAct[i]->setChecked(false);
            settingsBase->gpu.selected[i] = -1;
        }

        //select CPU
        settingsBase->gpu.selected[settingsBase->gpu.count] = settingsBase->gpu.count;
        settingsBase->gpu.active = false;
        deviceAct[deviceID]->setChecked(true);
        interfaceBase->sendCpuRequest();
    }
}

/*!
 * \brief Updates progress bar with the new value.
 * \param[in] currentProgress - current progress
 */
void GUI::updateProgressBar(int currentProgress)
{
    progress = currentProgress;

    //show the status bar only if the module is visible
    if(selected)
    {
        if(running && progress>-1)
        {
            mainWindow->showProgressBar();
            mainWindow->setProgressBarValue(progress);
        }
        else
        {
            mainWindow->hideProgressBar();
        }
    }

	#ifdef NTDDI_WIN7
	//add new client to the win7 progress bar
    if(progress > -1 && win7ProgressBarId==-1)
	{
		win7ProgressBarId = mainWindow->win7progressBar->addClient();
	}

    //the windows 7 overall progress needs to be updated even if module is not visible
    if(win7ProgressBarId != -1)
	{
		mainWindow->win7progressBar->setClientProgress(win7ProgressBarId, progress);
		mainWindow->win7progressBar->showOverallProgress();
	}
	#endif

	//reset progress if the module finished
    if(status==2)
	{
		progress = -1;
	}
}

/*!
 * \brief Updates progress bar clients.
 * \note This function is used on Windows 7 where a taskbar icon is used to show the overall
 * \note progress of all modules currently running. Therefore it is necessary to keep track of connected clients.
 */
void GUI::updateProgressBarClients()
{
    if(status==2)
    {
        #ifdef NTDDI_WIN7
        if(win7ProgressBarId != -1)
		{
			mainWindow->win7progressBar->removeClient(win7ProgressBarId);
			win7ProgressBarId = -1;
		}
        #endif
    }
}

/*!
 * \brief Closes module.
 */
void GUI::closeModule()
{
    //close module
    interfaceBase->sendQuitRequest();

    //stop the thread and close the ports
    interfaceBase->close();

    //close the module process
    module->close();

    //close the widget and delete this object
    close();
}

/*!
 * \brief Updates graphical user interface.
 * \note This is a virtual function that can be overridden.
 */
void GUI::updateGUI()
{
}

/*!
 * \brief Shows message box with the information about module.
 * \note This is a virtual function that can be overridden.
 */
void GUI::about()
{
}
