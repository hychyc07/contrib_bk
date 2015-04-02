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

#include "mainWindow.h"
#include "ui_mainWindow.h"
#include "som.h"
#include "mtrnn.h"
#include "tracker.h"
#include "esn.h"
#include "era.h"
#include "altair.h"
//ModuleGenerator:write:2 - add new module header

/*!
 * \brief Constructor.
 * \param[in] parent - MainWindow parent
 * \param[in] aquilaVersion - Aquila version
 * \param[in] hostName - local host name
 * \param[in] yarpVersion - minimum YARP version required
 */
MainWindow::MainWindow(QWidget *parent, QString aquilaVersion, QString hostName, QString yarpVersion) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle(aquilaVersion);

    localHostName = hostName;
    minimumYarpVersion = yarpVersion;

    createMenu();
    initialiseTabWidget();
    initialiseProgressBar();
    initialiseModules();
    initialiseServer();
    initialiseServerViewer();
}

/*!
 * \brief Destructor.
 */
MainWindow::~MainWindow()
{
    delete ui;
}

/*!
 * \brief Initialises module names.
 * \note These are the names of module executables, which are used when modules are being detected.
 */
void MainWindow::initialiseModules()
{
    //ModuleGenerator:write:3 - add new module name to the list of available modules
    modules<<"altair"<<"era"<<"esn"<<"mtrnn"<<"som"<<"tracker";
}

/*!
 * \brief Initialises tab widget and connects its signals.
 */
void MainWindow::initialiseTabWidget()
{
    tabWidget = new TabWidget();

    QHBoxLayout *centralWidgetLayout = new QHBoxLayout();
    centralWidgetLayout->setMargin(0);
    centralWidgetLayout->addWidget(tabWidget);
    ui->centralWidget->setLayout(centralWidgetLayout);

    QObject::connect(tabWidget->tabBar(), SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(showContextMenu(const QPoint&)));
    QObject::connect(tabWidget->addModuleButton, SIGNAL(clicked()), this, SLOT(showNewTabDialog()));
    QObject::connect(tabWidget, SIGNAL(currentChanged(int)), this, SLOT(showMenu()));
    QObject::connect(tabWidget, SIGNAL(tabRemoved(int)), this, SLOT(showMenu()));
}

/*!
 * \brief Initialises progress bar.
 * \note If necessary, this function will also initialise Windows 7 progress bar.
 */
void MainWindow::initialiseProgressBar()
{
    #ifdef NTDDI_WIN7
    win7progressBar = new Win7ProgressBar();
    win7progressBar->init(this->winId());
    #endif

    progressBar = new QProgressBar;
    statusBar()->addWidget(progressBar, 1);
    progressBar->setRange(0, 100);
    progressBar->setAlignment(Qt::AlignLeft);
    progressBar->hide();
}

/*!
 * \brief Creates menu.
 */
void MainWindow::createMenu()
{
    menuBar()->clear();
    createFileMenu();
    createViewMenu();
    createToolsMenu();
    createHelpMenu();
}

/*!
 * \brief Shows menu if no modules are loaded.
 */
void MainWindow::showMenu()
{
    if(tabWidget->count()==0)
    {
        createMenu();
    }
}

/*!
 * \brief Shows tab widget context menu.
 * \param[in] cursorPosition - cursonr position
 */
void MainWindow::showContextMenu(const QPoint& cursorPosition)
{
    QPoint globalPos = tabWidget->mapToGlobal(cursorPosition);
    QMenu myMenu;
    QAction *newTabAct = myMenu.addAction("New tab");
	myMenu.addSeparator();
	QAction *duplicateTabAct = myMenu.addAction("Duplicate");
	myMenu.addSeparator();
	QAction *closeTabAct = myMenu.addAction("Close tab");
    QAction* selectedItem = myMenu.exec(globalPos);

    int tabIndex = tabWidget->tabBar()->tabAt(cursorPosition);
    tabWidget->newTabID = tabIndex+1;

    if(selectedItem==newTabAct)
    {
		showNewTabDialog();
    }
    else if(selectedItem==closeTabAct)
    {
        tabWidget->removeSelectedTab();
    }
    else if(selectedItem==duplicateTabAct)
    {
        tabWidget->duplicateSelectedTab();
    }
}

#ifdef NTDDI_WIN7
/*!
 * \brief Processes winEvent message.
 * \return result of winEvent call
 */
bool MainWindow::winEvent(MSG *message, long *result)
{
    return win7progressBar->winEvent(message, result);
}
#endif

/*!
 * \brief Adds requested local modules.
 * \param[in] requestedModules - requested local modules
 */
void MainWindow::addLocalModules(QStringList requestedModules)
{
    for(int i=0; i<requestedModules.size(); i++)
    {

        if(!addModule(requestedModules.at(i), server.at(0)->portName))
        {
            qCritical(" - main_gui: could not initialise %s", requestedModules.at(i).toStdString().c_str());
        }
        else
        {
            qDebug(" - main_gui: %s was successfully initialised", requestedModules.at(i).toStdString().c_str());
        }
    }
}

/*!
 * \brief Adds requested remote modules.
 * \param[in] requestedModules - requested remote modules
 */
void MainWindow::addRemoteModules(QVector<Module> requestedModules)
{
    for(int i=0; i<requestedModules.size(); i++) //servers
    {
		bool serverFound = false;

		//check if the server was previously detected
        for(int s=0; s<server.size(); s++)
		{
            if(requestedModules.at(i).serverID == server.at(s)->id)
			{
				serverFound = true;
			}
		}

		//run modules 
		if(serverFound)
		{
			for(int j=0; j<requestedModules.at(i).modules.size(); j++) //modules
			{
                if(!addModule(requestedModules.at(i).modules.at(j), QString("/aquila/server/") + requestedModules.at(i).serverID)) //run module
				{
                    qCritical(" - main_gui: %s could not start on %s", requestedModules.at(i).modules.at(j).toStdString().c_str(), QString(QString("/aquila/server/") + requestedModules.at(i).serverID).toStdString().c_str());
				}
				else
				{
                    qDebug(" - main_gui: %s was successfully started on %s", requestedModules.at(i).modules.at(j).toStdString().c_str(), QString(QString("/aquila/server/") + requestedModules.at(i).serverID).toStdString().c_str());
				}
			}
		}
	}
}

/*!
 * \brief Adds a new module instance to a new tab.
 * \param[in] module - name of module executable
 * \param[in] server - name of server running module
 * \note developer: update this part with any new modules that you wish to add
 */
bool MainWindow::addModule(QString module, QString server)
{
	bool success = false;
    int instance = getModuleID(QString("/") + module + QString("/"));

	//remove the label showing information on how to add new module
	if(tabWidget->count()==0)
	{
		tabWidget->removeInfoLabel();
	}

    if(instance>-1)
    {
        if(module=="som")
        {
            som[instance] = new SOM(this, module, "SOM", server, instance, tabWidget->newTabID);
            success = som[instance]->moduleConnected;
        }
        else if(module=="mtrnn")
        {
            mtrnn[instance] = new MTRNN(this, module, "MTRNN", server, instance, tabWidget->newTabID);
            success = mtrnn[instance]->moduleConnected;
        }
        else if(module=="tracker")
        {
            tracker[instance] = new Tracker(this, module, "Tracker", server, instance, tabWidget->newTabID);
            success = tracker[instance]->moduleConnected;
        }
        else if(module=="esn")
        {
            esn[instance] = new ESN(this, module, "ESN", server, instance, tabWidget->newTabID);
            success = esn[instance]->moduleConnected;
        }
        else if(module=="era")
        {
            era[instance] = new ERA(this, module, "ERA", server, instance, tabWidget->newTabID);
            success = era[instance]->moduleConnected;
        }
		else if(module=="altair")
		{
			altair[instance] = new Altair(this, module, "Altair", server, instance, tabWidget->newTabID);
			success = altair[instance]->moduleConnected;
		}
        //ModuleGenerator:write:4 - add new module instance
	}

	return success;
}

/*!
 * \brief Adds a new module instance to a new tab.
 * \note This function is called once the addTab button in the new tab dialog was pressed.
 * \return true on success
 */
bool MainWindow::addModule()
{
    //add a new module instance based on user selection from the add tab dialog
    addModule(moduleCombobox->itemText(moduleCombobox->currentIndex()), server.at(serverCombobox->currentIndex())->portName);

    //set tab focus to the new tab
    tabWidget->setCurrentIndex(tabWidget->tabBar()->count()-1);

    return true;
}

/*!
 * \brief Shows a dialog for adding new tabs.
 * \note This function is called either when a user clicks the '+' button on the right side of the tab-bar
 * \note or when actions in the menu or tab context menu are trigerred by user selection.
 */
void MainWindow::showNewTabDialog()
{
    newTabDialog = new QDialog();
    newTabDialog->setMinimumSize(300, 110);
    newTabDialog->setMaximumSize(300, 110);
    newTabDialog->setWindowTitle("New tab");

    QShortcut *closeDialog = new QShortcut(newTabDialog);
    closeDialog->setKey(QKeySequence("CTRL+Q"));
    closeDialog->setContext(Qt::WindowShortcut);
    QObject::connect(closeDialog, SIGNAL(activated()), newTabDialog, SLOT(close()));

    QGridLayout *layout = new QGridLayout(newTabDialog);
    QGridLayout *buttonLayout = new QGridLayout();
    layout->addLayout(buttonLayout,3,1);

    QLabel *moduleLabel = new QLabel("Module");
    moduleCombobox = new QComboBox();
    moduleCombobox->addItems(server.at(0)->modules); //local modules

    QLabel *serverLabel = new QLabel("Run on");
    serverCombobox = new QComboBox();
    serverCombobox->addItems(servers);
    QObject::connect(serverCombobox, SIGNAL(currentIndexChanged(int)), this, SLOT(serverComboboxChanged(int)));

    QPushButton *addButton = new QPushButton("Add");
    QObject::connect(addButton, SIGNAL(clicked()), this, SLOT(addModule()));

    QPushButton *closeButton = new QPushButton("Close");
    QObject::connect(closeButton, SIGNAL(clicked()), newTabDialog, SLOT(close()));

    layout->addWidget(moduleLabel,0, 0);
    layout->addWidget(moduleCombobox,0, 1);
    layout->addWidget(serverLabel,2, 0);
    layout->addWidget(serverCombobox,2, 1);

    buttonLayout->addWidget(closeButton,0, 1);
    buttonLayout->addWidget(addButton,0, 2);

    newTabDialog->exec();
}

/*!
 * \brief Updates the list of modules running on a selected server.
 * \param[in] serverID - server id number
 */
void MainWindow::serverComboboxChanged(int serverID)
{
    moduleCombobox->clear();
    moduleCombobox->addItems(server.at(serverID)->modules);
}

/*!
 * \brief Updates the list of modules running on a selected server.
 * \param[in] portPrefix - module port prefix
 * \return int - the first unique module index available
 */
int MainWindow::getModuleID(QString portPrefix)
{
    int id = 0;
    bool success = false;
    QString portToCheck;

    //find available id
    while(id<MAX_MODULE_INSTANCES)
    {
        portToCheck = portPrefix+QString::number(id)+QString(":o");

        if(yarp::os::Network::exists(portToCheck.toStdString().c_str(), true)==false)
        {
            success = true;
            break;
        }
        id++;
    }

    if(success)
    {
        return id;
    }
    else
    {
        return -1;
    }
}

/*!
 * \brief Gets server CPU type
 * \return cpu - CPU type
 */
QString MainWindow::getServerCpuType(QString serverName)
{
	QString cpu("CPU");

    for(int i=0; i<server.size(); i++)
    {
        if(serverName==server.at(i)->portName)
        {
            cpu = server.at(i)->info;
            cpu.remove(0, cpu.indexOf("Processor model     :"));
            cpu.remove(cpu.indexOf("Processor model num"), cpu.size());
            cpu.replace("Processor model     :","");
            cpu.remove(cpu.indexOf("  "),cpu.size());
            cpu.replace("\n","");
        }
    }
	return cpu;
}

/*!
 * \brief Creates file menu.
 */
void MainWindow::createFileMenu()
{
	addTabAct = new QAction(tr("New &tab"), this);
    addTabAct->setStatusTip(tr("Add a new tab"));
	QObject::connect(addTabAct, SIGNAL(triggered()), this, SLOT(showNewTabDialog()));

    quitAct = new QAction(tr("&Quit"), this);
    quitAct->setShortcut(QKeySequence("CTRL+Q"));
    quitAct->setStatusTip(tr("Quit Aquila"));
    QObject::connect(quitAct, SIGNAL(triggered()), this, SLOT(quit()));

    fileMenu = menuBar()->addMenu(tr("&File"));
	fileMenu->addAction(addTabAct);
	fileMenu->addAction(quitAct);
}

/*!
 * \brief Creates view menu.
 */
void MainWindow::createViewMenu()
{
    viewMenu = menuBar()->addMenu(tr("&View"));
    addViewServerAction(viewMenu);
}

/*!
 * \brief Add view server actions.
 * \param[in] - menu - pointer to target menu
 */
void MainWindow::addViewServerAction(QMenu *menu)
{
    viewServersAct = new QAction("Servers", this);
    viewServersAct->setShortcut(QKeySequence("ALT+S"));
    viewServersAct->setStatusTip("Information about servers");
    QObject::connect(viewServersAct, SIGNAL(triggered()), this, SLOT(viewServers()));
    menu->addAction(viewServersAct);
}

/*!
 * \brief Creates tools menu.
 */
void MainWindow::createToolsMenu()
{
    toolsMenu = menuBar()->addMenu(tr("&Tools"));
    toolsSignalMapper = new QSignalMapper(toolsMenu);

    int toolID = 0;
    toolsAct[toolID] = new QAction(tr("&Module Generator"), this);
    toolsAct[toolID]->setShortcut(QKeySequence(tr("Shift+M")));
    toolsAct[toolID]->setStatusTip(tr("Start Module Generator"));
    toolsSignalMapper->setMapping(toolsAct[toolID], toolID);
    QObject::connect(toolsAct[toolID], SIGNAL(triggered()), toolsSignalMapper, SLOT(map()));
    toolID++;

    toolsAct[toolID] = new QAction(tr("&iCub Simulator"), this);
    toolsAct[toolID]->setShortcut(QKeySequence(tr("Shift+S")));
    toolsAct[toolID]->setStatusTip(tr("Start iCub simulator"));
    toolsSignalMapper->setMapping(toolsAct[toolID], toolID);
    QObject::connect(toolsAct[toolID], SIGNAL(triggered()), toolsSignalMapper, SLOT(map()));
    toolID++;

    toolsAct[toolID] = new QAction(tr("&Robot Motor GUI"), this);
    toolsAct[toolID]->setShortcut(QKeySequence(tr("Shift+R")));
    toolsAct[toolID]->setStatusTip(tr("Start robot motor GUI"));
    toolsSignalMapper->setMapping(toolsAct[toolID], toolID);
    QObject::connect(toolsAct[toolID], SIGNAL(triggered()), toolsSignalMapper, SLOT(map()));
    toolID++;

    toolsAct[toolID] = new QAction(tr("&iCub Interface GUI"), this);
    toolsAct[toolID]->setShortcut(QKeySequence(tr("Shift+I")));
    toolsAct[toolID]->setStatusTip(tr("Start iCub Interface GUI"));
    toolsSignalMapper->setMapping(toolsAct[toolID], toolID);
    QObject::connect(toolsAct[toolID], SIGNAL(triggered()), toolsSignalMapper, SLOT(map()));
    toolID++;

    toolsAct[toolID] = new QAction(tr("&CAN Loader"), this);
    toolsAct[toolID]->setShortcut(QKeySequence(tr("Shift+C")));
    toolsAct[toolID]->setStatusTip(tr("Start CAN loader"));
    toolsSignalMapper->setMapping(toolsAct[toolID], toolID);
    QObject::connect(toolsAct[toolID], SIGNAL(triggered()), toolsSignalMapper, SLOT(map()));
    toolID++;

    QObject::connect(toolsSignalMapper, SIGNAL(mapped(int)), this, SLOT(startTool(int)));

    for(int i=0; i<toolID; i++)
    {
        toolsMenu->addAction(toolsAct[i]);
    }
}

/*!
 * \brief Creates help menu.
 */
void MainWindow::createHelpMenu()
{
    aboutAct = new QAction(tr("&About"), this);
    aboutAct->setStatusTip(tr("About Aquila"));
    QObject::connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));
    helpMenu = menuBar()->addMenu(tr("&Help"));
    helpMenu->addAction(aboutAct);
}

/*!
 * \brief Initialises server on localhost.
 */
void MainWindow::initialiseServer()
{
    QString portName = QString("/aquila/server/") + localHostName;
    serverProcessRunning = false;

    if(!yarp::os::Network::exists(portName.toStdString().c_str(), false))
    {
        qDebug(" - main_gui: starting local server", portName.toStdString().c_str());

        serverProcess = new QProcess(this);
        serverProcess->setProcessChannelMode(QProcess::MergedChannels);
        QObject::connect(serverProcess, SIGNAL(readyRead()), this, SLOT(readServerOutput()));

        QString program = "yarprun";
        QStringList arguments = QStringList()<<"--server"<<portName.toStdString().c_str();

        serverProcess->start(program, arguments);
        serverProcess->waitForStarted();
        serverProcessRunning = true;
    }
}

/*!
 * \brief Reads standard output from server.
 */
void MainWindow::readServerOutput()
{
    while(serverProcess->canReadLine())
    {
        QByteArray data;
        data.append(serverProcess->readLine());

        if(data.size()>2) //filter out any empty lines or lines containing \n
        {
            if(data.endsWith("\n"))
            {
                data.chop(1);
            }

            if(data.contains("ERROR: no yarp network found"))
            {
                qDebug(" - server: %s",data.data());
                QMessageBox::critical(this, "server not found", "Please make sure that server is running and try again.");
                exit(1);
            }

            qDebug(" - server: %s",data.data());
        }
    }
}

/*!
 * \brief Probes available servers.
 * \param[in] useRemoteServers - if on, probe remote servers
 * \note This function detects all servers on a local network and are running Aquila server
 * \note named as /aquila/server/ID where ID can be anything between 0 and MAX_SERVERS.
 * \note Once a server is found, this function will probe it and will look for compiled modules.
 */
void MainWindow::probeServers(bool useRemoteServers)
{
    program = "yarprun";
    servers.clear();
    int id = 0;

    //get current time and use it to initialise random number generator
    QTime time = QTime::currentTime();
    qsrand((uint)time.msec());

    //generate random number used for module ping port starting index
    int portID = qrand()%1000;

    //try to probe all servers with the port suffix given by i
    for(int i=0; i<MAX_SERVERS; i++)
    {
        yarp::os::Time::delay(0.05);
        arguments.clear();

        if(id==0) //localhost
        {
            arguments<<"--on"<<QString("/aquila/server/") + localHostName<<"--sysinfo";
        }
        else //remotehost
        {
            arguments<<"--on"<<QString("/aquila/server/") + QString::number(i)<<"--sysinfo";
        }

        //probe server if its port name exists
        if(yarp::os::Network::exists(arguments.at(1).toStdString().c_str(), true))
        {
            qDebug(" - main_gui: server was found on %s", arguments.at(1).toStdString().c_str());

            //add new server and assign its port name
            Server *tmp = new Server(arguments.at(1), serverTabWidget, minimumYarpVersion);
            server.append(tmp);
            server.at(server.size()-1)->portName = arguments.at(1);

            //list of servers used by comboboxes
            servers.append(server.at(server.size()-1)->portName);

            //server id number
            server.at(server.size()-1)->id = QString::number(i);

            //find modules
            for(int j=0; j<modules.size(); j++)
            {
                qDebug(" - main_gui: looking for %s on %s", modules.at(j).toStdString().c_str(), servers.at(id).toStdString().c_str());

                //initialise variables
                int timeout = 0;
                bool success = false;

                //port name of the module used for checking its existence on servers
                QString portToCheck = QString("/") + modules.at(j) + QString("/ping/") + QString::number(portID);

                //try to launch an module
                QProcess probe;
                arguments.clear();
                arguments<<"--on"<<server.at(server.size()-1)->portName<<"--as"<<modules.at(j)+QString("Ping_") + QString::number(i)<<"--cmd"<<QString((modules.at(j)) + QString(" --ping ") + QString::number(portID)).toStdString().c_str();
                probe.start(program, arguments);

                //try to ping the module
                while(!success)
                {
                    qDebug(" - main_gui: probing %s module (trial %i out of %i)", modules.at(j).toStdString().c_str(), timeout, MAX_TIMEOUT_ATTEMPTS);

                    yarp::os::Time::delay(0.1);
                    if(!success)
                    {
                        if(yarp::os::Network::exists(portToCheck.toStdString().c_str(), true))
                        {
                            qDebug(" - main_gui: %s was found", modules.at(j).toStdString().c_str());
                            server.at(server.size()-1)->modules.append(modules.at(j));
                            success = true;
                        }
                        else
                        {
                            success = false;
                        }
                    }

                    timeout++;
                    if(timeout==MAX_TIMEOUT_ATTEMPTS)
                    {
                        qWarning(" - main_gui: %s was not found", modules.at(j).toStdString().c_str());
                        break;
                    }
                }

                portID++;
                probe.close();
            }

            server.at(server.size()-1)->updateInformation();
            id++;
        }
        else
        {
            if(i==0)
            {
                qCritical(" - main_gui: could not start server on localhost, make sure yarpserver is accessible");
            }
            else
            {
                qDebug(" - main_gui: server was not found on %s", arguments.at(1).toStdString().c_str());
            }
        }

        //do not try to probe remote servers
        if(!useRemoteServers)
        {
            break;
        }
    }
}

/*!
 * \brief Gets index of a server.
 * \param[in] serverName - the name of the server
 * \retrun index - server inder
 */
int MainWindow::getServerIndex(QString serverName)
{
    for(int i=0; i<server.size(); i++)
    {
        if(server.at(i)->portName == serverName)
        {
            return i;
        }
    }
    return -1;
}

/*!
 * \brief Initialises server viewer.
 */
void MainWindow::initialiseServerViewer()
{
    //initialise server dialog and add a layout to it
    serverDialog = new QDialog(this);
    QObject::connect(this, SIGNAL(serverMonitorRequested()), serverDialog, SLOT(show()));
    serverDialog->setMinimumSize(1000, 600);

    QShortcut *closeDialog = new QShortcut(serverDialog);
    closeDialog->setKey(QKeySequence("CTRL+Q"));
    closeDialog->setContext(Qt::WindowShortcut);
    QObject::connect(closeDialog, SIGNAL(activated()), serverDialog, SLOT(close()));

    QGridLayout *layout = new QGridLayout(serverDialog);
    serverDialog->setWindowTitle("Servers");

    //initialise tab widget and add it to the dialog
    serverTabWidget = new QTabWidget();
    layout->addWidget(serverTabWidget);
}

/*!
 * \brief View information about available servers.
 */
void MainWindow::viewServers()
{
    //add all available servers to the dialog where one server has its own tab
    for(int i=0; i<server.size(); i++)
    {
        server.at(i)->ui->textEdit_generalInfo->setText(server.at(i)->info);
        server.at(i)->ui->textEdit_gpuInfo->setText(server.at(i)->gpuInfo);
        server.at(i)->startAutoUpdate(250);
        QObject::connect(serverDialog, SIGNAL(rejected()), server.at(i), SLOT(stopAutoUpdate()));
    }

    //show the dialog
    emit serverMonitorRequested();
}

/*!
 * \brief Starts a tool as a new process.
 * \param[in] toolID - tool id
 */
void MainWindow::startTool(int toolID)
{
    program = "yarprun";
    arguments.clear();

    switch(toolID)
    {   
        case 0:
            moduleGenerator.show();
            break;

        case 1:
            arguments<<"--on"<<server.at(0)->portName.toStdString().c_str()<<"--as"<<"iCub_SIM"<<"--cmd"<<"iCub_SIM";
            break;

        case 2:
            arguments<<"--on"<<server.at(0)->portName.toStdString().c_str()<<"--as"<<"robotMotorGui"<<"--cmd"<<"robotMotorGui";
            break;

        case 3:
            arguments<<"--on"<<server.at(0)->portName.toStdString().c_str()<<"--as"<<"iCubInterfaceGuiClient"<<"--cmd"<<"iCubInterfaceGuiClient";
            break;

        case 4:
            arguments<<"--on"<<server.at(0)->portName.toStdString().c_str()<<"--as"<<"canLoader"<<"--cmd"<<"canLoader";
            break;
    }

    //keep process running even if Aquila quits
    if(!arguments.empty())
    {
        QProcess p;
        p.startDetached(program, arguments);
    }
}

/*!
 * \brief Hides progress bar.
 */
void MainWindow::hideProgressBar()
{
    progressBar->hide();
}

/*!
 * \brief Shows progress bar.
 */
void MainWindow::showProgressBar()
{
    progressBar->show();
}

/*!
 * \brief Sets progress bar value.
 * \param[in] currentValue - new progress bar value
 */
void MainWindow::setProgressBarValue(int currentValue)
{
    progressBar->setValue(currentValue);
}

/*!
 * \brief Shows a message box containing information about Aquila.
 */
void MainWindow::about()
{
    QMessageBox::about(this, windowTitle(), "<p>Aquila is an open-source project that was inspired by the recent advancements in supercomputing making use of CUDA-capable GPU devices."
                       "</p><p><b>Developers:</b><br><a href='http://www.martinpeniak.com' style='text-decoration: none'>Martin Peniak</a>"
                       "<br><a href='http://fostsvn.uopnet.plymouth.ac.uk/amorse/' style='text-decoration: none'>Anthony Morse</a>"
                       "</p><p><b>Resources:</b><br><a href='http://aquila.sourceforge.net' style='text-decoration: none'>Aquila project website</a><br>"
                       "<a href='http://dl.dropbox.com/u/81820/My%20Publications/IJCNN2011/IJCNN2011_Peniak_et_al.pdf' style='text-decoration: none'>Publication</a><br>"
                       "<a href='https://dl.dropbox.com/u/81820/Software/Aquila/Aquila.pdf' style='text-decoration: none'>Manual</a><br>"
                       "<a href='http://www.youtube.com/playlist?list=PL61ABCE73D3549D52&feature=plcp' style='text-decoration: none'>Videos</a></p>");
}

/*!
 * \brief Broadcasts quit signal to other modules and terminates server.
 */
void MainWindow::aboutToQuit()
{
    //let modules know that this application is quitting
    emit quitting();

    //terminate the process running server
    if(serverProcessRunning)
    {
        qDebug(" - main_gui: closing local server", server.at(0)->portName.toStdString().c_str());
        QString program = "yarprun";
        QStringList arguments = QStringList()<<"--on"<<server.at(0)->portName.toStdString().c_str()<<"--exit";
        QProcess p;
        p.start(program, arguments);
        p.waitForFinished();
        yarp::os::Time::delay(1.0);
        p.close();
        serverProcess->close();
    }
}

/*!
 * \brief Quits Aquila if the message box was accepted.
 */
void MainWindow::quit()
{
    if(!QMessageBox::question(this, tr("Exit"), tr("Do you really want to quit?"), tr("&Yes"), tr("&No"),QString::null, 0, 1 ))
    {
        this->close();
    }
}

/*!
 * \brief TabWidget constructor.
 */
TabWidget::TabWidget()
{
    newTabID = -1;
    removed = false;
	tabBar()->setContextMenuPolicy(Qt::CustomContextMenu);
    setGeometry(0, 0, 1031, 731);
    setMovable(true);
    setDocumentMode(true);
    addModuleButton = new QToolButton();
    addModuleButton->setText("+");
    addModuleButton->setShortcut(QKeySequence("Ctrl+T"));
    setCornerWidget(addModuleButton);
	addInfoLabel();
}

/*!
 * \brief Adds a label with information about how to add a new module.
 */
void TabWidget::addInfoLabel()
{
	#ifdef __APPLE__
	infoLabel = new QLabel("Press CMD+T to add a new module tab");
	#else
	infoLabel = new QLabel("Press CTRL+T to add a new module tab");
	#endif
    infoLabel->setAlignment(Qt::AlignCenter);
    infoLayout = new QHBoxLayout;
    infoLayout->addWidget(infoLabel);
	setLayout(infoLayout);
}

/*!
 * \brief Removes a label with information about how to add a new module.
 */
void TabWidget::removeInfoLabel()
{
	infoLabel->clear();
	infoLayout->removeWidget(infoLabel);
	delete infoLabel;
	delete infoLayout;
	repaint();
}

/*!
 * \brief Broadcasts a signal to other modules that user requested to duplicate a tab.
 * \note Only the module that matches the index passed to this signal will launch a new instance of itself.
 */
void TabWidget::duplicateSelectedTab()
{
    emit duplicateRequested(newTabID-1);
}

/*!
 * \brief Broadcasts a signal to other modules that user requested to remove a tab.
 * \note Only the module that matches the index passed to this signal will shotdown and remove itself from the tabwidget.
 */
void TabWidget::removeSelectedTab()
{
    removed = false;
	
	if(count()==1) //remove tab add the label showing information on how to add new module
	{
		emit removeRequested(newTabID-1);
		addInfoLabel();
	}
	else //remove the tab
	{
		emit removeRequested(newTabID-1);
	}
}

/*!
 * \brief Returns a pointer to the tab bar.
 * \return QTabWidget - pointer to the tab bar.
 */
QTabBar* TabWidget::tabBar()
{
    return QTabWidget::tabBar();
}
