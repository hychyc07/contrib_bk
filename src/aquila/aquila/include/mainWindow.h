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

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QtGui>

#if defined _WIN64 || defined _WIN32
#include <Windows.h>
#endif

#ifdef NTDDI_WIN7
#include "win7progressBar.h"
#endif

#include "server.h"
#include "moduleGenerator.h"

#define MAX_MODULE_INSTANCES 20 //!< maximum number of module instances
#define MAX_SERVERS 10 //!< maximum number of servers
#define MAX_TIMEOUT_ATTEMPTS 50 //!< maximum number of attempts to ping an module
#define MAX_GPU_DEVICES 4 //!< maximum number of GPU devices per each server
#define MAX_TOOLS 20 //!< maximum number of tools that can be used in the tools menu

namespace Ui
{
    class MainWindow;
}

class SOM;
class MTRNN;
class Tracker;
class ESN;
class ERA;
class Altair;
//ModuleGenerator:write:0 - add new module class forward declaration

/*! Tabs used for module GUIs */
class TabWidget : public QTabWidget
{
    Q_OBJECT

private:
	QLabel *infoLabel;
	QHBoxLayout *infoLayout;

public:
    TabWidget();
    QTabBar* tabBar();
    QToolButton *addModuleButton;
    int newTabID;
    bool removed;

    void duplicateSelectedTab();
    void removeSelectedTab();
	void addInfoLabel();
	void removeInfoLabel();

signals:
    void tabRemoved(int id);
	void removeRequested(int id);
    void duplicateRequested(int id);
};

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent=0, QString aquilaVersion=0, QString hostName=0, QString yarpVersion=0);
    ~MainWindow();

    Ui::MainWindow *ui;
    ModuleGenerator moduleGenerator;
    TabWidget *tabWidget;
    QVector<Server*> server;

    #ifdef NTDDI_WIN7
    Win7ProgressBar *win7progressBar;
    #endif

    struct Module
    {
        QString serverID;
        QStringList modules;
    };

    void createViewMenu();
    void createToolsMenu();
    void createHelpMenu();
    void showProgressBar();
    void hideProgressBar();
    void addViewServerAction(QMenu *menu);
    void probeServers(bool useRemoteServers);
	void addLocalModules(QStringList requestedModule);
    void addRemoteModules(QVector<Module> requestedModules);
    void setProgressBarValue(int currentValue);
    int getServerIndex(QString serverName);
    bool progressBarVisible();
    bool addModule(QString module, QString server);
    QString getServerCpuType(QString serverName);

    #ifdef NTDDI_WIN7
    virtual bool winEvent(MSG *message, long *result);
    #endif

private:
    //pointers to module objects
    SOM *som[MAX_MODULE_INSTANCES];
    MTRNN *mtrnn[MAX_MODULE_INSTANCES];
    Tracker *tracker[MAX_MODULE_INSTANCES];
    ESN *esn[MAX_MODULE_INSTANCES];
    ERA *era[MAX_MODULE_INSTANCES];
	Altair *altair[MAX_MODULE_INSTANCES];
    //ModuleGenerator:write:1 - add new module array of pointers to instances

    QSignalMapper *toolsSignalMapper;
    QProgressBar *progressBar;
    QTabWidget *serverTabWidget;
    QComboBox *moduleCombobox;
    QComboBox *serverCombobox;
    QDialog *newTabDialog;
    QDialog *serverDialog;
    QProcess *serverProcess;
    QAction *viewServersAct;
    QAction *addTabAct;
    QAction *quitAct;
    QAction *toolsAct[MAX_TOOLS];
    QAction *aboutAct;
    QMenu *fileMenu;
    QMenu *viewMenu;
    QMenu *toolsMenu;
    QMenu *helpMenu;
    QString localHostName;
    QString minimumYarpVersion;
    QString program;
    QStringList servers;
    QStringList modules;
    QStringList arguments;

    bool serverProcessRunning;

    void initialiseTabWidget();
    void initialiseProgressBar();
    void initialiseModules();
    void initialiseServer();
    void initialiseServerViewer();
    void createFileMenu();
    int getModuleID(QString portPrefix);

private slots:
    void about();
    void aboutToQuit();
    void quit();
    void createMenu();
    void showMenu();
    void showNewTabDialog();
    void readServerOutput();
    void viewServers();
    void startTool(int toolID);
    void showContextMenu(const QPoint& cursorPosition);
    void serverComboboxChanged(int serverID);

public slots:
    bool addModule();

signals:
    void serverMonitorRequested();
    void quitting();
};

#endif//MAIN_WINDOW_H
