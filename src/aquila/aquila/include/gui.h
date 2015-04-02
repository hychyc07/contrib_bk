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

#ifndef GUI_H
#define GUI_H

#include "mainWindow.h"
#include "ui_mainWindow.h"

class Interface;
class Settings;

/*! Base class for graphical user interfaces */
class GUI : public QWidget
{
    Q_OBJECT

public:
    GUI(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString serverName, int instanceID, int tabID=-1);
    ~GUI();

    QString binary;
    QString title;
    QString server;

    int instance;
	bool moduleConnected;

private:
    QProcess *module;

    bool startModule(QString binaryName, QString serverName, int instanceID);
    bool timeoutConnectTo(QString binaryName, QString portName, int instanceID);

protected:
    MainWindow *mainWindow;
    Settings *settingsBase;
    Interface *interfaceBase;

    QSignalMapper *deviceSignalMapper;
    QMenu *fileMenu;
    QMenu *editMenu;
    QMenu *viewMenu;
    QMenu *helpMenu;
    QMenu *deviceSubMenu;
	QMenu *moduleMenu;
    QAction *startAction;
    QAction *stopAction;
    QAction *settingsAct;
    QAction *deviceAct[MAX_GPU_DEVICES];
    QAction *abortAct;
    QAction *quitAct;
    QAction *aboutAct[2];
    QString tag;

#ifdef NTDDI_WIN7
    int win7ProgressBarId;
#endif

    int progress;
    int status;
    bool selected;
    bool running;
    bool remoteMode;
    bool useModuleMenu;

    void createStartAction(QMenu* menu, QString actionName, QString statusTip, QString shortcut);
    void createStopAction(QMenu* menu, QString actionName, QString statusTip, QString shortcut);
	
private slots:
    void initialise(MainWindow *pMainWindow, QString binaryName, QString moduleTitle, QString serverName, int instanceID, int tabID);
    void remove(int tabID);
    void duplicate(int tabID);
    void closeModule();

public slots:
    void setGpu(int deviceID);
    void setGpuList(QVector<QStringList> gpuList);

protected slots:
    virtual void about();
	virtual void show();
    virtual void checkFocus(int tabID);
    virtual void statusChanged(int newStatus);
	virtual void createMenu();
    virtual void createFileMenu();
    virtual void createEditMenu();
    virtual void createViewMenu();
    virtual void createHelpMenu();
    virtual void createModuleMenu();
    virtual void createDeviceSubmenu();
    virtual void updateGUI();
    virtual void updateProgressBarClients();
    virtual void updateProgressBar(int currentProgress);

signals:
	void tabSelected();
	void tabDeselected();
    void stateReceived(int newState);
    void abortRequested();
};

#endif//GUI_H
