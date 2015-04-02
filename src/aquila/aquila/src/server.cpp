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

#include "server.h"

/*!
 * \brief Constructor.
 */
Server::Server(QString name, QTabWidget *parent, QString yarpVersion) : ui(new Ui::Server)
{
    ui->setupUi(this);
    parent->addTab(this, name);
    gpuInfo.append("GPU devices will appear once any modules are running on this server.");
    minimumYarpVersion = yarpVersion;
    portName = name;
    initialised = false;
    timeStep = 0;

    timer = new QTimer(this);
    timer->setInterval(1000);
    QObject::connect(timer, SIGNAL(timeout()), this, SLOT(updateInformation()));

    systemInfo = new QProcess(this);
    QObject::connect(systemInfo, SIGNAL(readyRead()), this, SLOT(readFromStdout()));
    QObject::connect(systemInfo, SIGNAL(finished(int, QProcess::ExitStatus)), this, SLOT(closeProcess()));

    QFont plotFont("Times", 10, QFont::Bold);
    ui->cpuPlot->setTitle("CPU Load");
    ui->cpuPlot->setTitleFont(plotFont);
    ui->cpuPlot->xAxis->setTickLabels(false);
    ui->cpuPlot->yAxis->setTickLabels(false);
    ui->cpuPlot->yAxis->setRange(0, 100);
    ui->cpuPlot->addGraph();
    ui->cpuPlot->graph()->setLineStyle(QCPGraph::lsLine);
    ui->cpuPlot->graph()->setPen(QPen(Qt::black));
    ui->cpuPlot->graph()->clearData();

    ui->memoryPlot->setTitle("Memory");
    ui->memoryPlot->setTitleFont(plotFont);
    ui->memoryPlot->xAxis->setTickLabels(false);
    ui->memoryPlot->yAxis->setTickLabels(false);
    ui->memoryPlot->addGraph();
    ui->memoryPlot->graph()->setLineStyle(QCPGraph::lsLine);
    ui->memoryPlot->graph()->setPen(QPen(Qt::black));
    ui->memoryPlot->graph()->clearData();
}

/*!
 * \brief Destructor.
 */
Server::~Server()
{
    delete ui;
}

void Server::updateInformation()
{
    if(!systemInfo->isOpen())
    {
        QStringList arguments = QStringList()<<"--on"<<portName<<"--sysinfo";
        systemInfo->start("yarprun", arguments);
    }
}

void Server::updatePlots()
{
    ui->cpuPlot->graph()->addData(timeStep+100, cpuUsage);
    ui->cpuPlot->xAxis->setRange(timeStep, timeStep+100);

    ui->memoryPlot->graph()->addData(timeStep+100, totalMemory-freeMemory);
    ui->memoryPlot->xAxis->setRange(timeStep, timeStep+100);
    ui->memoryPlot->yAxis->setRange(0, totalMemory);

    ui->cpuPlot->replot();
    ui->memoryPlot->replot();

    timeStep++;
}

void Server::closeProcess()
{
    systemInfo->close();
}

void Server::readFromStdout()
{
    info = (QString)systemInfo->readAllStandardOutput();

    //strip off unnecessary output
    info.remove(0, 21);
    info.chop(3);

    if(!initialised)
    {
        //get the user name from the remote machine
        QString userName = info.section(":", 7, 7);
        userName.remove(0, 1);
        userName.chop(16);

        //indication of old yarp version where server does not recognise --sysinfo flag
        if(userName=="")
        {
            info.clear();
            supportedYarpVersion = false;
        }
        else
        {
            supportedYarpVersion = true;
        }

        initialised = true;
    }

    //add modules to server information
    QString moduleInfo;
    moduleInfo.append("Aquila modules: ");
    for(int i=0; i<modules.size(); i++)
    {
        moduleInfo.append(modules.at(i) + QString(" "));
    }

    if(modules.size()==0)
    {
        moduleInfo.append("no modules detected");
    }

    info.prepend(moduleInfo + QString("\n\n"));

    //show warning message in case the server runs unsupported yarp version
    if(!supportedYarpVersion)
    {
        info.append("This system appears to be running an older version of YARP that is not fully supported.\nPlease make sure that you run at least version ");
        info.append(minimumYarpVersion);
        stopAutoUpdate();
    }

    //set the text
    ui->textEdit_generalInfo->setText(info);

    //retrieve cpu and memory usage
    QString tmp = info;
    tmp.remove(0, tmp.indexOf("Cpu load Ins.: ")+15);
    tmp.remove(tmp.indexOf("\n"), tmp.size());
    cpuUsage = tmp.toInt();
    tmp = info;
    tmp.remove(0, tmp.indexOf("Memory total : ")+15);
    tmp.remove(tmp.indexOf("M"), tmp.size());
    totalMemory = tmp.toInt();
    tmp = info;
    tmp.remove(0, tmp.indexOf("Memory free  : ")+15);
    tmp.remove(tmp.indexOf("M"), tmp.size());
    freeMemory = tmp.toInt();

    //update cpu and memory usage plots
    updatePlots();
}

void Server::startAutoUpdate(int interval)
{
    timer->setInterval(interval);
    timer->start();
}

void Server::stopAutoUpdate()
{
    timer->stop();
    timeStep = 0;
    ui->cpuPlot->graph()->clearData();
    ui->memoryPlot->graph()->clearData();
    ui->cpuPlot->replot();
    ui->memoryPlot->replot();
}
