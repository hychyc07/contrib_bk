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

#include <QShortcut>
#include "eraNetworkMonitor.h"
#include "ui_eraNetworkMonitor.h"

/*!
 * \brief Constructor.
 */
ERANetworkMonitor::ERANetworkMonitor() : QDialog(), ui(new Ui::ERANetworkMonitor)
{
    ui->setupUi(this);
    setWindowTitle("ERA Network Monitor");
    initPlot();

    QShortcut *closeDialog = new QShortcut(this);
    closeDialog->setKey(QKeySequence("CTRL+Q"));
    closeDialog->setContext(Qt::WindowShortcut);
    QObject::connect(closeDialog, SIGNAL(activated()), this, SLOT(close()));
}

/*!
 * \brief Destructor.
 */
ERANetworkMonitor::~ERANetworkMonitor()
{
    delete ui;
}

/*!
 * \brief Initialises plot.
 */
void ERANetworkMonitor::initPlot()
{
    ui->plot->setInteractions(QCustomPlot::iRangeDrag | QCustomPlot::iRangeZoom);
    ui->plot->setRangeDrag(Qt::Horizontal|Qt::Vertical);
    ui->plot->setRangeZoom(Qt::Horizontal|Qt::Vertical);
    ui->plot->setupFullAxesBox();

    ui->plot->xAxis->setLabel("Neuron Index");
    ui->plot->xAxis->setPadding(20);

    ui->plot->yAxis->setLabel("activity");
    ui->plot->yAxis->setPadding(15);
    ui->plot->yAxis->setNumberPrecision(6);

    ui->plot->addGraph();
    ui->plot->graph(0)->setLineStyle(QCPGraph::lsLine);
    ui->plot->graph(0)->setPen(QPen(Qt::blue));
    ui->plot->graph(0)->clearData();

    ui->plot->addGraph();
    ui->plot->graph(1)->setLineStyle(QCPGraph::lsLine);
    ui->plot->graph(1)->setPen(QPen(Qt::red));
    ui->plot->graph(1)->clearData();
   // updatePlot();
}

/*!
 * \brief Updates plot with new values.
 * \param[in] step - current step
 * \param[in] error - error at current step
 */
void ERANetworkMonitor::updatePlot(int size, double *activityRecieved, double *extInput)
{
    QVector<double> idx(size), activity(size), external(size);
    //make some test data
    for(int i=0; i<size; i++)
    {
        idx[i] = (double)i;
        if(activityRecieved[i] > -0.25 && activityRecieved[i] < 1.05)
        {
            activity[i] = activityRecieved[i];
        }
        else
        {
            activity[i] = 0.0;
        }
        if(extInput[i] > -0.25 && extInput[i] < 1.05)
        {
            external[i] = extInput[i];
        }
        else
        {
            external[i] = 0.0;
        }
    }

    //clear data
    //ui->plot->graph()->clearData();

    //plot the data
    ui->plot->graph(0)->setData(idx, activity);
    ui->plot->graph(1)->setData(idx, external);

    //set the axis
    ui->plot->xAxis->setRange(0, size);
    ui->plot->yAxis->setRange(-0.25, 1.05);
    ui->plot->replot();
}
