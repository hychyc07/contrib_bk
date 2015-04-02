// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 francesco nori
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include "portScope.h"
#include <qcolor.h>

using namespace yarp::os;

QFont plotFont("Helvetica [Cronyx]", 8);
const int NUMBER_OF_COL = 20;
const int NUMBER_OF_LIN = 2;
QColor myRosy(188,143,143);      QColor mySand(244,164,96);    QColor myRed(255,0,0);       QColor myBrick(178,34,34);
QColor myAcqua(102, 205, 170);   QColor myGreen(124,252,0);    QColor mySpring(0,255,127);  QColor myOlive(107,142,35);
QColor myPaleGold(238,232,170);  QColor myYel(255,255,0);      QColor myGold(255,215,0);    QColor myDarkGold(184,134,11);
QColor myHPink(255,105,180);     QColor myPink(255,182,193);   QColor myViolet(176,48,96);  QColor myPlum(186,85,211);
QColor mySalmon(233,150,122);    QColor myOrange(255,165,0);   QColor myCoral(255,127,80);  QColor myTomato(255,99,71);

QPen coloredPens[NUMBER_OF_LIN][NUMBER_OF_COL] = {
    {QPen(myRosy, 1, Qt::SolidLine), QPen(mySand, 1, Qt::SolidLine), QPen(myRed, 1, Qt::SolidLine), QPen(myBrick, 1, Qt::SolidLine),
    QPen(myAcqua,1, Qt::SolidLine), QPen(myGreen, 1, Qt::SolidLine), QPen(mySpring, 1, Qt::SolidLine), QPen(myOlive, 1, Qt::SolidLine),
    QPen(myPaleGold, 1, Qt::SolidLine), QPen(myYel, 1, Qt::SolidLine), QPen(myGold, 1, Qt::SolidLine), QPen(myDarkGold, 1, Qt::SolidLine),
    QPen(myHPink, 1, Qt::SolidLine), QPen(myPink,1, Qt::SolidLine), QPen(myViolet, 1, Qt::SolidLine), QPen(myPlum, 1, Qt::SolidLine),
    QPen(mySalmon,1, Qt::SolidLine), QPen(myOrange,1, Qt::SolidLine), QPen(myCoral, 1, Qt::SolidLine), QPen(myTomato, 1, Qt::SolidLine)},
    {QPen(myRosy, 2, Qt::DashLine), QPen(mySand, 2, Qt::DashLine), QPen(myRed, 2, Qt::DashLine), QPen(myBrick, 2, Qt::DashLine),
    QPen(myAcqua,2, Qt::DashLine), QPen(myGreen, 2, Qt::DashLine), QPen(mySpring, 2, Qt::DashLine), QPen(myOlive, 2, Qt::DashLine),
    QPen(myPaleGold, 2, Qt::DashLine), QPen(myYel, 2, Qt::DashLine), QPen(myGold, 2, Qt::DashLine), QPen(myDarkGold, 2, Qt::DashLine),
    QPen(myHPink,2, Qt::DashLine), QPen(myPink,2, Qt::DashLine), QPen(myViolet, 2, Qt::DashLine), QPen(myPlum, 2, Qt::DashLine),
    QPen(mySalmon,2, Qt::DashLine), QPen(myOrange,2, Qt::DashLine), QPen(myCoral, 2, Qt::DashLine), QPen(myTomato, 2, Qt::DashLine)}
};

//#define ENABLE_REALTIME

//
//  Initialize main window
//
DataPlot::DataPlot(QWidget *parent):
    QwtPlot(parent),
    d_interval(0),
    numberAcquiredData(0),
    index(NULL),
    numberOfPlots(0),
    acquire(true),
    d_timerId(50)
{

    // Disable polygon clipping
    QwtPainter::setDeviceClipping(false);

    // We don't need the cache here
    canvas()->setPaintAttribute(QwtPlotCanvas::PaintCached, false);
    canvas()->setPaintAttribute(QwtPlotCanvas::PaintPacked, false);

#if QT_VERSION >= 0x040000
#ifdef Q_WS_X11
    /*
       Qt::WA_PaintOnScreen is only supported for X11, but leads
       to substantial bugs with Qt 4.2.x/Windows
     */
    canvas()->setAttribute(Qt::WA_PaintOnScreen, true);
#endif
#endif

    alignScales();
}

void DataPlot::setPorts(ConstString *portNameList, int nPorts)
{
    inputPorts = new yarp::os::BufferedPort<yarp::os::Bottle>[nPorts];
    for (int i = 0; i < nPorts; i++)
        {
            //open input port
            inputPorts[i].open(portNameList[i].c_str());     // Give it a name on the network.
        }
}

void DataPlot::setInputPortName(const char *inputName)
{
  QwtText plotTitle(inputName, QwtText::PlainText);
  plotTitle.setFont(plotFont);
  setTitle(plotTitle);
}

void DataPlot::setInputPortIndex(int **ind, int *size, int n)
{
    //if(VERBOSE) fprintf(stderr, "MESSAGE: Setting indexes\n");
    numberOfInputPorts = n;
    numberOfPlots = new int[n];
    index = new int*[n];
    d_y = new double**[n];
    for (int i = 0; i < numberOfInputPorts; i++)
        {
            numberOfPlots[i] = size[i];
            d_y[i] = new double*[numberOfPlots[i]];
            index[i] = new int[numberOfPlots[i]];
            for(int j = 0; j < numberOfPlots[i]; j++)
                {
                    index[i][j] = ind[i][j];
                    d_y[i][j] = new double[PLOT_SIZE];
                    //if(VERBOSE) fprintf(stderr, "INDEX: %d\n", index[i]);
                }
        }
    //if(VERBOSE) fprintf(stderr, "MESSAGE: Index set!\n");

}

void DataPlot::initSignalDimensions()
{
    if(VERBOSE) fprintf(stderr, "MESSAGE: Will now initialize the signal dimensions\n");
    //getting info on the connected port
    nonTimeBasedCurve = new QwtPlotCurve*[numberOfInputPorts];
    for (int k = 0; k < numberOfInputPorts; k++)
        {
            Bottle *b = NULL;
            int timeout = 0;
            const int maxTimeout = 1000;
            while(b==NULL && timeout < maxTimeout) 
                {
                    b = inputPorts[k].read(false);
                    Time::delay(0.005);
                    timeout++;
                }
            if (timeout==maxTimeout)
                {
                    if(VERBOSE) fprintf(stderr, "MESSAGE: Couldn't receive data. Going to put a zero! \n");
                    realTime = false;
                    //  Initialize data
                    for (int j = 0; j< numberOfPlots[k]; j++)
                        {
                            for (int i = 0; i < PLOT_SIZE; i++)
                                {
                                    d_x[i] = i;     // time axis
                                    //if(VERBOSE) fprintf(stderr, "MESSAGE: (%d, %d)\n", j, i);
                                    d_y[k][j][i] = 0;
                                }
                        }
                }
            else
                {
                    if(VERBOSE) fprintf(stderr, "MESSAGE: Will now try real time!\n");
                    inputVectorSize = b->size();
                    Stamp stmp;
                    inputPorts[k].getEnvelope(stmp);
                    if (stmp.isValid())
                        {     
                            if(VERBOSE) fprintf(stderr, "MESSAGE: will use real time!\n");
                            realTime = true;
                            initialTime = stmp.getTime();
                        }
                
                    //  Initialize data
                    for (int j = 0; j< numberOfPlots[k]; j++)
                        {
                            if (b->size()-1 < index[k][j])
                                if(VERBOSE) fprintf(stderr, "WARNING: will plot some zeros since the accessed index exceed the input vector dimensions!\n");
                            for (int i = 0; i < PLOT_SIZE; i++)
                                {
                                    d_x[i] = i;     // time axis
#ifdef ENABLE_REALTIME
                                    if (realTime)
                                        d_x_real_time[i] = i;
#endif
                                    //if(VERBOSE) fprintf(stderr, "MESSAGE: (%d, %d)\n", j, i);
                                    d_y[k][j][i] = 0;
                                }
                        }
                }

            //if(VERBOSE) fprintf(stderr, "MESSAGE: initializing plot datas!\n");
            // Assign a title
            insertLegend(new QwtLegend(), QwtPlot::BottomLegend);

            nonTimeBasedCurve[k] = new QwtPlotCurve[numberOfPlots[k]];
            for(int i=0; i < numberOfPlots[k]; i++)
                {
                    //Set title
                    char cTitle[256];
                    sprintf(cTitle, "Data(%d)", index[k][i]);
                    QwtText curveTitle(cTitle, QwtText::PlainText);
                    curveTitle.setFont(plotFont);
                    nonTimeBasedCurve[k][i].setTitle(curveTitle);

                    //if(VERBOSE) fprintf(stderr, "MESSAGE: Will now initialize the plot %d\n", index[i]);
                    // Insert new curves
                    nonTimeBasedCurve[k][i].attach(this);

                    // Set curve styles
                    nonTimeBasedCurve[k][i].setPen(coloredPens[k%NUMBER_OF_LIN][i%NUMBER_OF_COL]);

                    // Attach (don't copy) data. Both curves use the same x array.
                    nonTimeBasedCurve[k][i].setRawData(d_x, d_y[k][i], PLOT_SIZE);
                }

            // Axis 
            QwtText axisTitle("Time/seconds");
            axisTitle.setFont(plotFont);
            setAxisTitle(QwtPlot::xBottom, axisTitle);
            setAxisScale(QwtPlot::xBottom, 0, 100);
            setAxisFont(QwtPlot::xBottom, plotFont);

            setAxisTitle(QwtPlot::yLeft, "Values");
            //setAxisScale(QwtPlot::yLeft, -1.5, 1.5);
            setAxisAutoScale(QwtPlot::yLeft);
            setAxisAutoScale(QwtPlot::xBottom);
            setAxisFont(QwtPlot::yLeft, plotFont);

            setTimerInterval(50.0); 
        }
    //if(VERBOSE) fprintf(stderr, "MESSAGE: plot intialized!\n");
}

DataPlot::~DataPlot()
{
    if(VERBOSE) fprintf(stderr, "MESSAGE: Deleting index/indeces\n");
    for (int k = 0; k < numberOfInputPorts; k++)
        delete[] index[k];
    if(VERBOSE) fprintf(stderr, "MESSAGE: Deleting data plot vectors\n");
    for (int k = 0; k < numberOfInputPorts; k++)
        for (int j = 0; j < numberOfPlots[k]; j++)
            delete[] d_y[k][j];
    if(VERBOSE) fprintf(stderr, "MESSAGE: Deleting curves\n");
    for (int k = 0; k < numberOfInputPorts; k++)
        delete[] nonTimeBasedCurve[k];
    if(VERBOSE) fprintf(stderr, "MESSAGE: Closing the port/s\n");
    for (int k = 0; k < numberOfInputPorts; k++)
        {
            inputPorts[k].interrupt();
            inputPorts[k].close();
        }
    if(VERBOSE) fprintf(stderr, "MESSAGE: Deleting numberOfPlots\n");
    delete[] numberOfPlots;
    if(VERBOSE) fprintf(stderr, "MESSAGE: DataPlot successfully closed\n");
}

//
//  Set a plain canvas frame and align the scales to it
//
void DataPlot::alignScales()
{
    // The code below shows how to align the scales to
    // the canvas frame, but is also a good example demonstrating
    // why the spreaded API needs polishing.

    canvas()->setFrameStyle(QFrame::Box | QFrame::Plain );
    canvas()->setLineWidth(1);

    for ( int i = 0; i < QwtPlot::axisCnt; i++ )
    {
        QwtScaleWidget *scaleWidget = (QwtScaleWidget *)axisWidget(i);
        if ( scaleWidget )
            scaleWidget->setMargin(0);

        QwtScaleDraw *scaleDraw = (QwtScaleDraw *)axisScaleDraw(i);
        if ( scaleDraw )
            scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
    }
}

void DataPlot::setTimerInterval(double ms)
{
    d_interval = qRound(ms);

    if ( d_timerId >= 0 )
    {
        killTimer(d_timerId);
        d_timerId = -1;
    }
    if (d_interval >= 0 )
        d_timerId = startTimer(d_interval);
}

void DataPlot::toggleAcquire()
{
  if (acquire)
    acquire = false;
  else
    acquire = true;
}

//  Generate new values 
void DataPlot::timerEvent(QTimerEvent *)
{
    for (int k = 0; k < numberOfInputPorts; k++)
        {
            int timeout = 0;
            int maxTimeout = 2;
            Bottle *b = NULL;
            
            while(b==NULL && timeout < maxTimeout) 
                {
                    b = inputPorts[k].read(false);
                    Time::delay(0.005);
                    timeout++;
                }
            if (timeout==maxTimeout)
                {
                    if(VERBOSE) fprintf(stderr, "MESSAGE: Couldn't receive data. Going to put a zero! \n");
                    for(int i = 0; i < numberOfPlots[k]; i++)
                        {
                            d_y[k][i][PLOT_SIZE - 1] = 0;
                            for ( int j = 0; j < PLOT_SIZE - 1; j++ )
                                {
                                    d_y[k][i][j] = d_y[k][i][j+1];
                                }
                        }
                }
            else
                {
                    for(int i = 0; i < numberOfPlots[k]; i++)
                        {

                            Stamp stmp;
                            inputPorts[k].getEnvelope(stmp);
#ifdef ENABLE_REALTIME
                            if (stmp.isValid()) 
                                d_x_real_time[PLOT_SIZE - 1] = (stmp.getTime() - initialTime);                            
                            
                            for ( int j = 0; j < PLOT_SIZE - 1; j++ )
                                {
                                    if(realTime)
                                        d_x_real_time[j] =  d_x_real_time[j+1];
                                }
#endif
                            if (b==NULL)
                                d_y[k][i][PLOT_SIZE - 1] = 0;
                            else
                                if (b->size()-1 >= index[k][i])
                                    {
                                        d_y[k][i][PLOT_SIZE - 1] = b->get(index[k][i]).asDouble();
                                        //if(VERBOSE) fprintf(stderr, "MESSAGE: Getting from port %d the index %d\n", k, index[k][i]);
                                    }
                            // y moves from left to right:
                            // Shift y array right and assign new value to y[0].
                            
                            for ( int j = 0; j < PLOT_SIZE - 1; j++ )
                                {
                                    d_y[k][i][j] = d_y[k][i][j+1];
                                }
                            
                            // update the display
                            //setAxisScale(QwtPlot::yLeft, min, max);
#ifdef ENABLE_REALTIME                            
                            if(numberAcquiredData==PLOT_SIZE && realTime)
                                {
                                    if(VERBOSE) fprintf(stderr, "MESSAGE: switching to real time\n");
                                    
                                    QwtPlotCurve *timeBasedCurve = new QwtPlotCurve("Data");
                                    timeBasedCurve->attach(this);
                                    timeBasedCurve->setRawData(d_x_real_time, d_y[k][i], PLOT_SIZE);
                                    timeBasedCurve->setPen(coloredPens[k%NUMBER_OF_LIN][i%NUMBER_OF_COL]);
                                    nonTimeBasedCurve[k][i].attach(NULL);
                                    
                                    //Set title
                                    char cTitle[256];
                                    sprintf(cTitle, "Data(%d)", index[k][i]);
                                    QwtText curveTitle(cTitle, QwtText::PlainText);
                                    curveTitle.setFont(plotFont);
                                    timeBasedCurve->setTitle(curveTitle); 
                                }
#endif
                        }
                }
        }
    if (acquire)
        replot();

    numberAcquiredData++;
    //if(VERBOSE) fprintf(stderr, "Number of acquired data is %d\n", numberAcquiredData);
    //QSize plotSize= this->sizeHint();
    //if(VERBOSE) fprintf(stderr, "Hint is: hInt=%d, vInt=%d\n", plotSize.height(), plotSize.width()); 

	static double before = Time::now();	
	double now = Time::now();
	static double meanEffectiveTime = 0;

	double currentEffectiveTime = (now - before)*1000.0;
	if (numberAcquiredData >= 2)
		meanEffectiveTime = (meanEffectiveTime*(numberAcquiredData-2) + currentEffectiveTime)/(numberAcquiredData-1);
	//if(VERBOSE) fprintf(stderr, "Iteration %d: Current time is %f and mean is %f\n", numberAcquiredData, currentEffectiveTime, meanEffectiveTime);	
	if (currentEffectiveTime*0.5 > d_interval)
	{
		if(VERBOSE) fprintf(stderr, "Real Timer is %f but I was supposed to run at %d ms. Mean is: %f \n", currentEffectiveTime, d_interval, meanEffectiveTime);	
	    if(VERBOSE) fprintf(stderr, "You should slow down to %d ms \n", (int) (meanEffectiveTime * 2));	
        //setTimerInterval((int) (meanEffectiveTime * 2));
	}

	before = now;

}
