// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2006 Francesco Nori
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.
 *
 */

/**
 *
 * \defgroup icub_portScope portScope
 * @ingroup icub_contrib_guis
 * A simple graphical user interface for visualizing the numerical content of a
 * a port.
 *
 * \image html portScope.jpg
 * \image latex portScope.eps "A yarp portScope running on Linux" width=15cm
 *
 * \section depend_sec Dependencies
 * 
 * This module depends on <a href="http://qwt.sourceforge.net/"> QWT </A> a library
 * for plotting graphs and similar guis. This graphical library
 * can be easily installed on linux with the following command:
 * \code
 * sudo apt-get install libqwt5-qt3-dev
 * \endcode
 * On windows, installation is a little bit more complicated and requires qmake
 * in order to compile the library. An easy way to compile the windows library consists in: 
 * 
 * - downloading the source files <a href="http://sourceforge.net/projects/qwt/files/qwt/5.2.1/qwt-5.2.1.zip/download"> here </A>;
 * - unzipping the file in a preferred directory;
 * - defining the environmental variable QWT_DIR pointing at the QWT folder;
 * - following the INSTALL instructions which practically rely on the following steps:
 * \code
 * set QMAKESPEC=win32-msvc
 * qmake qwt.pro
 * nmake
 * \endcode
 * Compiling the library can be tricky on 64 bit machines; in this cases you should use the proper nmake command 
 * (e.g. Visual Studio has two different command prompts if you want to use 64 or 32 bit nmake).
 * Once the library has been successfully compiled
 * the portScope module can be compiled defining a suitable environmental variables
 * Qwt_DIR pointing at the QWT code location. The library path (e.g. Qwt_DIR/lib)
 * needs to be added to the system PATH.
 * 
 * \section intro_sec Description
 * This simple graphical user interface allows to visualize on a plot the content
 * of a yarp port. The input port is assumed to contain a vector of numbers. 
 * The visualization has four main features:
 *
 * - possibility of selecting one or more indeces to be plotted.
 * - possibility of plotting the content of different ports on the same plot.
 * - possibility of creating multiple plots on the same window (sort of Matlab subplot function).
 * - when available uses the timestamp for the horizontal axes (time=0 is defined when launching the GUI).
 *
 * \section usage_sec How to initialize the portScope
 * The initialization of the GUI uses the resource finder policy. Therefore,
 * the module can be either initialized via file or via command line. In particular,
 * the easiest way to initilize the module is the following:
 * \code
 * portScope --local /myPortScope:i --index 0
 * \endcode
 * This command will open a port named /myPortScope:i00 which will wait for incoming
 * data. Once the inconming data will be supplied, the module will start plotting 
 * the first element of this data in the plot (since the option --index 0 was) 
 * specified. Supplied data are always interpreted as double. Another usage of 
 * the module is the following:
 * \code
 * portScope --local /myPortScope:i --index "(0 1)"
 * \endcode
 * which will behave exactly as the previous example but the first and the second
 * data element will be plotted in different colors.
 * If you want to plot the content of a port (e.g. /remote/port:0) which is already
 * existing you can use:
 * \code
 * portScope --remote /remote/port:0 --index "(0 1)"
 * \endcode
 * and in this case the portScope will open a default port called /portScope/vector:i00.
 * If you want to plot the content of two ports (e.g. /remote/port:0 and /remote/port:1) 
 * which are already existing you can use (notation here is unfortunately quite unintuitive,
 * soon or later I will make it easier):
 * \code
 * portScope --remote "((/remote/port:0 /remote/port:1))" --index "(((0 1) (2 3)))"
 * \endcode
 * and in this case the portScope will open two default ports called /input0/portScope/vector:i00
 * and /input1/portScope/vector:i00. The first and second element contained in /remote/port:0
 * will be plotted together with the third and fourth element in /remote/port:1.
 * Finally, complex plots can be created by modifying the notation above. 
 * In particular if we want to create a matrix of plots with different inputs,
 * we can use the following (even more complicated) notation:
 * \code
 * portScope --rows 2 --cols 1 --remote "((/remote/port:0 /remote/port:1) (/remote/port:3 /remote/port:4))" --index "(((0 1) (2 3)) ((4 5) (6 7)))"
 * \endcode
 * In this case a matrix of plots composed of two rows and one column (practically a vector)
 * will be created. The plot in position (1,1) will plot the first and second element in /remote/port:0
 * together with the third and fourth element in /remote/port:1. The plot in position (2,1) will plot
 * the fifth and sixth element in /remote/port:3 together with the seventh and eighth element in /remote/port:4. 
 * In this case the following ports will be opened /input0/portScope/vector:i00, /input1/portScope/vector:i00
 * /input0/portScope/vector:i10, /input1/portScope/vector:i10.
 **/

//YARP
#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/os/ResourceFinder.h>

#include <qapplication.h>
#include <qmainwindow.h>
#include <qwt_counter.h>
#include <qtoolbar.h>
#include <qlabel.h>
#include <qlayout.h>
#include <qgrid.h>
#include <qlcdnumber.h>
#include <qpushbutton.h>
#include <qsize.h>

#include "portScope.h"
bool VERBOSE;


using namespace yarp::os;

class MainWindow: public QMainWindow
{
public:
    DataPlot **plot;
    ResourceFinder *resFind;
    int nRows;
    int nCols;
    int nPlots;
    int *nPortInputsInPlots;
  
    MainWindow(ResourceFinder *rf)
    {

        //if (VERBOSE) fprintf(stderr, "Passing RF by address ...");
        resFind = rf;
        //if (VERBOSE) fprintf(stderr, "done \n");

        Property options;
        options.fromString(resFind->toString());

        Value &robot  = options.find("robot");
        Value &part   = options.find("part");
        Value &rows   = options.find("rows");
        Value &cols   = options.find("cols");

        if (!options.check("robot"))
            printf("Missing --robot option. Using icub.\n");
        if (!options.check("part"))
            printf("Missing --part option. Using head.\n");    
        if (!options.check("local"))
            printf("Missing --name option. Using /portScope/vector:i.\n");
        if (!options.check("remote"))
            printf("Missing --remote option. Will wait for the connection...\n");
        if (!options.check("rows"))
            printf("Missing --rows option. Disabling subplotting.\n");

        //if (VERBOSE) fprintf(stderr, "Start plotting the GUI\n");
        QToolBar *toolBar = new QToolBar(this);
        toolBar->setFixedHeight(80);
    
#if QT_VERSION < 0x040000
        setDockEnabled(TornOff, true);
        setRightJustification(true);
#else
        toolBar->setAllowedAreas(Qt::TopToolBarArea | Qt::BottomToolBarArea);
#endif
        QWidget *hBox = new QWidget(toolBar);
        QLabel *label = new QLabel("Timer Interval", hBox);
        QwtCounter *counter = new QwtCounter(hBox);

        QLabel *labelActions = new QLabel("Actions", hBox);
        QPushButton *toggleAcquireButton = new QPushButton("stop", hBox, "stop");
        counter->setRange(-1.0, 1000.0, 1.0);
    
        QHBoxLayout *layout = new QHBoxLayout(hBox);
        layout->addWidget(label);
        layout->addWidget(counter);
        layout->addWidget(labelActions);
        layout->addWidget(toggleAcquireButton);
        layout->addWidget(new QWidget(hBox), 10); // spacer);
    
#if QT_VERSION >= 0x040000
        toolBar->addWidget(hBox);
#endif
        addToolBar(toolBar);
    
        if (options.check("rows"))
            nRows = rows.asInt();
        else
            nRows = 1;

        if (options.check("cols"))
            nCols = cols.asInt();
        else
            nCols = nRows;

        nPlots  = nRows*nCols;
        nPortInputsInPlots = new int[nPlots];
        QGrid *grid = new QGrid( nRows, Qt::Vertical,  this );
        
        plot = new DataPlot*[nPlots];
        setCentralWidget(grid);

        for( int r = 0 ; r < nRows ; r++ )
            {
                for( int c = 0 ; c < nCols ; c++ )
                    {	  
                        char rcS[256];
                        sprintf(rcS, "%d%d", r, c);
                        ConstString rS(rcS);
                        ConstString local;
                        local = resFind->find("local").toString();
                        //local port name
                        local = local + rcS;

                        //dataPlot creation
                        plot[r+nRows*c] = new DataPlot(grid);
	    
                        //dataPlot set plot to read from
                        setInputPort(resFind, r+nRows*c, local);

                        //dataPlot set indeces of ports to plot
                        setIndexMask(resFind, r+nRows*c, nPortInputsInPlots[r+nRows*c]);

                    }
            }
	   
        for( int r = 0 ; r < nRows ; r++ )
            {
                for( int c = 0 ; c < nCols ; c++ )
                    { 
                        //stop acquisition button
                        connect( toggleAcquireButton, SIGNAL(clicked()), plot[r+nRows*c], SLOT(toggleAcquire()) );
                        plot[r+nRows*c]->initSignalDimensions();
                        //counter button
                        connect(counter, SIGNAL(valueChanged(double)),
                                plot[r+nRows*c], SLOT(setTimerInterval(double)) );
	  
                        counter->setValue(50.0);
	 
                        QSizePolicy sizePolicy2(QSizePolicy::Maximum, QSizePolicy::Maximum);
                        grid->setSizePolicy(sizePolicy2);

                        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
                        sizePolicy.setHeightForWidth(plot[r+nRows*c]->sizePolicy().hasHeightForWidth());
                        plot[r+nRows*c]->setSizePolicy(sizePolicy);

                        //QSize gridSize= grid->sizeHint();
                        //gridSize.setWidth(500);
                        //gridSize.setHeight(500);
                        plot[r+nRows*c]->setMinimumWidth(10);
                        plot[r+nRows*c]->setMaximumWidth(800);
                        plot[r+nRows*c]->setMinimumHeight(10);
                        plot[r+nRows*c]->setMaximumHeight(800);
                        //if (VERBOSE) fprintf(stderr, "Grid max is: hInt=%d, vInt=%d\n", grid->maximumHeight(), grid->maximumWidth()); 
                        //if (VERBOSE) fprintf(stderr, "Plot max is: hInt=%d, vInt=%d\n", plot[r+nRows*c]->maximumHeight(), plot[r+nRows*c]->maximumWidth()); 
                        //QSize plotSize= plot[r+nRows*c]->sizeHint();
                        //if (VERBOSE) fprintf(stderr, "Plot Hint is: hInt=%d, vInt=%d\n", plotSize.height(), plotSize.width()); 
                    }
            }
    }

    void setInputPort(ResourceFinder *rf, int index, ConstString local)
    {
        ConstString **remote;
        ConstString *localTmp;
        bool connectionDone = false;
        remote = new ConstString*[nPlots];

        //local/remote port connection
        if(rf->find("remote").isString())
            {
                nPortInputsInPlots[index]  = 1;
                remote[index] = new ConstString[nPortInputsInPlots[index]];
                remote[index][0] = resFind->find("remote").toString();
                plot[index]->setInputPortName(remote[index][0].c_str());
                //connect
                plot[index]->setPorts(&local, nPortInputsInPlots[index]);
                yarpConnectRemoteLocal(remote[index][0], local);
                connectionDone = true;
            }
        else if(rf->find("remote").isList())
            {
                Bottle *rrBot;
                //if (VERBOSE) fprintf(stderr, "MESSAGE: Option remote is a list\n");
                rrBot = resFind->find("remote").asList();
                int listSize = rrBot->size();
                if (rrBot->get(index).isString() && listSize==1)
                    {
                        nPortInputsInPlots[index]  = 1;
                        remote[index] = new ConstString[nPortInputsInPlots[index]];
                        remote[index][0] = rrBot->get(0).toString();
                        plot[index]->setInputPortName(remote[index][0].c_str());
                        //connect
                        plot[index]->setPorts(&local, nPortInputsInPlots[index]);
                        yarpConnectRemoteLocal(remote[index][0], local);
                        connectionDone = true;
                    }
                if (rrBot->get(index).isString() && listSize==nPlots)
                    {
                        nPortInputsInPlots[index]  = 1;
                        remote[index] = new ConstString[nPortInputsInPlots[index]];
                        remote[index][0] = rrBot->get(index).toString();
                        plot[index]->setInputPortName(remote[index][0].c_str());
                        //connect
                        plot[index]->setPorts(&local, nPortInputsInPlots[index]);
                        yarpConnectRemoteLocal(remote[index][0], local);
                        connectionDone = true;
                    }
                if (rrBot->get(index).isList() && listSize==nPlots)
                    {
                        //if (VERBOSE) fprintf(stderr, "MESSAGE: Getting a double list of remote ports! \n");
                        Bottle *rrrBot;
                        rrrBot = rrBot->get(index).asList();
                        nPortInputsInPlots[index]  = rrrBot->size();
                        remote[index] = new ConstString[nPortInputsInPlots[index]];
                        localTmp = new ConstString[nPortInputsInPlots[index]];
                        for (int j = 0; j < nPortInputsInPlots[index]; j++)
                            {
                                char stringN[64];
                                sprintf(stringN, "/input%d", j);
                                ConstString sN(stringN);

                                remote[index][j] = rrrBot->get(j).toString();
                                localTmp[j] = sN;
                                localTmp[j] = localTmp[j] + local;
                            }
                        ConstString sumRemote = remote[index][0];
                        sumRemote = sumRemote + " ";
                        for (int j = 1; j < nPortInputsInPlots[index]; j++)
                            {
                                sumRemote = sumRemote + remote[index][j];
                                sumRemote = sumRemote + " ";
                            }
                        plot[index]->setInputPortName(sumRemote.c_str());
                        //connect
                        plot[index]->setPorts(localTmp, nPortInputsInPlots[index]);
                        for (int j = 0; j < nPortInputsInPlots[index]; j++)
                            yarpConnectRemoteLocal(remote[index][j], localTmp[j]);
                        connectionDone = true;
                    }
                
            }
        if (!connectionDone)
            fprintf(stderr, "WARNING: no input port connected. Waiting explicit connection from user.\n");
    }

    void setIndexMask(ResourceFinder *rf, int index, int numberOfInputs)
    {
        int **indexToPlot = NULL;
        indexToPlot = new int*[numberOfInputs];
        int *sizeIndexToPlot = NULL;
        sizeIndexToPlot = new int[numberOfInputs];
        if (rf->find("index").isList())
            {
                Bottle *rrBot;
                //if (VERBOSE) fprintf(stderr, "Option remote is a list\n");
                rrBot = resFind->find("index").asList();
                if (rrBot->size() == nRows*nCols)
                    {
                        if (rrBot->get(index).isInt())
                            {
                                if (numberOfInputs==1)
                                    {
                                        indexToPlot[0] = new int;
                                        indexToPlot[0][0] = rrBot->get(index).asInt();
                                        sizeIndexToPlot[0] = 1;
                                    }
                                else
                                    fprintf(stderr, "ERROR: a single index was supplied but more than one input port was given!\n");
                            }
                        else if (rrBot->get(index).isList())
                            {
                                Bottle *rrrBot;
                                rrrBot = rrBot->get(index).asList();
                                if(rrrBot->get(0).isInt() && numberOfInputs==1)
                                    {
                                        indexToPlot[0] = new int[rrrBot->size()];
                                        sizeIndexToPlot[0] = rrrBot->size();
                                    }
                                
                                for(int j = 0; j < rrrBot->size(); j++)
                                    {
                                        if(rrrBot->get(j).isInt())
                                            {
                                                if (numberOfInputs==1)
                                                    {
                                                        indexToPlot[0][j] = rrrBot->get(j).asInt();
                                                        //if (VERBOSE) fprintf(stderr, "MESSAGE: getting a list of indeces for a plot with a single input. Index(%d)=%d\n", j, indexToPlot[0][j]);
                                                    }
                                            }
                                        else if (rrrBot->get(j).isList())
                                            {
                                                if (rrrBot->size() == numberOfInputs)
                                                    {
                                                        //if (VERBOSE) fprintf(stderr, "MESSAGE: Getting a double list in the --index parameter! \n");
                                                        Bottle *rrrrBot;
                                                        rrrrBot = rrrBot->get(j).asList();
                                                        indexToPlot[j] = new int[rrrrBot->size()];
                                                        sizeIndexToPlot[j] = rrrrBot->size();
                                                        //if (VERBOSE) fprintf(stderr, "MESSAGE: Plot %d has %d indeces to plot! %s \n", index, sizeIndexToPlot[j], rrrrBot->toString().c_str());
                                                        for(int k = 0; k < rrrrBot->size(); k++)
                                                            {
                                                                if(rrrrBot->get(k).isInt())
                                                                    indexToPlot[j][k] = rrrrBot->get(k).asInt();
                                                            }
                                                        //if (VERBOSE) fprintf(stderr, "MESSAGE: the index to plot for input %d was set! \n", j);
                                                    }
                                                else
                                                    fprintf(stderr, "ERROR: a list of indeces was supplied but its dimension differs from the number of input ports given!\n");
                                                
                                            }

                                    }

                            }
                    }
                else if(rrBot->size() > 1 && rrBot->get(0).isInt())
                    {
                        sizeIndexToPlot[index] = rrBot->size();
                        indexToPlot[index] = new int[sizeIndexToPlot[0]];
                        for (int i = 0; i < rrBot->size(); i++)
                            indexToPlot[index][i] = rrBot->get(i).asInt();
                    }
                else
                    {
                        sizeIndexToPlot[0] = 1;
                        indexToPlot[0] = new int;
                        indexToPlot[0][0] = index;
                        fprintf(stderr, "WARNING: Ignoring option index since dimensions do not match\n");
                    }
            }
        else if (rf->find("index").isInt())
            {
                //if (VERBOSE) fprintf(stderr, "MESSAGE: was not a list. Initializing with 0\n");
                sizeIndexToPlot[0] = 1;
                indexToPlot[0] = new int;
                indexToPlot[0][0] = rf->find("index").asInt();
            }
        else
            {
                //if (VERBOSE) fprintf(stderr, "MESSAGE: was not a list. Initializing with 0\n");
                sizeIndexToPlot[0] = 1;
                indexToPlot[0] = new int;
                indexToPlot[0][0] = index;
            }

        if (indexToPlot == NULL)
            {
                //if (VERBOSE) fprintf(stderr, "MESSAGE: was not initialized. Initializing with 0\n");
                sizeIndexToPlot[0] = 1;
                indexToPlot[0] = new int;
                indexToPlot[0][0] = 0;
            }
        
        //if (VERBOSE) fprintf(stderr, "Plot %d has %d input/s and the associated indeces to plot are: \n", index, numberOfInputs);
        //for (int k = 0; k < numberOfInputs; k++)
        //    {
        //      if (VERBOSE) fprintf(stderr, "Input (%d) index/indeces:", k);
        //     for (int i = 0; i < sizeIndexToPlot[k]; i++)
        //          if (VERBOSE) fprintf(stderr, " %d ", indexToPlot[k][i]);
        //       if (VERBOSE) fprintf(stderr, "\n");
        //  }
        
        plot[index]->setInputPortIndex((int**)indexToPlot, (int*)sizeIndexToPlot, (int)numberOfInputs);

    }

    void yarpConnectRemoteLocal(ConstString remote, ConstString local)
    {
        if(!Network::connect(remote.c_str(), local.c_str(), "udp"))
            fprintf(stderr, "WARNING: Connection to %s  was NOT successfull. Waiting from explicit connection from user.\n", remote.c_str());
    }

    ~MainWindow()
    {
        if (VERBOSE) fprintf(stderr, "MESSAGE: deleting the plots...");
        for( int r = 0 ; r < nRows ; r++ )
                for( int c = 0 ; c < nCols ; c++ )
                    delete plot[r+nRows*c];
        delete[] nPortInputsInPlots;
        if (VERBOSE) fprintf(stderr, "done!\n");
    }
};

class MyModule:public RFModule
{
	MainWindow *mw;
	QApplication *qa;

public:

    void setGtkVars(MainWindow *mainWindow,	QApplication *qtApp)
    {
        mw = mainWindow;
        qa = qtApp;
    }

    // This is our main function. Will be called periodically every getPeriod() seconds.
    bool updateModule()
    {   
		//qa->processEvents();
        qa->exec();
        return false;
    }
        
    // Message handler. Just echo all received messages.
    bool respond(const Bottle& command, Bottle& reply) 
    {
        if (command.get(0).asString()=="quit")
            return false;     
        else
            reply=command;
        return true;
    }

    
    // Configure function. Receive a previously initialized
    // resource finder object. Use it to configure your module.
    // If you are migrating from the old module, this is the function
    // equivalent to the "open" method.
    bool configure(yarp::os::ResourceFinder &rf)
    {	

		if (VERBOSE) fprintf(stderr, "MESSAGE: the QApplication was succefully initialized\n");

        //optional, attach to terminal if you want that text typed at the console
        //is redirected to the respond method
        //attachTerminal();     
        return true;
    }

    // Interrupt function.
    bool interruptModule()
    {
		if (VERBOSE) fprintf(stderr, "MESSAGE: trying to interrupt the module.\n");
		mw->close(false);
		if (VERBOSE) fprintf(stderr, "MESSAGE: window was closed! \n");
		qa->exit();
		if (VERBOSE) fprintf(stderr, "MESSAGE: application exited! \n");
        return true;
    }

    //
    // Close function, to perform cleanup.
    bool close()
    {
        return true;
    }
};


int main(int argc, char **argv)
{
    //resource finder
    yarp::os::ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("portScope/conf");
    rf.setDefaultConfigFile("portScope.ini");
    rf.configure("ICUB_ROOT",argc,argv);

    rf.setDefault ("local", "/portScope/vector:i");
    rf.setDefault ("remote", "");
    rf.setDefault ("robot", "icub");
    rf.setDefault ("part", "head");

    //Yarp network initialization
    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    //create your module
    MyModule module; 
	MainWindow *mainWindow;
	QApplication *qtApp;
	//QEventLoop *qel;

    if(rf.check("verbose"))
        {
            VERBOSE = true;
            if (VERBOSE) fprintf(stderr, "MESSAGE: will now be VERBOSE\n");
        }
    else
        VERBOSE = false;

    qtApp = new QApplication(argc, argv);
    if (VERBOSE) fprintf(stderr, "MESSAGE: starting the configuration \n");	
		
    mainWindow = new MainWindow(&rf);
    if (VERBOSE) fprintf(stderr, "MESSAGE: main window initialized\n");	
    

    if(rf.find("dx").isInt() && rf.find("dy").isInt())
        {
            mainWindow->resize(rf.find("dx").asInt(), rf.find("dy").asInt());
            if (VERBOSE) fprintf(stderr, "MESSAGE: forcing window size %d %d\n", rf.find("dx").asInt(), rf.find("dy").asInt());	
        }
    else
        mainWindow->resize(600, 400);
    
    if(rf.find("x").isInt() && rf.find("y").isInt())
        mainWindow->move(rf.find("x").asInt(), rf.find("y").asInt());
    else
        mainWindow->move(0, 0);

#if QT_VERSION < 0x040000
    qtApp->setMainWidget(mainWindow);
#endif
    
    if(rf.find("title").isString())
        mainWindow->setCaption(rf.find("title").asString().c_str());
    
    mainWindow->show();
    

    // prepare and configure the resource finder 
    module.setGtkVars(mainWindow, qtApp);
    module.configure(rf);
    module.runModule();
	if (VERBOSE) fprintf(stderr, "MESSAGE: runModule terminated\n");	
	if (VERBOSE) fprintf(stderr, "MESSAGE: network closed\n");	

    delete mainWindow;
    delete qtApp;
	if (VERBOSE) fprintf(stderr, "MESSAGE: main window and qApp deleted\n");	
	yarp.fini();
    return 0;

}
