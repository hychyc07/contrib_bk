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

/*!
\mainpage Welcome to Aquila project! - WORK IN PROGRESS
<table border="0" align="right">
<tr><td><img src="university_logo.png" align="right" border="0" style="padding:5px;"></td></tr>
<tr><td><img src="crns_logo.gif" align="right" border="0" style="padding:5px;"></td></tr>
<tr><td><img src="poeticon_logo.png" align="right" border="0" style="padding:5px;"></td></tr>
</table>

Human beings have come a long way thanks to our imagination, curiosity and hard work. Ever accelerating scientific and technological progress is opening up new horizons and making what was previously considered science-fiction a reality.\n
The emergence of artificial intelligence and robotics is inevitable and Aquila project aims to play an important part in the development of next-generation methods, tools and scalable, multi-platform software used for cognitive robotics research.

Aquila project started in 2009 shortly after our team received iCub humanoid robot as part of the iTalk project. The early versions of Aquila provided a simple graphical user interfaces and helped our team and researchers
from different countries with their work.

As time went by, more and more modules were developed by different people, which was good as we could use more features and do more with the iCub. However, this also meant
ever-growing list of dependencies making Aquila hard to compile and setup. All the modules were inegrated within a single monolithic application and could not be used as standalone programs without graphical interface.
Because of these constraints, it was not possible to make use of the processing power of other computers on the network, which limited scalability of modules to the resources available on the localhost. The previous Aquila
could only run on Linux, which was another problem as many people needed to run Aquila on OSX or Windows.

These and other limitations led to complete redesign of Aquila and the new architecture was finished in December 2012. This was followed by the release of the very first version Aquila 2.0 in January 2013.
The new Aquila design overcame all of the previous constraints. Aquila is now multiplatform and runs on Linux, OSX and Windows across many computers making the most out of available resources. 
The graphical user interface became very clean hiding all the unnecessary parts in separate option dialogs availabe from integrated menus.
We have removed several dependancies and kept only what was absolutely essentail. Aquila now consists of two logical parts, Aquila GUI and Aquila modules.
These two parts are completely independent. The modules can run without GUI and the GUI can run without modules. 


<div class="callout-right image">
<img alt="Aquila architecture" src="aquila_modules3.png" />
<p>Aquila architecture: highlighting the independence between modules and graphical user interface.</p></div>


When Aquila GUI starts, it probes the local network and detects what modules are installed on all available servers (local and remote).
Aquila then dynamically adds graphical user interfaces for those modules that were detected. A user can then anytime add new modules that are available on local or remote servers. Multiple instances of of multiple modules can be running simulataneously
across many servers without conficts thanks to their unique identification.

When a new module is started, new graphical user interface is added under a new tab, which can be then easily moved, duplicated and closed just like in an internet browser. Every Aquila module, 
when started, sends information to Aquila GUI via messages sent through YARP ports. These messages typically contain information about what GPU cards are available on that server, what are the module settings etc. Aquila GUI receives this informaiton 
and updates all the necessary components. When a user asks the module to do something then a message is sent back to the module.
Depending on the message the module might for example start neural network training. During the training, the module can sent messages back to GUI informing user about the progress via integrated progress bar at the bottom of the GUI. When the module
finishes it can let GUI know and send for example neural network weights to be visualised and/or saved.

<div class="callout-center image">
<img alt="Aquila architecture" src="aquila_modules4.png" />
<p>Aquila architecture: highlighting the main aspects of module and GUI implementation their communication flow.</p></div>




\section download Download
<img src="platforms.jpg" align="right" border="0" style="padding:5px;">
[source package]: http://sourceforge.net/projects/aquila/files/latest/download "Aquila source package"
[SVN repository]: http://sourceforge.net/p/aquila/code/ "SVN repository"
Aquila currently works on Linux, OSX and Windows and can be downloaded as a [source package] or directly from our [SVN repository], which will give you the most recent code.

**Download Aquila source package**\n
The latest version is available from here: http://sourceforge.net/projects/aquila/files/latest/download

**Download Aquila from SVN (Linux, OSX or Cygwin)**\n
Use the command below to download the latest 'work in progress' code to 'Aquila' directory:
\code svn co http://svn.code.sf.net/p/aquila/code/trunk aquila \endcode

**Download Aquila via SVN client (Windows, Linux, OSX)**\n
This option will get you the latest 'work in progress' code. Before you start, you will need an SVN client.
If you are on Windows we highly recommend using [TortoiseSVN](http://tortoisesvn.tigris.org/). If you are on Linux or OSX there you might like
[SmartSVN](http://smartsvn.com). You will need the link below when checking out from repository:
\code http://svn.code.sf.net/p/aquila/code/trunk \endcode





\section installation Installation
This section describes a simple tree-step process to get Aquila up and running:

**1. Dependency satisfaction**
- [Qt 4.8](http://qt-project.org/downloads) - used for graphical user interface, process managment and communication between objects
- [CUDA 5.0](https://developer.nvidia.com/cuda-downloads) - used for development and compilation of GPU-accelerated code
- [YARP 2.3.20](http://eris.liralab.it/yarpdoc/download.html) - used for interfacing GUI with modules, local and remote module launching
- [CMake 2.8](http://www.cmake.org/cmake/resources/software.html)  - used for management of the build process in a compiler-independent manner

If you are on Windows you might like to use YARP installer instead of compiling everything from the source code.\n
Windows YARP Installer: http://wiki.icub.org/wiki/Downloads

If you are on Linux or OSX then your gcc should be 4.6 so that NVIDIA NVCC compiler works.
If you are on OSX then you might need to have gcc 4.6 also because of the OpenMP problems reported in previous versions.
OSX instructions: http://staticimport.blogspot.co.uk/2012/02/building-gcc-462-on-os-x-lion.html

**2. Environment setup**
- Linx
	+ do this
	+ then do that
- OSX
	+ do this
	+ then do that
- Windows
	+ do this
	+ then do that

**3. Compilation and installation**
- Linx
	+ do this
	+ then do that
- OSX
	+ do this
	+ then do that
- Windows
	+ do this
	+ then do that





\section icub_software_installation iCub Software Installation - optional
iCub software is essential for our work with the [iCub humanoid robot](http://www.icub.org/) or the simulator. Many of Aquila modules are completely independent from the iCub software, however, some modules may need to
connect to specific ports created by other modules from the iCub software. Therefore, installing iCub sofware is not required unless you want to use or develop a module that communicates with iCub software.

Installation instructions: http://wiki.icub.org/wiki/ICub_Software_Installation







\section usage Usage
\a
This section covers how to use Aquila and its integrated tools. It covers how to run and manage Aquila-modules, however, it does not cover how
to use them. For information about specific modules and their usage see the Modules section. (hyperlink)

Show how to start Aquila in normal or debug mode and explain the differences (logging, unique IDs of modules and servers)

Explain what Aquila does when it starts and what modules are added etc.

Explain how menus work and how a new module can be added on local or remote servers. (pictures, shortcuts, etc.)

About menus

Tools

Menus are integrated on OSX and Linux running Unity, otherwise menus are part of Aquila UI.






\section development Development
Register at sourceforge

Describe how new modules can be devloped. Talk about svn access, branches, standards, etc. (hyperlink to creating new module)

Report any bugs or request new features: http://sourceforge.net/p/aquila/tickets/






\section resources Resources
Below are links to different resources realted to Aquila:\n
- [Aquila on SourceForge](http://sourceforge.net/projects/aquila/)
- [Aquila on Facebook](http://www.facebook.com/pages/Aquila-Cognitive-Robotics-Research-Toolkit/120686251343379)
- [Aquila publication](https://dl.dropbox.com/u/81820/My%20Publications/IJCNN2011/IJCNN2011_Peniak_et_al_FINAL.pdf)
- [Aquila NVIDIA GPU Techonology Conference webminar (video stream)](http://developer.download.nvidia.com/GTC/Aquila-Open-Source-GPU-Accelerate-Toolkit-for-Cognitive-and-Neuro-Robotics-Research.mp4)
- [Aquila NVIDIA GPU Techonology Conference webminar (slides)](http://www.slideshare.net/mpeniak/cognitive-robotics-and-gpus-nvidia-gtc-express-webminar?from=ss_embed)
- [Aquila NVIDIA GPU Techonology Conference webminar (slides)](http://www.slideshare.net/mpeniak/cognitive-robotics-and-gpus-nvidia-gtc-express-webminar?from=ss_embed)
- [Aquila running mutiple time-scales recurrent neural network on iCub humanoid robot (video)](http://www.youtube.com/watch?v=vmDByFN6eig)
- [Aquila running iCub teleoperation (video)](http://www.youtube.com/watch?v=JDKu4q6dr84)
- [Aquila running iCub object learning (video)](http://www.youtube.com/watch?v=U-FOr7hEw1c)
- [NVIDIA CEO Jen-Hsun Huang mentions our research during the SC11 conference in Seattle](http://www.youtube.com/watch?v=kEv4NTEfYh8)
 

 



\section acknowledgements Acknowledgements 
<table border="0" align="right">
<tr><td><img src="poeticon_logo.jpg" align="right" border="0" style="padding:5px;"></td></tr>
<tr><td><img src="italk_logo.jpg" align="right" border="0" style="padding:5px;"></td></tr>
<tr><td><img src="nvidia_logo.jpg" align="right" border="0" style="padding:5px;"></td></tr>
</table>
We would like to thank to everyone who supported Aquila project. In particular we would like to thank to: 
- EU FP7 [Italk](http://www.italkproject.org/) and [Poeticon++](http://www.poeticon.eu/) projects for supporting this work
- [Giogio Metta](http://pasa.lira.dist.unige.it/), [Vadim Tikhanoff](http://www.vtikhanoff.com/), [Frederic Delaunay](http://www.linkedin.com/profile/view?id=63438334), [Federico Da Rold](http://eris.liralab.it/wiki/VVV12_Participants#Federico_Da_Rold) and [Angelo Cangelosi](http://www.plymouth.ac.uk/staff/acangelosi) for valuable feedback
- [Ron Babich](http://www.linkedin.com/pub/ron-babich/45/892/8a8), [John Tran](http://www.linkedin.com/pub/john-tran/1/45a/885) and [David Glasco](www.linkedin.com/pub/david-glasco/1/549/541) (NVIDIA research) for their help during the developmnet, testing and benchmarking of MTRNN, SOM and GA modules [(see this blog post)](http://www.martinpeniak.com/index.php?option=com_content&view=article&id=279:moving-to-california-to-do-research-at-nvidia&catid=17:updates)
- [Calisa Cole](http://www.linkedin.com/in/calisacole) (Marketing, NVIDIA) for inviting us to NVIDIA headquarters and allowing us to present Aquila and related research [(see this blog post)](http://www.martinpeniak.com/index.php?option=com_content&view=article&id=259)
- [Nadeem Mohammad](http://www.linkedin.com/pub/nadeem-mohammad/10/27/1aa) (CUDA Developer Relations, NVIDIA) and [Donal Murphy](http://www.linkedin.com/pub/donal-murphy/1/241/342) (NVIDIA GTC Marketing Manager) for invitation to present Aquila on GTC Express webminar series [(see video)](http://developer.download.nvidia.com/GTC/Aquila-Open-Source-GPU-Accelerate-Toolkit-for-Cognitive-and-Neuro-Robotics-Research.mp4)
- [Chandra Cheij](http://www.linkedin.com/in/chandracheij) (Manager of Academic Research Programs, NVIDIA), [Davide Marocco](http://www.plymouth.ac.uk/pages/dynamic.asp?page=staffdetails&id=dmarocco) and [Plymouth University](http://www.plymouth.ac.uk) for helping with the establishment of [NVIDIA CUDA lab](http://www.nvidia.co.uk/object/cuda-teaching-centres-uk.html), which is in addition to teaching CUDA used for development, testing and running Aquila modules
- [Ante Vukurepa](http://www.linkedin.com/pub/ante-vukorepa/3/607/639) for designing the logos and splash screen






\section credits Credits
[Martin Peniak](http://www.martinpeniak.com/) & [Anthony Morse](http://fostsvn.uopnet.plymouth.ac.uk/amorse/)






\section license License
Aquila is licensed under [FreeBSD](http://www.freebsd.org/copyright/freebsd-license.html).




- - -
<img src="aquila_full_logo.png" align="center" border="0" style="padding:5px;">
*/

/*!
\page c++_style_guide C++ Style Guide
This page provides basic quideliness to the programming syle we use in Aquila. It is recommended that you follow this style so that we keep the code consistent.
*/

/*!
\page creating_new_module Creating New Module
This page covers how to create a new Aquila GUI and module (to be done)
*/

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>
#include <QtGui/QApplication>
#include <QtNetwork/QHostInfo>
#include <QProcess>
#include <QFile>
#include <QTextStream>
#include "mainWindow.h"
#include "splashScreen.h"

int numLines = 0;
void customMessageHandler(QtMsgType type, const char *msg);

/**
* \brief Entry function.
* \param[in] argc - number of arguments
* \param[in] argv[] - argumnet list
* \return true if all went well
*/
int main(int argc, char *argv[])
{
	bool debug = false;
    bool remoteServers = true;
    QStringList localModules;
    QVector<MainWindow::Module> remoteModules;

	//process arguments
	for(int i=1; i<argc; i++)
	{
		QString cmd(argv[i]);

		if(cmd=="--debug")
		{
			debug = true;
		}
        else if(cmd=="--localMode")
        {
            remoteServers = false;
        }
		else if(cmd=="--localModules") // e.g. --localModules som,mtrnn,esn,tracker,era
		{
			QString module = argv[i+1];

			//read modules 
			if(!module.contains("--") || !module.isEmpty())
			{
				localModules = module.split(",");
				i++;
			}
		}
		else if(cmd=="--remoteModules") // e.g. --remoteModules 1=som,mtrnn,esn 2=tracker,era
		{
			QString command = argv[i+1];
			bool isNumber;

			//check if any modules were added at all
			if(!command.contains("--"))
			{

				//read modules until new parameter is found or no more parameters exist
				while(!command.contains("--") && i<argc-1)
				{
					i++;
					QStringList modules = command.split("=");

					if(!modules.empty())
					{
                        MainWindow::Module serverModule;
						modules.at(0).toInt(&isNumber);

						//check if the argument starts with server id
						if(isNumber)
						{
                            serverModule.serverID = modules.at(0);
                            serverModule.modules = modules.at(1).split(",");
                            remoteModules.push_back(serverModule);
						}
					}

					command = argv[i+1];
				}
			}
		}
	}

	//initialises message handeler
	if(!debug)
	{
		qInstallMsgHandler(customMessageHandler);
	}

    //current Aquila verions
    QString version = "Aquila 2.0";

    //localhost name
    QString hostName = QHostInfo::localHostName();

    //minimum version of YARP required for full support
    QString yarpVersion = "2.3.20";

    //add active developers to the list
    QStringList developers;
    developers<<" Martin Peniak"<<" & Anthony Morse";

    //initialise YARP
    yarp::os::Network yarp;

    //initialises Aquila and loads its icon
    QApplication *a = new QApplication(argc, argv);
    a->setWindowIcon(QPixmap(":/images/icon.png"));
    a->setAttribute(Qt::AA_X11InitThreads);

    //initialise splash screen
    QPixmap pixmap(":/images/splash.png");
    SplashScreen *splash = new SplashScreen(pixmap, 4);
    splash->setFont(QFont("Ariel", 8, QFont::Bold));
    splash->show();

    //initialise GUI, probe yarpservers, its modules and add local modules to GUI
    MainWindow *w = new MainWindow(0, version, hostName, yarpVersion);
    splash->showMessage(QString("Initialising ")+version,Qt::AlignLeft | Qt::AlignBottom,Qt::white);
    QObject::connect(a, SIGNAL(aboutToQuit()), w, SLOT(aboutToQuit()));
    a->processEvents();
    splash->showMessage(QObject::tr("Detecting available modules on the local network"),Qt::AlignLeft | Qt::AlignBottom,Qt::white);
    w->probeServers(remoteServers);
    a->processEvents();

	//start local modules
	if(!localModules.isEmpty())
	{
		splash->showMessage(QObject::tr("Starting local modules"),Qt::AlignLeft | Qt::AlignBottom,Qt::white);
		w->addLocalModules(localModules);
		a->processEvents();
	}

	//start remote modules
    if(!remoteModules.isEmpty())
	{
        if(remoteServers)
        {
            splash->showMessage(QObject::tr("Starting remote modules"),Qt::AlignLeft | Qt::AlignBottom,Qt::white);
            w->addRemoteModules(remoteModules);
            a->processEvents();
        }
        else
        {
            qWarning(" - main_gui: '--localMode' argument prevented remote modules from loading during startup");
        }
	}

	//load graphial user interface, show credits and close the splash
    splash->showMessage(QObject::tr("Loading graphical user interface"),Qt::AlignLeft | Qt::AlignBottom,Qt::white);
    a->processEvents();
    QString credits("Developed by");
    for(int i=0; i<developers.size(); i++)
    {
        credits.append(developers.at(i));
    }
    splash->showMessage(credits,Qt::AlignLeft | Qt::AlignBottom,Qt::white);
    a->processEvents();
    splash->finish(w);

    //show graphial user interface
    w->show();
    return a->exec();
}

/**
*@brief     Message handler that redirects warning and debug messages to log file.
*param[in]  type - message type (e.g. debug, warning, critical..)
*param[in]  msg - message
*/
void customMessageHandler(QtMsgType type, const char *msg)
{
    QString text;
    switch (type)
    {
    case QtDebugMsg:
        text = QString::number(numLines)+QString(":Debug: %1").arg(msg);
        break;

    case QtWarningMsg:
        text = QString(":Warning: %1").arg(msg);
        break;

    case QtCriticalMsg:
        text = QString(":Critical: %1").arg(msg);
        break;

    case QtFatalMsg:
        text = QString(":Fatal: %1").arg(msg);
        abort();
    }

    QFile outFile(QDir::currentPath()+"/log.txt");
    outFile.open(QIODevice::WriteOnly | QIODevice::Append);
    QTextStream ts(&outFile);
    ts<<text<<endl;
    numLines++;
}
