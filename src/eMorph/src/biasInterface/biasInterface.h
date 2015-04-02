// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#if !defined(GCAMVIEW_H)
#define GCAMVIEW_H

//=============================================================================
// YARP Includes - Class Specific
//=============================================================================
#include <yarp/sig/Image.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
//=============================================================================
// GTK Includes 
//=============================================================================
#include <gtk/gtk.h>
#include "viewerResources.h"
#include <string>

#define COMMAND_VOCAB_PR      VOCAB2('p','r')
#define COMMAND_VOCAB_FOLL    VOCAB4('f','o','l','l')
#define COMMAND_VOCAB_DIFF    VOCAB3('d','i','f')
#define COMMAND_VOCAB_DIFFON  VOCAB4('d','i','f','n')
#define COMMAND_VOCAB_PUY     VOCAB3('p','u','y')
#define COMMAND_VOCAB_REFR    VOCAB4('r','e','f','r')
#define COMMAND_VOCAB_REQ     VOCAB3('r','e','q')
#define COMMAND_VOCAB_DIFFOFF VOCAB4('d','i','f','f')
#define COMMAND_VOCAB_PUX     VOCAB3('p','u','x')
#define COMMAND_VOCAB_REQPD   VOCAB4('r','e','q','p') 
#define COMMAND_VOCAB_INJGND  VOCAB4('i','n','j','g')
#define COMMAND_VOCAB_CAS     VOCAB3('c','a','s')
#define COMMAND_VOCAB_PROG    VOCAB4('p','r','o','g')
#define COMMAND_VOCAB_BIAS    VOCAB4('b','i','a','s')
#define COMMAND_VOCAB_SAVE    VOCAB4('s','a','v','e')

//-------------------------------------------------
// Callbacks
//-------------------------------------------------
gboolean forceDraw(gpointer data);
// Timeout CB
gint timeout_CB (gpointer data);
// Main Window close CB
gboolean delete_event( GtkWidget *widget, GdkEvent *event, gpointer data );
//  Window redraw CB
gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data);
// Click on Drawinf Area CB
gint clickDA_CB (GtkWidget *widget, GdkEventButton *event, gpointer data);
// Menubar CBs
gint menuFileQuit_CB(GtkWidget *widget, gpointer data);
gint menuHelpAbout_CB(GtkWidget *widget, gpointer data);
gint menuImageInterval_CB(GtkWidget *widget, gpointer data);
gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data);

void setTimedMode(guint dT);

void setSynchroMode();

void saveImageNow();

//-------------------------------------------------
// Non Modal Dialogs
//-------------------------------------------------
GtkWidget* createSaveSingleDialog();
GtkWidget* createSaveSetDialog();
//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* createMenubar();
//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
void updateStatusbar(GtkWidget *statusbar, gchar *msg);
//-------------------------------------------------
// Main Window
//-------------------------------------------------
GtkWidget* createMainWindow(void);
//-------------------------------------------------
// Service Functions
//-------------------------------------------------
bool getImage();
void parseOptFile(char *fileName);
bool parseParameters(int argc, char *argv[]);
void saveOptFile(char *fileName);
void setOptionsToDefault();
bool openPorts();
void closePorts();
bool setUp();
void cleanExit();
void printHelp();

//-------------------------------------------------
// Global Variables
//-------------------------------------------------
// main window 
GtkObject *adj1, *adj2,*adj3, *adj4,*adj5, *adj6;
GtkObject *adj11, *adj12,*adj13, *adj14,*adj15, *adj16;
GtkObject *adjMotion;
GtkWidget *entrySAVELOAD;
yarp::os::Semaphore mutex;
int c=0;
const gchar *entry_file;
FILE* fout; 

//-------------------------------------------------
// Program Options 
//-------------------------------------------------
struct mOptions
{
    unsigned int    refreshTime;
    std::string     portName;
    char            networkName[256];
    int             windWidth;
    int             windHeight;
    int             posX;
    int             posY;
    char            fileName[256];
    int             saveOnExit;
    std::string           outPortName;
    char            outNetworkName[256];
    int             outputEnabled;
    bool            synch;
};
typedef struct mOptions pgmOptions;



#endif // #if !defined(GCAMVIEW_H)
