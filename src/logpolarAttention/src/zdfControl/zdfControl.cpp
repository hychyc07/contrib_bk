// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
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

#include "zdfControl.h"
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Property.h> 
#include <yarp/os/Network.h> 
#include <yarp/os/BufferedPort.h>


#include <string.h>
#include <sstream>

#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_K1 VOCAB2('k','1') //data penalty
#define COMMAND_VOCAB_K2 VOCAB2('k','2') //smoothness penalty base
#define COMMAND_VOCAB_K3 VOCAB2('k','3') //smoothness penalty
#define COMMAND_VOCAB_K4 VOCAB2('k','4') //radial penalty
#define COMMAND_VOCAB_K5 VOCAB2('k','5') //smoothness 3sigmaon2
#define COMMAND_VOCAB_K6 VOCAB2('k','6') //bland dog thresh

using namespace yarp::os;
using namespace std;

GtkWidget *mainWindow=0;
static GdkPixbuf *frame = NULL;

BufferedPort<yarp::sig::FlexImage> *ptr_inputPort=0;


std::string* command;   //reference to the string refering to the last command to send

int _frameN;            //Frame Number
bool _savingSet;        //Save Set of Images mode
// 
//yarp::os::BufferedPort<yarp::os::Bottle> *_pOutPort=0; //Output Point Port
Port* _pOutPort=0;
yarp::os::Bottle _outBottle;//Output Bottle Container
pgmOptions _options; //option set by the config file

GtkWidget *saveSingleDialog;
GtkWidget *saveSetDialog;
GtkWidget *menubar;
GtkWidget *fileMenu, *imageMenu, *helpMenu;
GtkWidget *fileItem, *helpItem;
GtkWidget *fileSingleItem, *fileSetItem, *fileQuitItem;
GtkWidget *imageSizeItem, *imageRatioItem, *imageFreezeItem, *imageFramerateItem;
GtkWidget *synchroDisplayItem;
GtkWidget *helpAboutItem;
// StatusBar
GtkWidget *statusbar;
GtkWidget *fpsStatusBar;
GtkWidget *fpsStatusBar2;

guint timeout_ID=0;
guint timeout_update_ID=0;

ViewerResources _resources;


static void createObjects() {
    command=new string("");
}

static void deleteObjects() {
    /*if (ptr_inputPort!=0)
        delete ptr_inputPort;
    if (ptr_portCallback!=0)
        delete ptr_portCallback;*/
    delete command;
}




//-------------------------------------------------
// Main Window Callbacks
//-------------------------------------------------
/**
* usual callback function 
*/
static void callback( GtkWidget *widget,gpointer   data ){
    printf ("Hello again - %s was pressed \n", (char *) data);
    
    if(!strcmp((char *)data,"OpencvSobel")){
        printf("OpencvSobel");
        command->assign("set ocv");
    }
    if(!strcmp((char *)data,"ConvolveMax")){
        printf("ConvolveMax");
        command->assign("set max");
    }
    if(!strcmp((char *)data,"ConvolveSeq")){
        printf("ConvolveSeq");
        command->assign("set seq");
    }
    if(!strcmp((char *)data,"ConvolveFil")){
        printf("ConvolvFil");
        command->assign("set fil");
    }
    if(!strcmp((char *)data,"IppSobel")){
        printf("IppSobel");
        command->assign("set ipp");
    }

}



//-------------------------------------------------
// Call Backs
//-------------------------------------------------
static void cb_digits_scale( GtkAdjustment *adj ) {
    /* Set the number of decimal places to which adj->value is rounded */
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K1);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write(bot);
    }
}

static void cb_digits_scale2( GtkAdjustment *adj ) {
    /* Set the number of decimal places to which adj->value is rounded */
    //selectiveAttentionModule->processor1->cannyOperator->setThresholdL((double)adj->value);
    //printf("k2: %f \n",(double) adj->value);
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K2);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write(bot);
    }
}

static void cb_digits_scale3( GtkAdjustment *adj ) {
    /* Set the number of maskSeed */
    //selectiveAttentionModule->processor1->maskSeed=adj->value;
    //printf("k3: %f \n",(double) adj->value);
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K3);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write(bot);
    }
}

static void cb_digits_scale4( GtkAdjustment *adj ) {
    /* Set the number of maskSeed */
    //selectiveAttentionModule->processor1->maskTop=adj->value;
    //printf("k4: %f \n",(double) adj->value);
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K4);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write(bot);
    }
}

static void cb_digits_scale5( GtkAdjustment *adj ) {
    /* Set the number of maskSeed */
    //selectiveAttentionModule->processor1->maskTop=adj->value;
    //printf("k5: %f \n",(double) adj->value);
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K5);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write(bot);
    }
}

static void cb_digits_scale6( GtkAdjustment *adj ) {
    /* Set the number of maskSeed */
    //selectiveAttentionModule->processor1->maskTop=adj->value;
    if (_pOutPort!=NULL) {
        yarp::os::Bottle bot; //= _pOutPort->prepare();
        bot.clear();
        bot.addVocab(COMMAND_VOCAB_SET);
        bot.addVocab(COMMAND_VOCAB_K6);
        bot.addDouble((double) adj->value);
        //_pOutPort->Content() = _outBottle;
        _pOutPort->write(bot);
    }
}

gint timeout_update_CB(gpointer data) {
    //portFpsData.getStats(av, min, max);
    //portFpsData.reset();
    gchar *msg;
    //gdk_threads_enter();
    msg=g_strdup_printf("zdfControl");
    updateStatusbar(fpsStatusBar, msg);
    g_free(msg);
    //displayFpsData.getStats(av, min, max);
    //displayFpsData.reset();
    
    //periodToFreq(av, min, max, avHz, minHz, maxHz);

    //msg=g_strdup_printf("Display: %.1f (min:%.1f max:%.1f) fps", avHz, minHz, maxHz);
    //updateStatusbar(fpsStatusBar2, msg);
    //g_free(msg);
    //gdk_threads_leave();
    return TRUE;
}

gint timeout_CB (gpointer data) {
    c++;
    switch(c) {
        case 1: {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                     bot.addVocab(COMMAND_VOCAB_K1);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj1),value);
                    mutex.post();
                }
        case 2: {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                    bot.addVocab(COMMAND_VOCAB_K2);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj2),value);
                    mutex.post();
                }
        case 3: {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                     bot.addVocab(COMMAND_VOCAB_K3);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj3),value);
                    mutex.post();
                }
        case 4: {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                     bot.addVocab(COMMAND_VOCAB_K4);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj4),value);
                    mutex.post();
                }
        case 5: {
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                     bot.addVocab(COMMAND_VOCAB_K5);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj5),value);
                    mutex.post();
                }
        case 6: {  
                    yarp::os::Bottle bot; //= _pOutPort->prepare();
                    bot.clear();
                    bot.addVocab(COMMAND_VOCAB_GET);
                     bot.addVocab(COMMAND_VOCAB_K6);
                    //_pOutPort->Content() = _outBottle;
                    Bottle in;
                    _pOutPort->write(bot,in);
                    double value=in.get(2).asDouble();
                    mutex.wait();
                    gtk_adjustment_set_value(GTK_ADJUSTMENT (adj6),value);
                    mutex.post();
                }

        default: {
                    c=0;
                 }
    }
    return TRUE;
}

gboolean delete_event( GtkWidget *widget, GdkEvent *event, gpointer data ) {
    // If you return FALSE in the "delete_event" signal handler,
    // GTK will emit the "destroy" signal. Returning TRUE means
    // you don't want the window to be destroyed.
    // This is useful for popping up 'are you sure you want to quit?'
    // type dialogs. 
    cleanExit();
    return TRUE;
}

gint expose_CB (GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    return TRUE;
}

static gboolean configure_event( GtkWidget *widget, GdkEventConfigure *event ) {
   _resources.configure(widget, 
        widget->allocation.width,
        widget->allocation.height);
  return TRUE;
}
gint menuFileQuit_CB(GtkWidget *widget, gpointer data) {
    cleanExit();
    return TRUE;
}

gint menuHelpAbout_CB(GtkWidget *widget, gpointer data) {
#if GTK_CHECK_VERSION(2,6,0)
    const gchar *authors[] = 
        {
            "Yarp developers",
            NULL
        };
    const gchar *license =
        "Released under the terms of the LGPLv2.1 or later, see LGPL.TXT\n"
        "The complete license description is contained in the\n"
        "COPYING file included in this distribution.\n"
        "Please refer to this file for complete\n"
        "information about the licensing of YARP.\n"
        "\n"
        "DISCLAIMERS: LICENSOR WARRANTS THAT THE COPYRIGHT IN AND TO THE\n"
        "SOFTWARE IS OWNED BY THE LICENSOR OR THAT THE SOFTWARE IS\n"
        "DISTRIBUTED BY LICENSOR UNDER A VALID CURRENT LICENSE. EXCEPT AS\n"
        "EXPRESSLY STATED IN THE IMMEDIATELY PRECEDING SENTENCE, THE\n"
        "SOFTWARE IS PROVIDED BY THE LICENSOR, CONTRIBUTORS AND COPYRIGHT\n"
        "OWNERS AS IS, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED\n"
        "INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,\n"
        "FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO\n"
        "EVENT SHALL THE LICENSOR, CONTRIBUTORS OR COPYRIGHT OWNERS BE\n"
        "LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN\n"
        "ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN\n"
        "ONNECTION WITH THE SOFTWARE.\n";

    gtk_show_about_dialog(GTK_WINDOW(mainWindow),
                          "name", "zdfControl",
                          "version", "1.0",
                          "license", license,
                          "website", "http://sourceforge.net/projects/yarp0",
                          "comments", "Parameters for the zdfModule ",
                          "authors", authors,
                          NULL);
#else
    printf("Missing functionality on older GTK version, sorry\n");
#endif

    return TRUE;
}

gint menuFileSingle_CB(GtkWidget *widget, GdkEventExpose *event, gpointer data) {
    if ( gtk_check_menu_item_get_active (GTK_CHECK_MENU_ITEM(widget)) ) 
        {
            gtk_check_menu_item_set_active(GTK_CHECK_MENU_ITEM(fileSetItem), FALSE);
            gtk_widget_show_all (saveSingleDialog);
        } 
    else 
        {
            gtk_widget_hide (saveSingleDialog);
        }

    return TRUE;
}
void setTimedMode(guint dT) {
   // ptr_portCallback->mustDraw(false);
    if (timeout_ID!=0)
        gtk_timeout_remove(timeout_ID);
    timeout_ID = gtk_timeout_add (dT, timeout_CB, NULL);
}

void setSynchroMode() {
    gtk_timeout_remove(timeout_ID);
    timeout_ID=0;
    //ptr_portCallback->mustDraw(true);
}

//-------------------------------------------------
// Main Window Menubar
//-------------------------------------------------
GtkWidget* createMenubar(void) {
    GtkWidget *menubar;

    menubar =  gtk_menu_bar_new ();
    GtkWidget *menuSeparator;	
    // Submenus Items on menubar
    fileItem = gtk_menu_item_new_with_label ("File");
    helpItem = gtk_menu_item_new_with_label ("Help");
    // Submenu: File 
    fileMenu = gtk_menu_new();
    menuSeparator = gtk_separator_menu_item_new();
    gtk_menu_append( GTK_MENU(fileMenu), menuSeparator);
    fileQuitItem = gtk_menu_item_new_with_label ("Quit");
    gtk_menu_append( GTK_MENU(fileMenu), fileQuitItem);
    gtk_signal_connect( GTK_OBJECT(fileQuitItem), "activate", GTK_SIGNAL_FUNC(menuFileQuit_CB), mainWindow);
    
    // Submenu: Help
    helpMenu = gtk_menu_new();
    helpAboutItem = gtk_menu_item_new_with_label ("About..");
    gtk_menu_append( GTK_MENU(helpMenu), helpAboutItem);
    gtk_signal_connect( GTK_OBJECT(helpAboutItem), "activate", GTK_SIGNAL_FUNC(menuHelpAbout_CB), mainWindow);
    // linking the submenus to items on menubar
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(fileItem), fileMenu);
    gtk_menu_item_set_submenu(GTK_MENU_ITEM(helpItem), helpMenu);
    // appending the submenus to the menubar
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), fileItem);
    gtk_menu_item_set_right_justified (GTK_MENU_ITEM (helpItem), TRUE);
    gtk_menu_bar_append(GTK_MENU_BAR(menubar), helpItem);
  
    return menubar;
}

static GtkWidget *xpm_label_box( gchar     *xpm_filename,gchar *label_text ) {
    GtkWidget *box;
    GtkWidget *label;
    GtkWidget *image;

    /* Create box for image and label */
    box = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box), 2);

    /* Now on to the image stuff */
    if(xpm_filename!=NULL)
        image = gtk_image_new_from_file (xpm_filename);

    /* Create a label for the button */
    label = gtk_label_new (label_text);

    /* Pack the image and label into the box */
    if(xpm_filename!=NULL)
        gtk_box_pack_start (GTK_BOX (box), image, FALSE, FALSE, 3);
    gtk_box_pack_start (GTK_BOX (box), label, FALSE, FALSE, 3);

    if(xpm_filename!=NULL)
        gtk_widget_show (image);
    gtk_widget_show (label);

    return box;
}

static void scale_set_default_values( GtkScale *scale ) {
    gtk_range_set_update_policy (GTK_RANGE (scale),GTK_UPDATE_CONTINUOUS);
    gtk_scale_set_digits (scale, 2);
    gtk_scale_set_value_pos (scale, GTK_POS_TOP);
    gtk_scale_set_draw_value (scale, TRUE);
}

//-------------------------------------------------
// Main Window Statusbar
//-------------------------------------------------
void updateStatusbar(GtkWidget *statusbar, gchar *msg) {
    GtkStatusbar *sb=GTK_STATUSBAR (statusbar);

    gtk_statusbar_pop (sb, 0); // clear any previous message, underflow is allowed 
    gtk_statusbar_push (sb, 0, msg);
}

//-------------------------------------------------
// Main Window 
//-------------------------------------------------
GtkWidget* createMainWindow(void) {
    
    
    GtkRequisition actualSize;
    GtkWidget* window;
    
    //gtk_init (&argc, &argv);
    window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title (GTK_WINDOW (window), "zdfControl");
    gtk_window_set_default_size(GTK_WINDOW (window), 205, 300); 
    gtk_window_set_resizable (GTK_WINDOW (window), TRUE);
    g_signal_connect (G_OBJECT (window), "destroy",
                      G_CALLBACK (gtk_main_quit),
                      NULL);

    // When the window is given the "delete_event" signal (this is given
    // by the window manager, usually by the "close" option, or on the
    // titlebar), we ask it to call the delete_event () function
    // as defined above. The data passed to the callback
    // function is NULL and is ignored in the callback function.
    //g_signal_connect (G_OBJECT (window), "delete_event", G_CALLBACK (delete_event), NULL);
    // Box for main window
    GtkWidget *box;
    GtkWidget *box2, *box3, *box5;
    box = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box);
    // MenuBar for main window
    menubar = createMenubar();
    gtk_box_pack_start (GTK_BOX (box), menubar, FALSE, TRUE, 0); // parameters (GtkBox *box, GtkWidget *child, gboolean expand, gboolean fill, guint padding);
    //gtk_widget_size_request(menubar, &actualSize);
    // Drawing Area : here the image will be drawed
    //da = gtk_drawing_area_new ();
    //g_signal_connect (da, "expose_event", G_CALLBACK (expose_CB), NULL);
    /*if (_options.outputEnabled == 1)
        {
            g_signal_connect (da, "button_press_event", G_CALLBACK (clickDA_CB), NULL);
            // Ask to receive events the drawing area doesn't normally subscribe to
            gtk_widget_set_events (da, gtk_widget_get_events (da) | GDK_BUTTON_PRESS_MASK);
        }*/
    //gtk_box_pack_start(GTK_BOX(box), da, TRUE, TRUE, 0);
    
    
    //Toolbox area
    //creates the area as collection of port processes sequence
    box2 = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_add (GTK_CONTAINER (window), box2);
    GtkWidget *boxButtons;
    GtkWidget *boxSliders;
    boxButtons = gtk_vbox_new (FALSE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxButtons), 0);
    boxSliders = gtk_hbox_new (TRUE, 0); // parameters (gboolean homogeneous_space, gint spacing);
    gtk_container_set_border_width (GTK_CONTAINER (boxSliders), 0);
    
    //-----SCALE section
//    GtkWidget *scrollbar;
//    GtkWidget *separator;
    GtkWidget *label;
    //GtkWidget *scale;
    
    
    GtkWidget *hscale, *vscale;

    // value, lower, upper, step_increment, page_increment, page_size 
    adj1 = gtk_adjustment_new (0.0, 0.0, 1.0, 0.01, 1.0, 1.0);
    vscale = gtk_vscale_new (GTK_ADJUSTMENT (adj1));
    scale_set_default_values (GTK_SCALE (vscale));
    gtk_box_pack_start (GTK_BOX (boxSliders), vscale, TRUE, TRUE, 0);
    gtk_widget_show (vscale);

    //separator = gtk_vseparator_new ();
    //gtk_box_pack_start (GTK_BOX (boxSliders), separator, FALSE, FALSE, 0);
    //gtk_widget_show (separator);

    //----------BOX3 SECTION:1
    //box3 is the single area that controls the processing towards the output port
    //every processes sequence has a sequence of checkboxes a label and a button
    box3 = gtk_hbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box3), 0);
    gtk_box_pack_start (GTK_BOX (box2), box3, FALSE, FALSE, 0);
    gtk_widget_show (box3);

    //----------BOX5 SUBSECTION:1
    box5 = gtk_vbox_new (FALSE, 0);
    gtk_container_set_border_width (GTK_CONTAINER (box5), 0);

    label = gtk_label_new ("ZDF PARAMETERS");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    label = gtk_label_new ("data penalty:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj1 = gtk_adjustment_new (1.0, 0.0,500.0,1.0, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj1));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj1), "value_changed",
                      G_CALLBACK (cb_digits_scale), NULL);

    label = gtk_label_new ("smoothness penalty base:");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);

    adj2 = gtk_adjustment_new (0.1, 0.0,500.0,1.0, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj2));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj2), "value_changed",
                      G_CALLBACK (cb_digits_scale2), NULL);

    label = gtk_label_new ("smoothness penalty");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj3 = gtk_adjustment_new (0.5, 0.0,1000.0,1.0, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj3));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj3), "value_changed",
                      G_CALLBACK (cb_digits_scale3), NULL);


    label = gtk_label_new ("radial penalty");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj4 = gtk_adjustment_new (0.1, 0.0,500.0,1.0, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj4));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj4), "value_changed",
                      G_CALLBACK (cb_digits_scale4), NULL);

    label = gtk_label_new ("smoothness 3sigmaon2");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj5 = gtk_adjustment_new (0.1, 0.0,500.0,1.0, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj5));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj5), "value_changed",
                      G_CALLBACK (cb_digits_scale5), NULL);

    label = gtk_label_new ("bland dog thresh");
    gtk_box_pack_start (GTK_BOX (box5), label, FALSE, FALSE, 0);
    gtk_widget_show (label);
    
    adj6 = gtk_adjustment_new (0.5, 0.0,500.0,1.0, 0.0, 0.0);
    hscale = gtk_hscale_new (GTK_ADJUSTMENT (adj6));
    gtk_widget_set_size_request (GTK_WIDGET (hscale), 200, -1);
    scale_set_default_values (GTK_SCALE (hscale));
    gtk_box_pack_start (GTK_BOX (box5), hscale, TRUE, TRUE, 0);
    gtk_widget_show (hscale);
    g_signal_connect (G_OBJECT (adj6), "value_changed",
                      G_CALLBACK (cb_digits_scale6), NULL);

    gtk_box_pack_start (GTK_BOX (box3), box5, FALSE, FALSE, 0);
    gtk_widget_show (box5);

 
    //gtk_container_add (GTK_CONTAINER (box2), boxSliders);
    gtk_box_pack_start(GTK_BOX(box), box2,FALSE,FALSE, 10);
    // StatusBar for main window
    statusbar = gtk_statusbar_new ();
    //updateStatusbar(GTK_STATUSBAR (statusbar));
    gtk_box_pack_start (GTK_BOX (box), statusbar, FALSE, TRUE, 0);
    gtk_widget_size_request(statusbar, &actualSize);
    //_occupiedHeight += 2*(actualSize.height);*/

    frame = gdk_pixbuf_new (GDK_COLORSPACE_RGB, FALSE, 8, 320, 240);
    // TimeOut used to refresh the screen
    timeout_ID = gtk_timeout_add (100, timeout_CB, NULL);

    mainWindow=window;

    return window;
}

void configure(yarp::os::ResourceFinder rf){
    /* Process all parameters from both command-line and .ini file */
    /* get the module name which will form the stem of all module port names */
    _options.portName      = rf.check("name", 
                           Value("/zdfControl"), 
                           "module name (string)").asString();
  
    _options.posX      = rf.check("x", 
                           Value(100), 
                           "module pos x (int)").asInt();
    _options.posY      = rf.check("y", 
                           Value(100), 
                           "module pos y (int)").asInt();

}

void setOptionsToDefault() {
    // Options defaults
    _options.refreshTime = 100;
    _options.outputEnabled = 0;
    _options.windWidth = 300;
    _options.windHeight = 300;
    _options.posX = 100;
    _options.posY = 100;
    _options.saveOnExit = 0;
}

bool openPorts() {
    //_pOutPort = new yarp::os::BufferedPort<yarp::os::Bottle>;
    _pOutPort=new Port;
    _options.portName+="/command:o";
    bool ok = _pOutPort->open(_options.portName.c_str());
    if (ok) {
        g_print("Port registration succeed!\n");
    }
    else {
            g_print("ERROR: Port registration failed.\nQuitting, sorry.\n");
            return false;
    }
    return true;
}

void closePorts() {
    _pOutPort->close();
    bool ok = true;
    if  (ok)
        printf("Port unregistration succeed!\n");
    else 
        printf("ERROR: Port unregistration failed.\n");
    delete _pOutPort;
    _pOutPort = NULL;

}

void cleanExit() {
    if (timeout_ID!=0)
        g_source_remove (timeout_ID);
    timeout_ID = 0;
    
    g_source_remove(timeout_update_ID);
    timeout_update_ID=0;

    gtk_main_quit ();
}

//-------------------------------------------------
// Main
//-------------------------------------------------
#undef main //ace leaves a "main" macro defined

int myMain(int argc, char* argv[]) {
    yarp::os::Network yarp;

    //initialize threads in gtk, copied almost verbatim from
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    //g_thread_init (NULL);
    //gdk_threads_init ();
    //gdk_threads_enter ();
    createObjects();
    _frameN = 0;
    timeout_ID = 0;
    setOptionsToDefault();

    yarp::os::ResourceFinder rf;
    //rf=new ResourceFinder();
    rf.setVerbose(true);
    rf.setDefaultConfigFile("zdfControl.ini"); //overridden by --from parameter
    rf.setDefaultContext("zdfControl/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
    
    _options.portName      = rf.check("name", 
                           Value("/zdfControl"), 
                           "module name (string)").asString();
  
    _options.posX      = rf.check("x", 
                           Value(100), 
                           "module pos x (int)").asInt();
    _options.posY      = rf.check("y", 
                           Value(100), 
                           "module pos y (int)").asInt();
    //configure(rf);
    
    // Parse command line parameters, do this before
    // calling gtk_init(argc, argv) otherwise weird things 
    // happens
    if (!openPorts())
        goto exitRoutine;

    // This is called in all GTK applications. Arguments are parsed
    // from the command line and are returned to the application.
    gtk_init (&argc, &argv);

    // create a new window
    mainWindow = createMainWindow();
    
    // Non Modal Dialogs
#if GTK_CHECK_VERSION(2,6,0)
    //saveSingleDialog = createSaveSingleDialog();
    //saveSetDialog = createSaveSetDialog();
#else
    printf("Functionality omitted for older GTK version\n");
#endif
    // Shows all widgets in main Window
    gtk_widget_show_all (mainWindow);
    gtk_window_move(GTK_WINDOW(mainWindow), _options.posX, _options.posY);
    // All GTK applications must have a gtk_main(). Control ends here
    // and waits for an event to occur (like a key press or
    // mouse event).

    //ptr_portCallback->attach(&_resources);
    //ptr_portCallback->attach(&portFpsData);
    //ptr_inputPort->useCallback(*ptr_portCallback);

    if (_options.synch)
    {
        setSynchroMode();
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), true);
    }
    else
    {
        setTimedMode(_options.refreshTime);
        gtk_check_menu_item_set_active (GTK_CHECK_MENU_ITEM(synchroDisplayItem), false);
    }
    gtk_main ();

exitRoutine:
    // leave critical section here. From example
    // http://library.gnome.org/devel/gdk/unstable/gdk-Threads.htm
    //gdk_threads_leave ();

    closePorts();
    deleteObjects();
    return 0;
}

#ifdef YARP_WIN32_NOCONSOLE
#include <windows.h>
// win32 non-console applications define WinMain as the
// entry point for the linker
int WINAPI WinMain(HINSTANCE hInstance,
                   HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine,
                   int nCmdShow)
{
    return myMain (__argc, __argv);
}
#else
int main(int argc, char* argv[]) {
    return myMain(argc, argv);
}
#endif

void printHelp() {
    g_print("zdfControl usage:\n");
    g_print("--name: input port name (default: /zdfControl)\n");
}

