/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Marco Randazzo Vadim Tikhanoff
 * email:  marco.randazzo@iit.it
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

#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <gtkmm.h>
#include <string>
#define N_IMAGES 10

class GraphicsManager : public Gtk::Window
{
public:
    GraphicsManager(std::string picBackgroud, std::string picBlocks, std::string picNumbers);
    virtual ~GraphicsManager();
    void update_graphics(double signal, double strenght);

protected:
    virtual void load_pixbufs();

    //signal handlers:
    virtual bool on_drawingarea_expose(GdkEventExpose *event);
    
    //Member widgets:
    Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf;
    Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Background;
    Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Numbers;
    Glib::RefPtr<Gdk::Pixbuf> m_refPixbuf_Blocks;
    Gtk::DrawingArea m_DrawingArea;

    std::string picBackgroud;
    std::string picBlocks;
    std::string picNumbers;

    guint m_back_width, m_back_height;
    gint m_frame_num;
    std::string pics_path;
};

#endif
