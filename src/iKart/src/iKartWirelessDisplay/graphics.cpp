/* 
 * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff Marco Randazzo Vadim Tikhanoff 
 * email:  vadim.tikhanoff@iit.it
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

#include <math.h>
#include "graphics.h"

#ifndef M_PI
#define M_PI 3.1415927
#endif

using namespace std;

/**********************************************************/

Gtk::Window* do_pixbufs()
{
    fprintf(stderr,"ERROR: program should not reach this point \n");
    return new GraphicsManager("default", "default", "default");
}

/**********************************************************/

GraphicsManager::GraphicsManager(string picBackgroud, string picBlocks, string picNumbers)
{
    m_back_width = 0;
    m_back_height = 0;

    set_title("Wireless Monitor");
    set_resizable(false);
    //set_decorated(false);
    set_keep_above(true);
    stick();

    this->picBackgroud = picBackgroud;
    this->picBlocks = picBlocks;
    this->picNumbers = picNumbers;

    load_pixbufs();

    set_size_request(m_back_width, m_back_height);
    m_refPixbuf = Gdk::Pixbuf::create(Gdk::COLORSPACE_RGB, FALSE, 8, m_back_width, m_back_height);
    m_DrawingArea.signal_expose_event().connect(sigc::mem_fun(*this, &GraphicsManager::on_drawingarea_expose));
    add(m_DrawingArea);

    show_all();
}

/**********************************************************/

GraphicsManager::~GraphicsManager()
{
}

/**********************************************************/

void GraphicsManager::load_pixbufs()
{
    if(m_refPixbuf_Background)
        return; /* already loaded earlier */

    Glib::RefPtr<Gdk::Pixbuf> tmp_Pixbuf_Numbers;
    Glib::RefPtr<Gdk::Pixbuf> tmp_Pixbuf_Blocks;
    Glib::RefPtr<Gdk::Pixbuf> tmp_Pixbuf_Charge;

    printf ("loading: %s\n", picBackgroud.c_str());
    try
    {
        m_refPixbuf_Background = Gdk::Pixbuf::create_from_file(picBackgroud.c_str());
    }
    catch (Glib::FileError e)
    {
        g_message("caught Glib::FileError %d", e.code());
    return;
    }
    catch (Gdk::PixbufError e)
    {
        g_message("Gdk::PixbufError: %d",e.code());
    return;
    }

    printf ("loading: %s\n", picNumbers.c_str());
    tmp_Pixbuf_Numbers    = Gdk::Pixbuf::create_from_file(picNumbers.c_str());

    printf ("loading: %s\n", picBlocks.c_str());
    tmp_Pixbuf_Blocks     = Gdk::Pixbuf::create_from_file(picBlocks.c_str());

    m_refPixbuf_Numbers   = tmp_Pixbuf_Numbers->add_alpha   (true, 255,0,255);
    m_refPixbuf_Blocks    = tmp_Pixbuf_Blocks->add_alpha    (true, 255,0,255);
  
    m_back_width = m_refPixbuf_Background->get_width();
    m_back_height = m_refPixbuf_Background->get_height();
}

/**********************************************************/

bool GraphicsManager::on_drawingarea_expose(GdkEventExpose *event)
{
    gint rowstride = m_refPixbuf->get_rowstride();

    const guchar* pixels = m_refPixbuf->get_pixels() + (rowstride * event->area.y) + (event->area.x * 3);

    Glib::RefPtr<Gdk::Window> refWindow = m_DrawingArea.get_window();
    Glib::RefPtr<Gdk::GC> refGC = m_DrawingArea.get_style()->get_black_gc();

    refWindow->draw_rgb_image_dithalign(refGC,
                            event->area.x, event->area.y,
                            event->area.width, event->area.height,
                            Gdk::RGB_DITHER_NORMAL,
                            pixels, rowstride,
                            event->area.x, event->area.y);

    return true;
}

/**********************************************************/

void GraphicsManager::update_graphics(double signal, double strenght)
{
    m_refPixbuf_Background->copy_area( 0, 0, m_back_width, m_back_height, m_refPixbuf, 0, 0);
  
    //signal = 100; //for debugging
    //draw numbers
    char buff[10];
    sprintf(buff,"%4.0f",signal);
    int len = strlen(buff);
    int point_off=0;
    for (int i=0;i<len;i++)
    {
        int off;
        int xpos;
        int ypos;
        GdkRectangle dest;
        if (buff[i]=='.')
            point_off=17;

        if (buff[i]>='0' && buff[i]<='9')
        {
            off=(buff[i]-'0')*29+point_off;    
            dest.x=80+i*29-point_off;
            dest.y=26;
            dest.width=29;
            dest.height=52;
            xpos=80+i*29-off;
            ypos=26;
            m_refPixbuf_Numbers->composite(m_refPixbuf,
                                        dest.x, dest.y,
                                        dest.width, dest.height,
                                        xpos, ypos,
                                        1, 1, Gdk::INTERP_NEAREST,255);
        }
    }

    sprintf(buff,"%4.1f",fabs(strenght));
    len = strlen(buff);
    point_off=0;
    for (int i=0;i<len;i++)
    {
        int xoff=0;
        int yoff=0;
        int xpos=0;
        int ypos=0;
        GdkRectangle dest;
        if (buff[i]=='.')
            point_off=17;

        if (buff[i]>='0' && buff[i]<='9')
        {
            xoff=(buff[i]-'0')*29+point_off;    
            dest.x=63+i*29-point_off;
            dest.y=86-yoff;
            dest.width=29;
            dest.height=52;
            xpos=63+i*29-xoff;
            ypos=86-yoff;
            m_refPixbuf_Numbers->composite(m_refPixbuf,
                                        dest.x, dest.y,
                                        dest.width, dest.height,
                                        xpos, ypos,
                                        1, 1, Gdk::INTERP_NEAREST,255);
        }
    }

    //draw charge indicator
    int n_blocks = int (signal*10 / 100.0);
    for (int i=0;i<n_blocks;i++)
    {
        int xoff=0;
        int yoff=15;
        int xpos=0;
        int ypos=0;
        GdkRectangle dest;

        dest.x=65-xoff;
        dest.y=75-i*6;
        dest.width=14;
        dest.height=7;
        xpos=65-xoff;
        ypos=75-i*6-yoff;
      
        m_refPixbuf_Blocks->composite(m_refPixbuf,
                                        dest.x, dest.y,
                                        dest.width, dest.height,
                                        xpos, ypos,
                                        1, 1, Gdk::INTERP_NEAREST,255);
    }

    m_DrawingArea.queue_draw();
    m_frame_num++;
}
