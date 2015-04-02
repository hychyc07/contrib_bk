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

#include "splashScreen.h"

/*!
* \brief     Constructor.
* \param[in] pixmap - picture for window background
* \param[in] maxLines - number of lines in boot log
* \param[in] f - window flags
*/
SplashScreen::SplashScreen(const QPixmap &pixmap, int maxLines, Qt::WindowFlags f):QSplashScreen(pixmap, f), maxLns(maxLines)
{
    msg = QString::null;
    lns = 0;
    align = Qt::AlignLeft;
    clr = Qt::black;
}

/*!
* \brief     Constructor.
* \param[in] parent - parent of widget
* \param[in] pixmap - picture for window background
* \param[in] maxLines - number of lines in boot log
* \param[in] f - window flags
*/
SplashScreen::SplashScreen(QWidget *parent, const QPixmap &pixmap, int maxLines, Qt::WindowFlags f):QSplashScreen(parent, pixmap, f), maxLns(maxLines)
{
    msg = QString::null;
    lns = 0;
    align = Qt::AlignLeft;
    clr = Qt::black;
}

/*!
* \brief Destructor.
*/
SplashScreen::~SplashScreen()
{
}

/*!
* \brief Sets the maximal number of lines in log.
* \param[in] maxLines - number of lines in log
*/
void SplashScreen::setMaxLines(int maxLines)
{
    if(maxLines>=1)
    {
        maxLns = maxLines;
        if(lns>maxLns)
        {
            deleteTop(lns-maxLns);
            QSplashScreen::showMessage(msg, align, clr);
        }
    }
}

/*!
* \brief Returns the maximal number of lines in log.
* \return maxLns - maximal number of lines
*/
int SplashScreen::maxLines()
{
    return maxLns;
}

/*!
* \brief Adds new message line in log.
* \param[in] message - message text
* \param[in] alignment - placement of log in window
* \param[in] color - color used for protocol display
*/
void SplashScreen::showMessage(const QString &message, int alignment, const QColor &colour)
{
    align = alignment;
    clr = colour;

    if(msg.size()!=0)
    {
        msg += '\n' + message;
    }
    else
    {
        msg = message;
    }

    QStringList linesList = msg.split('\n');
    lns = linesList.size();

    if(lns>maxLns)
    {
        deleteTop(lns - maxLns);
    }

    QSplashScreen::showMessage(msg, align, clr);
}

/*!
* \brief Removes all messages being displayed in log.
*/
void SplashScreen::clearMessage()
{
    msg = QString::null;
    lns = 0;
    QSplashScreen::showMessage(msg, align, clr);
}

/*!
* \brief Removes first lines entries in log.
* \param[in] lines - number of lines to be removed
*/
void SplashScreen::deleteTop(int lines)
{
    QStringList linesList = msg.split('\n');

    for(int i=0; i<lines; i++)
    {
        linesList.removeFirst();
    }

    msg = linesList.join(QString('\n'));
    lns -= lines;
}
