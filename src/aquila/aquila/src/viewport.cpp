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

#include "viewport.h"

/*!
 * \brief Constructor.
 * \param[in] parent - parent widget
 */
Viewport::Viewport(QWidget* parent) : QGLWidget(parent)
{
    numFrames = 0;
    fps = 0;
}

/*!
 * \brief Initialsation.
 * \param[in] senderPortName - port name of the sender
 * \param[in] receiverPortName - port name of the receiver
 */
void Viewport::initialise(QString senderPortName, QString receiverPortName)
{
    thread = new ViewportThread(this);
    QObject::connect(thread, SIGNAL(imageReceived(yarp::sig::ImageOf<yarp::sig::PixelRgb>*)), this, SLOT(setImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>*)));
    imagePort.open(receiverPortName.toStdString().c_str());
    yarp::os::Network::connect(senderPortName.toStdString().c_str(), receiverPortName.toStdString().c_str());
}

/*!
 * \brief Starts thread.
 */
void Viewport::start()
{
    thread->stopping = false;
    thread->start();
}

/*!
 * \brief Stops thread.
 */
void Viewport::stop()
{
    thread->stopping = true;
    thread->exit();
}

/*!
 * \brief Sets and displays image.
 * \param[in] qImage - image to be displayed
 */
void Viewport::setImage(const QImage& qImage)
{
    img = qImage;
    this->update();
}

/*!
 * \brief Sets and displays image.
 * \param[in] yarpImage - image to be displayed
 */
void Viewport::setImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> *yarpImage)
{
    QImage qImage(yarpImage->width(), yarpImage->height(), QImage::Format_RGB32);
    for(int i=0; i<yarpImage->width(); i++)
    {
        for(int j=0; j<yarpImage->height(); j++)
        {
            yarp::sig::PixelRgb& yarpPixel = yarpImage->pixel(i, j);
            qImage.setPixel(i, j, qRgb(yarpPixel.r, yarpPixel.g, yarpPixel.b));
        }
    }
    img = qImage;
    this->update();
}

/*!
 * \brief Draws the content of the qPainter.
 */
void Viewport::paintEvent(QPaintEvent*)
{
    //initialises and start painter
    QPainter p;
    p.begin(this);
    p.setRenderHint(QPainter::SmoothPixmapTransform, 1);

    //draws the image
    p.drawImage(this->rect(), img);

    //draws frames per second
    QFont font=p.font();
    font.setPointSize(10);
    p.setFont(font);
    QPen b;
    b.setColor(Qt::white);
    p.setPen(b);

    if(fps>0)
    {
        p.drawText(10,20,QString::number(fps));
    }

    if(numFrames==0)
    {
        lastTime = currentTime;
        currentTime = QTime::currentTime();
        numFrames++;
    }
    else if(numFrames==FPS_SAMPLE_SIZE)
    {
        elapsedTime = (currentTime.second()*1000 + currentTime.msec()) - (lastTime.second()*1000 + lastTime.msec());
        fps = (int)((FPS_SAMPLE_SIZE*1000)/elapsedTime);
        numFrames = 0;
    }
    else
    {
        numFrames++;
    }

    //closes the painer
    p.end();
}

/*!
 * \brief Gets Yarp image from a port.
 * \return Yarp image
 */
yarp::sig::ImageOf<yarp::sig::PixelRgb>* Viewport::getImage()
{
    return imagePort.read();
}

/*!
 * \brief Constructor.
 * \param[in] pViewport - pointer to viewport
 */
ViewportThread::ViewportThread(Viewport *pViewport)
{
    viewport = pViewport;
    stopping = false;
}

/*!
 * \brief Thread loop.
 */
void ViewportThread::run()
{
    while(!isFinished())
    {
        //gets image from a port and broadcast it in signal
        if(!stopping)
        {
            emit imageReceived(viewport->getImage());
        }
    }

    stopping = false;
}
