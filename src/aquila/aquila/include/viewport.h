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

#ifndef VIEWPORT_H
#define VIEWPORT_H

#include <QGLWidget>
#include <QThread>
#include <QTime>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>

#define FPS_SAMPLE_SIZE 20 //number of frames used for calculating the average FPS

class ViewportThread;

/*! Viewport for displaying images */
class Viewport : public QGLWidget
{
    Q_OBJECT

public:
    Viewport(QWidget *parent=0);
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePort;

    void initialise(QString senderPortName, QString receiverPortName);
    void setImage(const QImage& qImage);
    void start();
    void stop();

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* getImage();

protected:
    void paintEvent(QPaintEvent*);

private:
    ViewportThread *thread;

    QImage img;
    QTime currentTime;
    QTime lastTime;

    int elapsedTime;
    int numFrames;
    int fps;

private slots:
    void setImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> *yarpImage);
};

class ViewportThread : public QThread
{
    Q_OBJECT
public:
    ViewportThread(Viewport *pViewport);

    bool stopping;

private:
    Viewport *viewport;

    void run();

signals:
    void imageReceived(yarp::sig::ImageOf<yarp::sig::PixelRgb> *yarpImage);
};

#endif//VIEWPORT_H
