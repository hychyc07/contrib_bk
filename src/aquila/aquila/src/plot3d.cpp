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

#include <QMouseEvent>
#include <math.h>
#include "plot3d.h"

/*!
 * \brief Constructor.
 */
Plot3D::Plot3D(QWidget *parent) : QGLWidget(parent)
{
    setMouseTracking(true);
}

/*!
 * \brief Initialises OpenGL.
 */
void Plot3D::initializeGL()
{
    minX = 1.0;
    minY = 1.0;
    minZ = 1.0;
    maxX = 0.0;
    maxY = 0.0;
    maxZ = 0.0;

    startX = 0;
    startY = 0;

    centreX = 0.0;
    centreY = 0.0;
    centreZ = 0.0;

    rotationX = 0.0;
    rotationY = 0.0;

    surfaceRenderMode = 0;
    
    zoom = -2.5;
    scale = 1;
    ratio = 1;

    data = NULL;

    initialised = false;
    holdingButton = false;
    draw = false;

    glClearColor(1, 1, 1, 1);
    glEnable(GL_DEPTH_TEST);
    glShadeModel(GL_SMOOTH);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

/*!
 * \brief Sets perspective.
 * \param[in] fovy - field of view
 * \param[in] aspect - aspect ratio
 * \param[in] zmin - distance to the near depth clipping plane
 * \param[in] zmax - distance to the far depth clipping plane
 */
void Plot3D::setPerspective(GLfloat fovy, GLfloat aspect, GLfloat zmin, GLfloat zmax)
{
    GLfloat xmin, xmax, ymin, ymax;
    ymax = zmin * tan(fovy * 3.14159265 / 360.0);
    ymin = -ymax;
    xmin = ymin * aspect;
    xmax = ymax * aspect;
    glFrustum(xmin, xmax, ymin, ymax, zmin, zmax);
}

/*!
* \brief Resizes OpenGL window.
* \param[in] x - width
* \param[in] y - height
*/
void Plot3D::resizeGL(int w, int h)
{
    if(w>0 && h>0)
    {
        ratio = (float)w/(float)h;
    }

    glViewport((GLfloat)0.0, (GLfloat)0.0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    setPerspective((GLfloat)45.0, (GLfloat)w/(GLfloat)h, (GLfloat)0.1, (GLfloat)1000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

/*!
* \brief Main painting function.
*/
void Plot3D::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if(draw)
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef(0.0, 0.0, zoom);

        //surface plot and bounding box
        glPushMatrix();
        glRotatef(rotationX, 1.0, 0.0, 0.0);
        glRotatef(rotationY, 0.0, 1.0, 0.0);
        drawBoundingBox();
        float ctrX = minX + (maxX-minX) / 2;
        float ctrY = minY + (maxY-minY) / 2;
        float ctrZ = minZ + (maxZ-minZ) / 2;
        glTranslatef(-ctrX*scaleX, -ctrY*scaleY, -ctrZ*scaleZ); // move object to centre
        glScalef(scaleX,scaleY,scaleZ);
        drawSurface();
        glPopMatrix();

        //draw 2D map
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glTranslatef((GLfloat)((2.3*ratio)-0.6), (GLfloat)1.7, (GLfloat)-5.0);
        glPushMatrix();
        drawMap();
        glPopMatrix();

        //render text
        drawInformation();
    }
}

/*!
* \brief Processes mouse press event.
* \param[in] event - mouse event
*/
void Plot3D::mousePressEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton)
    {
        holdingButton = true;
    }
}

/*!
* \brief Processes mouse release event.
* \param[in] event - mouse event
*/
void Plot3D::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->button()==Qt::LeftButton)
    {
        holdingButton = false;

        //save mouse location
        startX = event->y();
        startY = event->x();
    }
}

/*!
* \brief Processes mouse move event.
* \param[in] event - mouse event
*/
void Plot3D::mouseMoveEvent(QMouseEvent *event)
{
    if(holdingButton)
    {
        int diffX = startX - event->y();
        int diffY = startY - event->x();

        rotationX -= diffX;
        rotationY -= diffY;
    }

    //save mouse location
    startX = event->y();
    startY = event->x();
    updateGL();
}

/*!
* \brief Processes mouse wheel event.
* \param[in] event - mouse event
*/
void Plot3D::wheelEvent(QWheelEvent *event)
{
    int numDegrees = event->delta()/8;
    int numSteps = numDegrees/15;

    zoom += numSteps/float(5);

    updateGL();
}

/*!
* \brief Draws plot bounding box.
*/
void Plot3D::drawBoundingBox()
{
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    glColor3f(0.0,0.0,0.0);
    glLineWidth(2);

    glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex3f(  0.5f, -0.5f, -0.5f );
    glTexCoord2f(1, 0); glVertex3f( -0.5f, -0.5f, -0.5f );
    glTexCoord2f(1, 1); glVertex3f( -0.5f,  0.5f, -0.5f );
    glTexCoord2f(0, 1); glVertex3f(  0.5f,  0.5f, -0.5f );

    glTexCoord2f(0, 0); glVertex3f(  0.5f, -0.5f,  0.5f );
    glTexCoord2f(1, 0); glVertex3f(  0.5f, -0.5f, -0.5f );
    glTexCoord2f(1, 1); glVertex3f(  0.5f,  0.5f, -0.5f );
    glTexCoord2f(0, 1); glVertex3f(  0.5f,  0.5f,  0.5f );

    glTexCoord2f(0, 0); glVertex3f( -0.5f, -0.5f,  0.5f );
    glTexCoord2f(1, 0); glVertex3f(  0.5f, -0.5f,  0.5f );
    glTexCoord2f(1, 1); glVertex3f(  0.5f,  0.5f,  0.5f );
    glTexCoord2f(0, 1); glVertex3f( -0.5f,  0.5f,  0.5f );

    glTexCoord2f(0, 0); glVertex3f( -0.5f, -0.5f, -0.5f );
    glTexCoord2f(1, 0); glVertex3f( -0.5f, -0.5f,  0.5f );
    glTexCoord2f(1, 1); glVertex3f( -0.5f,  0.5f,  0.5f );
    glTexCoord2f(0, 1); glVertex3f( -0.5f,  0.5f, -0.5f );

    glTexCoord2f(0, 1); glVertex3f( -0.5f,  0.5f, -0.5f );
    glTexCoord2f(0, 0); glVertex3f( -0.5f,  0.5f,  0.5f );
    glTexCoord2f(1, 0); glVertex3f(  0.5f,  0.5f,  0.5f );
    glTexCoord2f(1, 1); glVertex3f(  0.5f,  0.5f, -0.5f );

    glTexCoord2f(0, 0); glVertex3f( -0.5f, -0.5f, -0.5f );
    glTexCoord2f(0, 1); glVertex3f( -0.5f, -0.5f,  0.5f );
    glTexCoord2f(1, 1); glVertex3f(  0.5f, -0.5f,  0.5f );
    glTexCoord2f(1, 0); glVertex3f(  0.5f, -0.5f, -0.5f );
    glEnd();
}

/*!
* \brief Draws 2d map.
* \note Chosen 3 dimensions are plotted as RGB colours.
*/
void Plot3D::drawMap()
{
    glEnable(GL_TEXTURE_2D);

    glColor4f(1,1,1,1);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glBindTexture(GL_TEXTURE_2D, mapTexture);

    glBegin(GL_QUADS);
    glTexCoord2f(0, 0); glVertex3f(  0.5f, -0.5f, -0.5f );
    glTexCoord2f(1, 0); glVertex3f( -0.5f, -0.5f, -0.5f );
    glTexCoord2f(1, 1); glVertex3f( -0.5f,  0.5f, -0.5f );
    glTexCoord2f(0, 1); glVertex3f(  0.5f,  0.5f, -0.5f );
    glEnd();

    glDisable(GL_TEXTURE_2D);
}

/*!
* \brief Loads plot data as a texture.
* \param[in] plotData - plot data
*/
void Plot3D::loadTexture(float ***plotData)
{
    glEnable(GL_TEXTURE_2D);
    QImage image(plWidth, plWidth, QImage::Format_ARGB32);
    for(int i=0; i<plWidth; i++)
    {
        for(int j=0; j<plWidth; j++)
        {
            //get pixel value from float
            QColor c = QColor::fromRgbF(scaleRange(plotData[i][j][0], minX, maxX, 0, 1),
                                        scaleRange(plotData[i][j][1], minY, maxY, 0, 1),
                                        scaleRange(plotData[i][j][2], minZ, maxZ, 0, 1));
            //set pixel
            image.setPixel(i, j, qRgba(c.red(),c.green(),c.blue(),1));
        }
    }

    texture = QGLWidget::convertToGLFormat(image);
    glGenTextures(1, &mapTexture);
    glBindTexture(GL_TEXTURE_2D, mapTexture);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_NEAREST);
    glTexImage2D( GL_TEXTURE_2D, 0, 3, texture.width(), texture.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, texture.bits() );

    glDisable(GL_TEXTURE_2D);
}

/*!
* \brief Draws plot surface.
*/
void Plot3D::drawSurface()
{
    int i;
    int j;

    //draw wireframe and fill polygons if selected
    if(surfaceRenderMode==0 || surfaceRenderMode==1)
    {
        glLineWidth(1);
        glColor3f(0.0,0.0,0.0);
        glEnable(GL_POLYGON_OFFSET_FILL);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glPolygonOffset( 1.0, 1.0 );

        glBegin(GL_LINE_LOOP);
        for(i=0; i<plWidth-1; i++)
        {
            glVertex3fv(data[i][0]);
        }
        for(j=0; j<plWidth-1; j++)
        {
            glVertex3fv(data[i][j]);
        }
        for(; i >= 0; i--)
        {
            glVertex3fv(data[i][j]);
        }
        for(; j>=0; j--)
        {
            glVertex3fv(data[0][j]);
        }
        glEnd();

        for(i=1; i<plWidth-1; i++)
        {
            glBegin(GL_LINE_STRIP);
            for(j=0; j<plWidth; j++)
            {
                glVertex3fv(data[i][j]);
            }
            glEnd();
        }
        for(j=1; j<plWidth-1; j++)
        {
            glBegin(GL_LINE_STRIP);
            for(i=0; i<plWidth; i++)
            {
                glVertex3fv(data[i][j]);
            }
            glEnd();
        }

        if(surfaceRenderMode==0)
        {
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            for(i=0; i<plWidth-1; i++)
            {
                glBegin(GL_TRIANGLE_STRIP);
                glColor4d(scaleRange(data[i][0][0], minX, maxX, 0, 1), scaleRange(data[i][0][1], minY, maxY, 0, 1), scaleRange(data[i][0][2], minZ, maxZ, 0, 1), 1);
                glVertex3fv(data[i][0]);
                glColor4d(scaleRange(data[i+1][0][0], minX, maxX, 0, 1), scaleRange(data[i+1][0][1], minY, maxY, 0, 1), scaleRange(data[i+1][0][2], minZ, maxZ, 0, 1), 1);
                glVertex3fv(data[i+1][0]);

                for(j=0; j<plWidth-1; j++)
                {
                    glColor4d(scaleRange(data[i][j+1][0], minX, maxX, 0, 1), scaleRange(data[i][j+1][1], minY, maxY, 0, 1), scaleRange(data[i][j+1][2], minZ, maxZ, 0, 1), 1);
                    glVertex3fv(data[i][j+1]);
                    glColor4d(scaleRange(data[i+1][j+1][0], minX, maxX, 0, 1), scaleRange(data[i+1][j+1][1], minY, maxX, 0, 1), scaleRange(data[i+1][j+1][2], minZ, maxZ, 0, 1), 1);
                    glVertex3fv(data[i+1][j+1]);
                }
                glEnd();
            }
        }
    }
    else if(surfaceRenderMode==2)
    {
        glPolygonMode(GL_FRONT_AND_BACK, GL_POINT);
        glPointSize(3);
        glBegin(GL_POINTS);
        for(i=0; i<plWidth; i++)
        {
            for(j=0; j<plWidth; j++)
            {
                glVertex3fv(data[i][j]);
            }
        }
        glEnd();
    }

    glDisable(GL_POLYGON_OFFSET_FILL);
}

/*!
* \brief Sets surface rendering mode.
* \param[in] modeID - rendering mode id
*/
void Plot3D::setSurfaceRenderMode(int modeID)
{
    surfaceRenderMode = modeID;
    updateGL();
}

/*!
* \brief Sets plot information.
* \param[in] info - plot information
*/
void Plot3D::setInformation(QStringList info)
{
    information = info;
}

/*!
* \brief Draws plot information.
*/
void Plot3D::drawInformation()
{
    glColor3f(0.0,0.0,0.0);

    for(int i=0; i<information.size(); i++)
    {
        QFont font;
        font.setPointSize(10);
        renderText(this->geometry().width() - 150, (this->geometry().height() - 100 ) + (i*20), information.at(i),font);
    }
}

/*!
* \brief Calculates center of plot data.
* \param[in] plotData - plot data
* \param[in] plotWidth - number of plot elements in each dimension
*/
void Plot3D::calculateCentre(float ***plotData, int plotWidth)
{
    minX = 1.0;
    minY = 1.0;
    minZ = 1.0;
    maxX = 0.0;
    maxY = 0.0;
    maxZ = 0.0;
    centreX = 0.0;
    centreY = 0.0;
    centreZ = 0.0;

    for(int i=0; i<plotWidth; i++)
    {
        for(int j=0; j<plotWidth; j++)
        {
            centreX += plotData[i][j][0];
            centreY += plotData[i][j][1];
            centreZ += plotData[i][j][2];

            //find min
            if(plotData[i][j][0]<minX)
            {
                minX = plotData[i][j][0];
            }

            if(plotData[i][j][1]<minY)
            {
                minY = plotData[i][j][1];
            }

            if(plotData[i][j][2]<minZ)
            {
                minZ = plotData[i][j][2];
            }

            //find max
            if(plotData[i][j][0]>maxX)
            {
                maxX = plotData[i][j][0];
            }

            if(plotData[i][j][1]>maxY)
            {
                maxY = plotData[i][j][1];
            }

            if(plotData[i][j][2]>maxZ)
            {
                maxZ = plotData[i][j][2];
            }
        }
    }

    //find longest axis for scaling
    float maxLength = maxX-minX;

    if(maxY-minY > maxLength)
    {
        maxLength = maxY-minY;
    }
    if(maxZ-minZ > maxLength)
    {
        maxLength = maxZ-minZ;
    }
    scale = float(1)/maxLength;
    scaleX = float(1)/(maxX-minX);
    scaleY = float(1)/(maxY-minY);
    scaleZ = float(1)/(maxZ-minZ);

    int size = plotWidth * plotWidth;
    centreX = centreX/(float)size;
    centreY = centreY/(float)size;
    centreZ = centreZ/(float)size;
}

/*!
* \brief Sets plot data.
* \param[in] plotData - plot data
* \param[in] plotWidth - number of plot elements in each dimension
* \param[in] x - dimension id to be mapped to x axis
* \param[in] y - dimension id to be mapped to y axis
* \param[in] z - dimension id to be mapped to z axis
*/
void Plot3D::setData(QVector<float> plotData, int plotWidth, int x, int y, int z)
{
    disableRendering();
    weights = plotData;

    //dealocate memory before updating 'plWidth' parameter
    if(data)
    {
        for(int i=0; i<plWidth; i++)
        {
            for(int j=0; j<plWidth; j++)
            {
                delete [] data[i][j];
            }
            delete [] data[i];
        }
        delete [] data;
    }

    plWidth = plotWidth;
    int size = plWidth * plWidth;

    //allocate memory
    data = new float**[plWidth];
    for(int i=0; i<plWidth; i++)
    {
        data[i] = new float*[plWidth];

        for(int j=0; j<plWidth; j++)
        {
            data[i][j] = new float[3];
        }
    }

    int counter = 0;
    for(int i=0; i<plWidth; i++)
    {
        for(int j=0; j<plWidth; j++)
        {
            data[i][j][0] = plotData[counter+(x*size)];
            data[i][j][1] = plotData[counter+(y*size)];
            data[i][j][2] = plotData[counter+(z*size)];

            counter++;
        }
    }

    calculateCentre(data, plWidth);
    loadTexture(data);

    initialised = true;

    enableRendering();
    updateGL();
}

/*!
* \brief Updates plot data.
* \param[in] plotData - plot data
* \param[in] x - dimension id to be mapped to x axis
* \param[in] y - dimension id to be mapped to y axis
* \param[in] z - dimension id to be mapped to z axis
*/
void Plot3D::updateData(QVector<float> plotData, int x, int y, int z)
{
    weights = plotData;

    int counter = 0;
    int size = plWidth * plWidth;

    for(int i=0; i<plWidth; i++)
    {
        for(int j=0; j<plWidth; j++)
        {
            data[i][j][0] = plotData[counter+(x*size)];
            data[i][j][1] = plotData[counter+(y*size)];
            data[i][j][2] = plotData[counter+(z*size)];

            counter++;
        }
    }

    calculateCentre(data, plWidth);
    loadTexture(data);
    if(draw)
    {
        updateGL();
    }
}

/*!
* \brief Enables plot rendering.
*/
void Plot3D::enableRendering()
{
	if(initialised)
	{
		draw = true;
	}
}

/*!
* \brief Disables plot rendering.
*/
void Plot3D::disableRendering()
{
	draw = false;
}

/*!
* \brief Scales the input between any min and max values.
* \param[in] in      - a value to be scaled
* \param[in] oldMin  - previous minimum value
* \param[in] oldMax  - previous maximum value
* \param[in] newMin  - new minimum value
* \param[in] newMax  - new maximum value
* \return result  - scaled value
* \note This function takes the input max/min range, desired max/min range and converts the input accordingly.
*/
float Plot3D::scaleRange(float in, float oldMin, float oldMax, float newMin, float newMax)
{
    float scale = (oldMax-oldMin)/(newMax-newMin);
    float result = newMin+(in-oldMin)/scale;
    return result;
}
