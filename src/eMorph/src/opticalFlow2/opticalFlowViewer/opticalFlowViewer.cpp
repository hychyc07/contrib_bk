#include "opticalFlowViewer.h"

opticalFlowViewer::opticalFlowViewer()
{
    for (int x=0; x<XDIM; ++x)
    {
        for (int y=0; y<YDIM; ++y)
        {
            mMapVx[x][y]=0.0;
            mMapVy[x][y]=0.0;
        }
    }

    mBaseImg.resize(4*XDIM,4*YDIM);
    
    for(int x=0; x<4*XDIM; ++x)
    {
        for(int y=0; y<4*YDIM; ++y)
        {
            mBaseImg(x,y)=255;
        }
    }

    mPort.open("/image/opticalFlowFrame:o");

    mTimeStart=yarp::os::Time::now();
}

opticalFlowViewer::~opticalFlowViewer()
{
}

void opticalFlowViewer::onRead(vecBuffer& data)
{
    mTimeCurrent=yarp::os::Time::now();
    double timeDiff=mTimeCurrent-mTimeStart;

    int x=data.get_x();
    int y=data.get_y();

    double vx=data.get_vx();
    double vy=data.get_vy();

    //printf("%d  %d  %lf  %lf\n",x,y,vx,vy);
    //fflush(stdout);
    
    mMapVx[x][y]+=vx;
    mMapVy[x][y]+=vy;
    double norm;

    int hx,hy;

    if (timeDiff>=TNK_TIME)
    {
        yarp::sig::ImageOf<yarp::sig::PixelMono16>& img=mPort.prepare();
        img=mBaseImg;
        
        int X,Y;

        for (int x=0; x<XDIM; ++x)
        {
            for (int y=0; y<YDIM; ++y)
            {
                vx=mMapVx[x][y];
                vy=mMapVy[x][y];
                norm=vx*vx+vy*vy;

                if (norm>0.0)
                {
                    X=2+4*x;        
                    Y=2+4*y;
                    norm=15.0/sqrt(norm);

                    hx=X+int(norm*vx+0.5);
                    hy=511-Y-int(norm*vy+0.5);

                    static const yarp::sig::PixelMono16 black=0;
                    yarp::sig::draw::addSegment(img,black,X,511-Y,hx,hy);
                    yarp::sig::draw::addCircle(img,black,hx,hy,2);
                }

                mMapVx[x][y]=mMapVy[x][y]=0.0;
            }
        }
        
        mPort.write();
        
        mTimeStart=yarp::os::Time::now();
    }
}
