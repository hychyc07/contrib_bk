#ifndef _DATA_PLOT_H
#define _DATA_PLOT_H 1

#include <qwt_plot.h>

//YARP
#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/dev/PreciselyTimed.h>
using namespace yarp::os;

//QWT
#include <stdlib.h>
#include <qwt_painter.h>
#include <qwt_plot_canvas.h>
#include <qwt_plot_marker.h>
#include <qwt_plot_curve.h>
#include <qwt_scale_widget.h>
#include <qwt_legend.h>
#include <qwt_scale_draw.h>
#include <qwt_math.h>

const int PLOT_SIZE = 201;      // 0 to 200
extern bool VERBOSE;

class DataPlot : public QwtPlot
{
    Q_OBJECT

public:
    DataPlot(QWidget* = NULL);
    ~DataPlot();
    void initSignalDimensions();
    void setInputPortName(const char *name);
    void setInputPortIndex(int **index, int *size, int n);
    void setPorts(ConstString *portNameList, int nPorts);

public slots:
    void setTimerInterval(double interval);
    void toggleAcquire();

protected:
    virtual void timerEvent(QTimerEvent *e);

private:
    void alignScales();

    bool acquire;
    bool realTime;
    int inputVectorSize;
    double initialTime;
    int numberAcquiredData;
    int numberOfInputPorts;
    int *numberOfPlots;
    int **index;
    char *remoteName;

    QwtPlotCurve **nonTimeBasedCurve;
    QwtPlotCurve *timeBasedCurve;

    double d_x[PLOT_SIZE]; 
    double ***d_y; 
    double d_x_real_time[PLOT_SIZE]; 

    int d_interval; // timer in ms
    int d_timerId;

    yarp::os::BufferedPort<yarp::os::Bottle> *inputPorts;
};

#endif
