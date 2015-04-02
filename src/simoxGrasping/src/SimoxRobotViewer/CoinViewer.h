#ifndef __Coin_Viewer_h__
#define __Coin_Viewer_h__
/*!
* \class CoinViewer
*
* \author Nikolaus Vahrenkamp
*
* Copyright 2011 Nikolaus Vahrenkamp
*
* Extends the Coin3d/Qt SoQtExaminerViewer by adding mutex protection in order to allow safe 3D model updates within multi-threaded applications.
*/


#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/threads/SbMutex.h>
#include <boost/thread.hpp>

class CoinViewer : public SoQtExaminerViewer
{
public:
    
    CoinViewer(QWidget *parent=NULL, const char *name=NULL, SbBool embed=TRUE, SoQtFullViewer::BuildFlag flag=BUILD_ALL, SoQtViewer::Type type=BROWSER);
    virtual ~CoinViewer();

	//! use lock/unlock methods when accessing the scene graph (e.g. by updating joint values of a robot)
    void lock();
    void unlock();
    
protected:
    virtual void actualRedraw(void);
    
    SbThreadMutex mutex;
    //mutable boost::recursive_mutex mutex; 
    //boost::unique_lock<boost::recursive_mutex> _lock;
};

#endif
