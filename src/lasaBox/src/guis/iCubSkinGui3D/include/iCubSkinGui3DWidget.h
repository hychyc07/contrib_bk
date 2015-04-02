#ifndef ICUBSKINGUI3DWIDGET_H_
#define ICUBSKINGUI3DWIDGET_H_

#include <QObject>
#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QTimer>
#include <QMutex>

#include "GLTools/GLTools.h"
#include "GLTools/GL3DObject.h"
#include "GLTools/GLCamera.h"
#include "GLTools/GLBaseWindow.h"
#include "MathLib/MathLib.h"

#include <vector>
#include <set>
using namespace std;

#include "SkinCluster.h"

#define MAX_NUM_LIGHTS 4

class iCubSkinGui3DWidget : public QGLWidget
{
    Q_OBJECT
protected:

    GLBaseWindow                mBaseWindow;
    int                         mGLWidth;
    int                         mGLHeight;
 
    GLCamera                    mCamera;
    Vector3                     mCameraInitPos;
    Vector3                     mCameraInitLookAt;

    Qt::MouseButton             mCurrentMouseButton;
    int                         mCurrentMouseX;
    int                         mCurrentMouseY;
    GLCamera::CameraMode        mCameraMode;

    GL3DObject                  mCameraObject;
    GL3DObject                  mCameraObjectLine;

    char                        mCurrentKey;

    bool                        bCtrlModiferOn;

    int                         mNumLights;
    float                       mLightColor[MAX_NUM_LIGHTS][3][4];
    bool                        bUseLightPosition[MAX_NUM_LIGHTS];
    float                       mLightPosition[MAX_NUM_LIGHTS][4];
    bool                        bUseLightDirection[MAX_NUM_LIGHTS];
    float                       mLightDirection[MAX_NUM_LIGHTS][4];



    Vector                      mCenter;

    GL3DObject                  mSphere;

    GL3DObject                  mCover;
    GL3DObject                  mCoverBack;

    Matrix                      mCoverPos;
    Matrix                      mCoverNormal;
    std::vector<bool>           mCoverIsValid;
    Vector                      mCoverScale;
    vector<set<double> >        mCoverInDistSet;
    Vector                      mCoverInDist;

    SkinCluster                 mSkinCluster;

    Matrix                      mObstaclesPos;

    bool                        bSkinModeReal;

    QMutex                      mMutex;

    bool                        bShowNormal;

public:
    iCubSkinGui3DWidget(QWidget *parent=NULL);
    ~iCubSkinGui3DWidget();

    bool    Load(const char* path);


    char    GetCurrentKey();
    void    DrawCameraLookAt();

    void    SetSkinValues(Vector & values);
    void    SetObstaclePos(Matrix & pos);
    void    GetClusterData(Vector & data);

    void    CheckVirtualContact();

    void    ShowNormal(bool show);

public slots:
    void    OnIdle();
    void    OnPaint();

protected:
  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();


  virtual void keyPressEvent (QKeyEvent * event);
  virtual void keyReleaseEvent (QKeyEvent * event);
  virtual void mouseMoveEvent(QMouseEvent * event);
  virtual void mousePressEvent(QMouseEvent * event);
  virtual void mouseReleaseEvent(QMouseEvent * event);

};

#endif

