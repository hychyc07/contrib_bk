#include "iCubSkinGui3DWidget.h"

#define SET_COLOR4(array,r,g,b,a)   {(array)[0] = (r); (array)[1] = (g); (array)[2] = (b); (array)[3] = (a);}

iCubSkinGui3DWidget::iCubSkinGui3DWidget(QWidget *parent)
:QGLWidget(parent) {

    setWindowTitle("iCubSkinGui3D");

    QGLFormat desiredFormat(QGL::DoubleBuffer|
                            QGL::DepthBuffer|
                            QGL::Rgba|
                            QGL::AlphaChannel|
                            QGL::StencilBuffer|
                            QGL::DirectRendering|
                            QGL::SampleBuffers);

    desiredFormat.setSamples(4);
    setFormat(desiredFormat);


    QTimer *mIdleTimer = new QTimer(this);
    connect(mIdleTimer, SIGNAL(timeout()), this, SLOT(OnIdle()));
    mIdleTimer->start(10);


    QTimer *myPaintQtimer = new QTimer(this);
    connect(myPaintQtimer, SIGNAL(timeout()), this, SLOT(OnPaint()));
    myPaintQtimer->start(33);

    mCurrentKey = 0;

    mGLWidth    = 0;
    mGLHeight   = 0;

    bCtrlModiferOn  = false;


    mCameraInitLookAt.Set(0,0,0);
    mCameraInitPos.Set(3,-3,1);

    Matrix3 id;
    id.Identity();
    id*=0.1;
    mCameraObject.GenerateSphere(16,12);
    mCameraObject.Transform(id);
    mCameraObjectLine.GenerateCube();
    id(0,0)*=0.2;
    id(1,1)*=0.2;
    id(2,2)*=200;
    mCameraObjectLine.Transform(id);

    mNumLights          = 2;

    SET_COLOR4(mLightColor[0][0],   0.5,0.5,0.5,1.0);
    SET_COLOR4(mLightColor[0][1],   0.5,0.5,0.5,1.0);
    SET_COLOR4(mLightColor[0][2],   0.0,0.0,0.0,1.0);
    SET_COLOR4(mLightPosition[0],    8.0, 8.0, 10.0,1.0);
    SET_COLOR4(mLightDirection[0],  -8.0,-8.0,-10.0,1.0);
    bUseLightDirection[0] = true;
    bUseLightPosition[0]  = true;

    SET_COLOR4(mLightColor[1][0],   0.0,0.0,0.0,1.0);
    SET_COLOR4(mLightColor[1][1],   0.3,0.3,0.3,1.0);
    SET_COLOR4(mLightColor[1][2],   0.0,0.0,0.0,1.0);
    SET_COLOR4(mLightPosition[1],  -8.0,-8.0, 10.0,1.0);
    SET_COLOR4(mLightDirection[1],  8.0, 8.0,-10.0,1.0);
    bUseLightDirection[1] = true;
    bUseLightPosition[1]  = true;

    bSkinModeReal = true;
    
    bShowNormal   = false;


}
iCubSkinGui3DWidget::~iCubSkinGui3DWidget(){
}

void    iCubSkinGui3DWidget::ShowNormal(bool show){
  bShowNormal = show;
}

bool    iCubSkinGui3DWidget::Load(const char* path){

    bool bOk = true;

    char txt[256];
    sprintf(txt,"%s/cover.obj",path);

    bOk &=  mCover.LoadFromObjFile(txt,true);
    if(!bOk) return false;
    bOk &=  mCoverBack.LoadFromObjFile(txt,false);
    if(!bOk) return false;

    mCover.mVertices.SumRow(mCenter);
    mCenter *= 1.0/(mCover.mVertices.RowSize());
    //mCenter.Print();

    //Matrix3 trans;
    //trans.RotationX(M_PI);
    //trans.Identity();
    //trans*=10.0;
    //mCover.Transform(trans);
    //mCoverBack.Transform(trans);



    Matrix A,B1,B2,C1,C2;



    sprintf(txt,"%s/sensorMap.txt",path);
    bOk &= A.Load(txt);

    if(!bOk) return false;

    int size = A.RowSize();

    A.GetColumns(0,2,B1);
    A.GetColumns(3,5,C1);
    mCoverPos.Resize(A.RowSize(),3);
    mCoverNormal.Resize(A.RowSize(),3);
    mCoverPos.SetRowSpace(B1,0);
    mCoverNormal.SetRowSpace(C1,0);

    mCoverIsValid.clear();
    mCoverIsValid.resize(size);
    for(int i=0;i<size;i++){
        mCoverIsValid[i] = mCoverNormal.GetRow(i).Norm2()>0.0;
    }
    mCoverScale.Resize(size,false);mCoverScale.Zero();

    mCoverInDistSet.resize(size);
    mCoverInDist.Resize(size,false);mCoverInDist.Zero();


    mSkinCluster.Init(mCoverPos,mCoverNormal,0.01,0,0.1);
    mSkinCluster.SetWeights(0.008,0.01);

    //mCoverPos*=10.0;


    mSphere.GenerateSphere(10,8);
    Matrix3 trans;
    trans.Identity();
    trans*=0.01;
    mSphere.Transform(trans);

    return true;
}

void    iCubSkinGui3DWidget::SetSkinValues(Vector &values){
    mMutex.lock();
    mCoverScale.SetSubVector(0,values);
    bSkinModeReal = true;
    mMutex.unlock();
}

void    iCubSkinGui3DWidget::SetObstaclePos(Matrix &pos){
    mMutex.lock();
    mObstaclesPos = pos;
    bSkinModeReal = false;
    mMutex.unlock();
}






void iCubSkinGui3DWidget::paintGL(){

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    glPushMatrix();
    glLoadIdentity();


    mCamera.Apply(false);
    
    glRotatef(90.0,0,1,0);

    //glScalef(10,10,10);
    glPushMatrix();
    glScalef(10,10,10);

    for(int i=0;i<mNumLights;i++){
        if(bUseLightPosition[i]){
            glLightfv(GL_LIGHT0+i,GL_POSITION,mLightPosition[i]);
        }
        if(bUseLightDirection[i]){
            glLightfv(GL_LIGHT0+i,GL_SPOT_DIRECTION,mLightDirection[i]);
            glLightf(GL_LIGHT0+i,GL_SPOT_CUTOFF,30.0);
        }
    }

    glTranslatef(-mCenter(0),-mCenter(1),-mCenter(2));


    int size = mCoverPos.RowSize();

    for(int i=0;i<size;i++){
        if(!mCoverIsValid[i])
            continue;

        glPushMatrix();
        glTranslatef(mCoverPos.AtNoCheck(i,0),
                     mCoverPos.AtNoCheck(i,1),
                     mCoverPos.AtNoCheck(i,2));
        glScalef(0.4,0.4,0.4);

        float color[3];
        double val = (mSkinCluster.GetOutput().AtNoCheck(i))*3.0;
        val = mCoverScale(i);//(mSkinCluster.GetInput().AtNoCheck(i)-0.2)*3.0;
        color[0] = 0;
        color[1] = (val>0.0?0.5+val*0.5:0);
        color[2] = (val>0.0?0.5+val*0.5:0);
        glColor4f(val,val,0,1);
        mSphere.Render();
	if(bShowNormal){
	  glBegin(GL_LINES);
	  glVertex3f(0,0,0);
	  glVertex3f(mCoverNormal.AtNoCheck(i,0)*0.05,
		     mCoverNormal.AtNoCheck(i,1)*0.05,
		     mCoverNormal.AtNoCheck(i,2)*0.05);
	  glEnd();
	  //glTranslatef(0.01*mCoverNormal.AtNoCheck(i,0),
	  //             0.01*mCoverNormal.AtNoCheck(i,1),
	  //             0.01*mCoverNormal.AtNoCheck(i,2));
	  //mSphere.Render();
	}
	glPopMatrix();
    }
    
    for(int i=0;i<mSkinCluster.GetClusters().size();i++){

        Vector mean = mSkinCluster.GetClusterPos().GetRow(i);
        Vector norm = mSkinCluster.GetClusterNormal().GetRow(i);
        //mean*=10.0;

        glPushMatrix();
        glTranslatef(mean.AtNoCheck(0),
                     mean.AtNoCheck(1),
                     mean.AtNoCheck(2));
        //glScalef(0.8,0.8,0.8);
        glColor4f(1,0,0,1);
        mSphere.Render();
        norm *= mSkinCluster.GetClusterSecondaryResponse()[i];
        //cout << i<<" "<<mSkinCluster.GetClusterSecondaryResponse()[i]<<endl;
        glTranslatef(norm.AtNoCheck(0),
                     norm.AtNoCheck(1),
                     norm.AtNoCheck(2));
        glScalef(0.8,0.8,0.8);
        glColor4f(1,1,1,1);
        mSphere.Render();
        glPopMatrix();
    }

    int count = mObstaclesPos.RowSize();
    for(int i=0;i<count;i++){
        glPushMatrix();
        glTranslatef(mObstaclesPos.AtNoCheck(i,0),
                     mObstaclesPos.AtNoCheck(i,1),
                     mObstaclesPos.AtNoCheck(i,2));
        glColor4f(0,1,0,0.5);
        mSphere.Render();
        glPopMatrix();
    }

    glColor4f(0.8,0.8,0.8,0.8);
    mCover.Render();
    mCoverBack.Render();


    glPopMatrix();
    if((mCurrentMouseButton != Qt::NoButton)){
        DrawCameraLookAt();
    }
    glPopMatrix();
    mBaseWindow.BaseRender();
}


void    iCubSkinGui3DWidget::GetClusterData(Vector & data){
    mMutex.lock();
    int n = mSkinCluster.GetClusters().size();
    data.Resize(7*n,false);
    Matrix3 trans;
    trans.Identity();
    //trans.RotationX(M_PI);
    //trans*=10.0;

    for(int i=0;i<n;i++){
        data.SetSubVector(i*7,  SharedMatrix(trans)*mSkinCluster.GetClusterPos().GetRow(i));
        data.SetSubVector(i*7+3,SharedMatrix(trans)*mSkinCluster.GetClusterNormal().GetRow(i));        
        data[i*7+6] = mSkinCluster.GetClusterSecondaryResponse()[i];
    }
    mMutex.unlock();   
}


void  iCubSkinGui3DWidget::OnPaint(){
  update();
}
void  iCubSkinGui3DWidget::OnIdle(){
    mMutex.lock();

    if(!bSkinModeReal){
        CheckVirtualContact();
    }

    mSkinCluster.SetInput(mCoverScale,mCoverInDist);
    mSkinCluster.Update(0.03);
    mSkinCluster.Prepare();
    mSkinCluster.Cluster();
    mSkinCluster.ComputeClusterPos();

    mMutex.unlock();
}


void iCubSkinGui3DWidget::resizeGL(int w, int h){
    mGLWidth    = w;
    mGLHeight   = h;

    mCamera.SetViewport(w,h);
    mCamera.Apply();

    mBaseWindow.BaseResize(w,h);
}

void iCubSkinGui3DWidget::initializeGL(){
    glEnable(GL_DEPTH_TEST);

    for(int i=0;i<mNumLights;i++){
        glLightfv(GL_LIGHT0+i, GL_AMBIENT,  mLightColor[i][0]);
        glLightfv(GL_LIGHT0+i, GL_DIFFUSE,  mLightColor[i][1]);
        glLightfv(GL_LIGHT0+i, GL_SPECULAR, mLightColor[i][2]);
        glEnable(GL_LIGHT0+i);
    }
    for(int i=mNumLights;i<MAX_NUM_LIGHTS;i++){
        glDisable(GL_LIGHT0+i);
    }
    if(mNumLights>0)
        glEnable(GL_LIGHTING);

    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);


    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);


    glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable (GL_LINE_SMOOTH);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);
    glShadeModel(GL_SMOOTH);

    glEnable(GL_TEXTURE_2D);

    glEnable(GL_NORMALIZE);
    glClearColor(0.25,0.25,0.45,1);


    float zero[]={0.0,0.0,0.0,1.0};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,zero);

    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations

    mCurrentMouseButton = Qt::NoButton;
    mCurrentMouseX=0;
    mCurrentMouseY=0;
    mCamera.SetPosition(mCameraInitPos);
    mCamera.SetLookAt(mCameraInitLookAt);
    mCamera.Move(0,0,0);
    mCamera.Apply();
}




void iCubSkinGui3DWidget::keyPressEvent ( QKeyEvent * event ){
    if(event->modifiers() & Qt::ControlModifier){
        bCtrlModiferOn = true;
    }else{
        if(event->text().size()>0){
            mCurrentKey = event->text().at(0).unicode();
            mBaseWindow.BaseInputNormalKey(mCurrentKey, 0,0);
        }
    }

    event->accept();

    QGLWidget::keyPressEvent(event);
}
void iCubSkinGui3DWidget::keyReleaseEvent (QKeyEvent * event){
    if(event->modifiers() & Qt::ControlModifier){
    }else{
        bCtrlModiferOn = false;
    }
}

char iCubSkinGui3DWidget::GetCurrentKey(){
  char res = mCurrentKey;
  mCurrentKey = 0;
  return res;
}


void iCubSkinGui3DWidget::mouseMoveEvent(QMouseEvent * event){
    mBaseWindow.BaseInputMouseMotion(event->x(), event->y());

    if(mCurrentMouseButton == Qt::LeftButton){
        mCamera.Move((float)(event->x()-mCurrentMouseX),-(float)(event->y()-mCurrentMouseY),0);
        mCurrentMouseX = event->x();
        mCurrentMouseY = event->y();
    }
    if(mCurrentMouseButton == Qt::RightButton){
        mCamera.Move(0,0,(float)(event->y()-mCurrentMouseY));
        mCurrentMouseX = event->x();
        mCurrentMouseY = event->y();
    }
}

void iCubSkinGui3DWidget::mousePressEvent(QMouseEvent * event){

  int res = 0;
  if(event->button() == Qt::LeftButton)
    res = mBaseWindow.BaseInputMouseButton(GLBW_LEFTBTN, GLBW_BTNDOWN, event->x(), event->y());
  if(event->button() == Qt::RightButton)
    res = mBaseWindow.BaseInputMouseButton(GLBW_RIGHTBTN, GLBW_BTNDOWN, event->x(), event->y());

  if(res) return;

    if(mCurrentMouseButton == Qt::NoButton){
        if(bCtrlModiferOn){
            mCameraMode = GLCamera::CAMMODE_FreeMove;
        }else{
            mCameraMode = GLCamera::CAMMODE_Centered;
        }
        mCamera.SetCameraMode(mCameraMode);

        mCurrentMouseButton = event->button();
        mCurrentMouseX      = event->x();
        mCurrentMouseY      = event->y();
    }
}

void iCubSkinGui3DWidget::mouseReleaseEvent(QMouseEvent * event){
  int res = 0;
  if(event->button() == Qt::LeftButton)
    res = mBaseWindow.BaseInputMouseButton(GLBW_LEFTBTN, GLBW_BTNUP, event->x(), event->y());
  if(event->button() == Qt::RightButton)
    res = mBaseWindow.BaseInputMouseButton(GLBW_RIGHTBTN, GLBW_BTNUP, event->x(), event->y());
  if(res) return;

  if(mCurrentMouseButton == event->button()){
    mCurrentMouseButton = Qt::NoButton;
  }
}

void  iCubSkinGui3DWidget::DrawCameraLookAt(){
    glColor4f(0,0.8,1,0.6);
    glPushMatrix();
    glTranslatef(mCamera.m_lookAtPoint[0],mCamera.m_lookAtPoint[1],mCamera.m_lookAtPoint[2]);
    mCameraObject.Render();
    mCameraObjectLine.Render();
    glRotatef(90,1,0,0);
    mCameraObjectLine.Render();
    glRotatef(90,0,1,0);
    mCameraObjectLine.Render();
    glPopMatrix();
}


void iCubSkinGui3DWidget::CheckVirtualContact(){
    mCoverScale.Zero();

    int count = mObstaclesPos.RowSize();
    int skinCount = mCoverPos.RowSize();

    for(int j=0;j<skinCount;j++)
        mCoverInDistSet[j].clear();

    mCoverInDist.Zero();

    Matrix npos;
    Matrix sqnpos;

    if(count>0){
        for(int i=0;i<count;i++){
            npos = mCoverPos;

            //npos.Print();
            Vector c(3);
            c(0) = -mObstaclesPos(i,0);
            c(1) = -mObstaclesPos(i,1);
            c(2) = -mObstaclesPos(i,2);
            npos.SAddToRow(c);
            npos.PMult(npos,sqnpos);
            Vector d;
            sqnpos.SumColumn(d);
            for(int j=0;j<skinCount;j++){
                if(d.AtNoCheck(j)<0.03*0.03){
                    Vector dist,norm;
                    npos.GetRow(j,dist);
                    mCoverNormal.GetRow(j,norm);
                    double dot = norm.Dot(dist);

                    norm.ScaleAddTo(-dot,dist);
                    double dist2 = dist.Norm2();
                    //cout << dot<<endl;
                    if(dot>0.0){
                        //cout << "-- " <<dot<<" " << dist.Norm2()<<endl;
                        if(dist2<0.01*0.01){
                            d.RefNoCheck(j) = dist2;
                        //if(dot/dist.Norm() > 0.86){ // cos(30);
                            mCoverInDistSet[j].insert(-dot);
                        }else{
                            d.RefNoCheck(j) = 100.0*100.0;
                        }
                        //cout << dot<<endl;
                    }else{
                        d.RefNoCheck(j) = 100.0*100.0;
                    }
                }else{
                    d.RefNoCheck(j) = 100.0*100.0;
                }
            }
            if(i==0){
                mCoverScale = d;
            }else{
                for(int j=0;j<skinCount;j++){
                    if(mCoverScale(j)>d(j))
                        mCoverScale(j) = d(j);
                }
            }
        }

        for(int j=0;j<skinCount;j++){
            if(!mCoverInDistSet[j].empty()){
                mCoverInDist(j) = - *(mCoverInDistSet[j].begin());
                //cout << j<<": "<<mCoverInDist(j)<<endl;
            }
            mCoverScale(j) = 0.5*exp(-mCoverScale(j) / (0.01*0.01));
        }
    }
}
