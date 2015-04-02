
#include "SimoxRobotViewer.h"
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/SceneObject.h>
#include <VirtualRobot/Scene.h>
#include <VirtualRobot/Grasping/GraspSet.h>
#include <VirtualRobot/RobotNodeSet.h>
#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/ObjectIO.h>
#include <VirtualRobot/XML/SceneIO.h>
#include <MotionPlanning/Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h>
#include <MotionPlanning/CSpace/CSpaceSampled.h>
#include <Inventor/nodes/SoTranslation.h>

#include "ui_SimoxRobotViewer.h"

using namespace VirtualRobot;
using namespace yarp::os;

SimoxRobotViewer::SimoxRobotViewer(const std::string &title,yarp::os::ResourceFinder &rf)
    :QMainWindow(NULL)
{
    pingValue = 0;
    visualisationTypeRobot = VirtualRobot::SceneObject::Full;
    visualizationRobotEnabled = true;
    viewer = NULL;
    sceneSep = new SoSeparator();
    sceneSep->ref();
    robotSep = new SoSeparator();
    robotSep->ref();
    reachSep = new SoSeparator();
    reachSep->ref();
    isClosed = false;
    enableViewer = true;

    setupUI(title);

    if (rf.check("SimoxDataPath"))
    {
        ConstString dataPath=rf.find("SimoxDataPath").asString();
        VR_INFO << "Adding rf.SimoxDataPath: " << dataPath.c_str() << endl;
        VirtualRobot::RuntimeEnvironment::addDataPath(dataPath.c_str());
    }
    if (rf.check("RobotFile"))
    {

        ConstString robotFile=rf.find("RobotFile").asString();
        std::string robFileComplete = robotFile.c_str();
        if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(robFileComplete))
        {
            VR_ERROR << " Could not find file: " << robFileComplete << endl;
        } else
        {
            std::cout << "Using robot file: " << robFileComplete << endl;
            loadRobot(robFileComplete);
        }
    }
    if (!robot)
        VR_INFO << "No robot..." << endl;

    if (robot && rf.check("EndEffector"))
    {
        std::string eef = rf.find("EndEffector").asString().c_str();
        VR_INFO << "Selecting rf.EndEffector: " << eef << endl;
        selectEEF(eef);
    }

    if (robot && !currentEEF)
    {
        // select first eef

        std::vector<EndEffectorPtr> eefs;
        robot->getEndEffectors(eefs);
        if (eefs.size()>0)
        {
            VR_INFO << "Selecting first EEF: " << eefs[0]->getName() << endl;
            selectEEF(eefs[0]->getName());
        }
    }
    if (!currentEEF)
        VR_INFO << "Skipping EEF definition..." << endl;

    if (robot && currentEEF && rf.check("ReachabilityFile"))
    {
        std::string reachFile = rf.find("ReachabilityFile").asString().c_str();
        std::string reachFileComplete = reachFile.c_str();
        if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(reachFileComplete))
        {
            VR_ERROR << " Could not find file: " << reachFileComplete << endl;
        } else
            loadReachability(reachFileComplete);
    }
    if (!reachSpace)
        VR_INFO << "Skipping reachability information..." << endl;
    if (viewer)
        viewer->viewAll();
    SoQt::show(this);
}


SimoxRobotViewer::SimoxRobotViewer(const std::string &title)
    :QMainWindow(NULL)
{
    pingValue = 0;
    visualisationTypeRobot = VirtualRobot::SceneObject::Full;
    visualizationRobotEnabled = true;
    viewer = NULL;
    sceneSep = new SoSeparator();
    sceneSep->ref();
    robotSep = new SoSeparator();
    robotSep->ref();
    reachSep = new SoSeparator();
    reachSep->ref();
    isClosed = false;
    enableViewer = true;

    setupUI(title);

    if (viewer)
        viewer->viewAll();
    SoQt::show(this);
}

SimoxRobotViewer::~SimoxRobotViewer()
{
    while (objects.size()>0)
    {
        cout << "Removing object " << objects.begin()->first << endl;
        removeObject(objects.begin()->first);
    }
    while (eefVisu.size()>0)
    {
        cout << "Removing eef visu " << eefVisu.begin()->first << endl;
        removeEEFVisu(eefVisu.begin()->first);
    }

    if (viewer)
        quit();

    delete UI;
}

bool SimoxRobotViewer::selectEEF(const std::string &eef)
{
    if (!robot || !robot->hasEndEffector(eef))
    {
        VR_ERROR << "Robot does not have EEF with name " << eef << endl;
        return false;
    }
    currentEEF = robot->getEndEffector(eef);
    return true;
}

bool SimoxRobotViewer::selectRNS(const std::string &rns)
{
    if (!robot || !robot->hasRobotNodeSet(rns))
    {
        VR_ERROR << "Robot does not have RNS with name " << rns << endl;
        return false;
    }
    currentRNS = robot->getRobotNodeSet(rns);
    if (!currentRNS->isKinematicChain())
    {
        VR_WARNING << "RNS " << rns << " is not a strictly defined kinematic chain!" << endl;
    }
    return true;
}

bool SimoxRobotViewer::loadReachability( const std::string &filename )
{
	lock();
    reachSep->removeAllChildren();
    reachSpace.reset();
	unlock();
    if (!robot)
        return false;
    VR_INFO << "Loading reachability data from " << filename << endl;

    ReachabilityPtr rs(new Reachability(robot));
    try
    {
        rs->load(filename);
    }
    catch (VirtualRobotException &e)
    {
        VR_ERROR << " Could not load reachability space" << endl;
        VR_ERROR << e.what();

        return false;
    }
    if (!rs)
    {
        VR_ERROR << " ERROR while creating reachability data" << endl;
        return false;
    }
    return setReachability(rs);
}


bool SimoxRobotViewer::setReachability( VirtualRobot::ReachabilityPtr reachabilityData )
{
    if (!reachabilityData)
    {
		lock();
        reachSpace.reset();
        reachSep->removeAllChildren();
		unlock();
        return false;
    }
    reachSpace = reachabilityData;
    reachSpace->print();
    if (reachSpace->getNodeSet())
    {
        VR_INFO << "Using RNS: " << reachSpace->getNodeSet()->getName() << endl;
        if (!selectRNS(reachSpace->getNodeSet()->getName()))
        {
            VR_ERROR << "Could not select ReachSpace RNS: " << reachSpace->getNodeSet()->getName() << endl;
            reachSpace.reset();
            return false;
        }
        if (!currentRNS || !currentRNS->getTCP())
        {
            VR_ERROR << "Could not select ReachSpace RNS (or TCP not present): " << reachSpace->getNodeSet()->getName() << endl;
            reachSpace.reset();
            return false;
        }

        if (currentRNS->getTCP()->getName() != currentEEF->getTcpName())
        {
            VR_ERROR << "Expecting reachability space's tcp (" << currentRNS->getTCP()->getName() << ") == currentEEF's tcp (" << currentEEF->getTcpName() << ")" << endl;
            reachSpace.reset();
            return false;
        }
    }

    if (!viewer)
        return true;

    // force redraw
    lock();
    if (sceneSep->findChild(reachSep)>=0)
        sceneSep->removeChild(reachSep);
    unlock();
    redraw();
    return true;
}

bool SimoxRobotViewer::loadRobot( const std::string &filename )
{
    lock();
    robotSep->removeAllChildren();
    VR_INFO << "Loading robot from " << filename << endl;
    RobotPtr r;
    try
    {
        r = RobotIO::loadRobot(filename);
    }
    catch (VirtualRobotException &e)
    {
        VR_ERROR << " ERROR while creating robot" << endl;
        VR_ERROR << e.what();
        robot.reset();
		unlock();
        return false;
    }

    if (!r)
    {
        VR_ERROR << " ERROR while creating robot" << endl;
		unlock();
        return false;
    }
	unlock();
    return setRobot(r);
}


bool SimoxRobotViewer::setRobot( VirtualRobot::RobotPtr robot )
{
    lock();
    robotSep->removeAllChildren();
    unlock();
    if (!robot)
        return false;
    lock();
    this->robot = robot;
    unlock();
    if (!viewer)
        return true;

	//cout << "setting robot" << endl;
	//robot->print();
	
    // force redraw
    lock();
    if (sceneSep->findChild(robotSep)>=0)
        sceneSep->removeChild(robotSep);
    visualisationTypeRobot = SceneObject::Collision; // this forces to redraw
    unlock();
    showRobot(visualizationRobotEnabled,SceneObject::Full);
    return true;
}

void SimoxRobotViewer::showRobot( bool enable, VirtualRobot::SceneObject::VisualizationType visu /*= VirtualRobot::SceneObject::Full*/ )
{
    if (!viewer)
        return;
    lock();

    // check if we need to re-create the model
    if (visu!=visualisationTypeRobot && sceneSep->findChild(robotSep)>=0)
        sceneSep->removeChild(robotSep);

    if (enable && sceneSep->findChild(robotSep)<0)
    {
        sceneSep->addChild(robotSep);
        if (visu!=visualisationTypeRobot && robot)
        {
           robotSep->removeAllChildren();

            SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(robot,visu);
            if (visualisationNode)
            {
				robotSep->addChild(visualisationNode);
			}
        }
    }

    if (!enable && sceneSep->findChild(robotSep)>=0)
    {
       sceneSep->removeChild(robotSep);
    }
    visualisationTypeRobot = visu;
    visualizationRobotEnabled = enable;
    unlock();
    redraw();
}

void SimoxRobotViewer::setupUI(const std::string &title)
{
    UI = new Ui::MainWindowSimoxRobotViewer;

    UI->setupUi(this);
    //UI->centralwidget->setWindowTitle(QString(title.c_str()));
    this->setWindowTitle(QString(title.c_str()));
    viewer = new CoinViewer(UI->frameViewer,"",TRUE,SoQtExaminerViewer::BUILD_POPUP);

    // setup
    viewer->setBackgroundColor(SbColor(1.0f, 1.0f, 1.0f));
    viewer->setAccumulationBuffer(true);
#ifdef WIN32
#ifndef _DEBUG
    VR_INFO << "Enabling anti-aliasing..." << endl;
    viewer->setAntialiasing(true, 4);
#endif
#endif
    viewer->setGLRenderAction(new SoLineHighlightRenderAction);
    viewer->setTransparencyType(SoGLRenderAction::BLEND);
    viewer->setFeedbackVisibility(true);
    viewer->setSceneGraph(sceneSep);
    viewer->viewAll();
}


void SimoxRobotViewer::closeEvent(QCloseEvent *event)
{
    quit();
    QMainWindow::closeEvent(event);
}


void SimoxRobotViewer::quit()
{
    std::cout << "SimoxRobotViewer: Closing" << std::endl;
    this->close();
    if (sceneSep)
        sceneSep->unref();
    if (robotSep)
        robotSep->unref();
    if (reachSep)
        reachSep->unref();
    sceneSep = reachSep = robotSep = NULL;
    delete viewer;
    viewer = NULL;
    isClosed = true;
    SoQt::exitMainLoop();
}


int SimoxRobotViewer::main()
{
    SoQt::show(this);
    SoQt::mainLoop();
    return 0;
}

bool SimoxRobotViewer::setJoints( const std::string &robotNodeSet, std::vector<float> jointValues )
{
    if (!robot)
        return false;
	lock();
    if (!robot->hasRobotNodeSet(robotNodeSet))
    {
        VR_ERROR << "No RNS with name " << robotNodeSet << " found..." << endl;
		unlock();
        return false;
    }
    RobotNodeSetPtr rns = robot->getRobotNodeSet(robotNodeSet);
    if (rns->getSize() != jointValues.size())
    {
        VR_ERROR << "RNS with name " << robotNodeSet << " covers " << rns->getSize() << " joints, but the given joint vector is of size " << jointValues.size() << endl;
		unlock();
        return false;
    }

    rns->setJointValues(jointValues);
    unlock();
    redraw();
    return true;
}

void SimoxRobotViewer::showViewer( bool enable )
{
    // don't send event directly, since we might be called from another thread!
    enableViewer = enable;
}

void SimoxRobotViewer::ping()
{
    if (this->isHidden() && enableViewer)
        this->show();
    if (!this->isHidden() && !enableViewer)
        this->hide();
    pingValue++;
    if (pingValue>5)
    {
        pingValue = 0;
        UI->dial->setValue( (UI->dial->value() + 10) % (UI->dial->maximum()+1));
        if (viewer)
            viewer->scheduleRedraw();
        //UI.radioButton->setChecked(!UI.radioButton->isChecked());
    }
}

bool SimoxRobotViewer::wasClosed()
{
    return isClosed;
}

bool SimoxRobotViewer::setObject( const std::string &objectName, std::string filename, VirtualRobot::ColorPtr c )
{
    removeObject(objectName);

    if (filename=="")
        return false;

    if (!VirtualRobot::RuntimeEnvironment::getDataFileAbsolute(filename))
    {
        VR_WARNING << "Could not find file: " << filename;
        return false;
    }
	lock();
    ManipulationObjectPtr o = ObjectIO::loadManipulationObject(filename);
	unlock();
    if (!o)
    {
        VR_WARNING << "Could not load object from file: " << filename;
        return false;
    }
    return setObject(objectName,o,c);
}

bool SimoxRobotViewer::removeObject( const std::string &objectName)
{
	lock();
	bool objRemove = objects.find(objectName) != objects.end();
    if (objRemove)
    {
        VR_INFO << "removing object " << objectName << endl;
		unlock();
        showObject(objectName,false); // disable visualization
		lock();
        objects[objectName].object.reset();
        objects[objectName].visuGrasps->unref();
        objects[objectName].visuObject->unref();
        objects[objectName].visuReachableGrasps->unref();
        objects.erase(objectName);
    }
	unlock();
    return objRemove;
}

bool SimoxRobotViewer::setObject( const std::string &objectName, VirtualRobot::ManipulationObjectPtr object, VirtualRobot::ColorPtr c )
{
    removeObject(objectName);
    if (!object)
        return false;

    lock();

    // setting object's name to the one we use within the viewer
    object->setName(objectName);

    objectData d;
    d.object = object;
    d.c = c;
    d.visuObject = new SoSeparator;
    d.visuObject->ref();
    d.visuGrasps = new SoSeparator;
    d.visuGrasps->ref();
    Eigen::Matrix4f pose = object->getGlobalPose();
    SoMatrixTransform *mat = CoinVisualizationFactory::getMatrixTransformM(pose);
    d.visuGrasps->addChild(mat);
    d.visuReachableGrasps = new SoSeparator;
    d.visuReachableGrasps->ref();
    SoMatrixTransform *mat2 = CoinVisualizationFactory::getMatrixTransformM(pose);
    d.visuReachableGrasps->addChild(mat2);
    objects[objectName] = d;
    unlock();
    showObject(objectName,true);
    return true;
}

bool SimoxRobotViewer::showObject( const std::string &objectName, bool enable /*= true*/ )
{
	if (!sceneSep)
		return false;
	lock();
    if (objects.find(objectName) == objects.end())
    {
        VR_INFO << "Do not know object " << objectName << endl;
		unlock();
        return false;
    }


    if (enable)
    {
        if (sceneSep->findChild(objects[objectName].visuObject)<0)
        {
            if (objects[objectName].visuObject->getNumChildren()==0)
            {
                // create visu
                SoNode *sep = CoinVisualizationFactory::getCoinVisualization(objects[objectName].object,VirtualRobot::SceneObject::Full);
                if (objects[objectName].c)
                    sep = CoinVisualizationFactory::Colorize(sep,*(objects[objectName].c));
                objects[objectName].visuObject->addChild(sep);
            }
            sceneSep->addChild(objects[objectName].visuObject);
        }
    }

    if (!enable)
    {
        // remove all visus
        if (sceneSep->findChild(objects[objectName].visuObject)>=0)
        {
            sceneSep->removeChild(objects[objectName].visuObject);
        }
        if (sceneSep->findChild(objects[objectName].visuGrasps)>=0)
        {
            sceneSep->removeChild(objects[objectName].visuGrasps);
        }
        if (sceneSep->findChild(objects[objectName].visuReachableGrasps)>=0)
        {
            sceneSep->removeChild(objects[objectName].visuReachableGrasps);
        }
    }
	unlock();

    redraw();
    return true;
}


bool SimoxRobotViewer::showGrasps( const std::string &objectName, bool enableAll, bool enableReachable )
{
	if (!sceneSep)
		return false;

	lock();
    if (objects.find(objectName) == objects.end())
    {
        VR_INFO << "Do not know object " << objectName << endl;
		unlock();
        return false;
    }
    if (!robot || !currentEEF)
    {
        VR_INFO << "No EEF or robot, could not visualize grasps for object " << objectName << endl;
		unlock();
        return false;
    }

    GraspSetPtr grasps = objects[objectName].object->getGraspSet(currentEEF);



    if (enableAll)
    {
        if (sceneSep->findChild(objects[objectName].visuGrasps)<0)
        {
            if (objects[objectName].visuGrasps->getNumChildren()==1)
            {
                // create visu
                objects[objectName].visuGrasps->addChild( CoinVisualizationFactory::CreateGraspSetVisualization(grasps, currentEEF, Eigen::Matrix4f::Identity(), SceneObject::Full) );
            }
            sceneSep->addChild(objects[objectName].visuGrasps);
        }

    }

    if (!enableAll)
    {
        // remove all except the first SoMatrixTransform
        while (objects[objectName].visuReachableGrasps->getNumChildren()>=2)
            objects[objectName].visuReachableGrasps->removeChild(1);

        // remove grasps visu
        if (sceneSep->findChild(objects[objectName].visuGrasps)>=0)
        {
            sceneSep->removeChild(objects[objectName].visuGrasps);
        }
    }

    if (reachSpace && enableReachable)
    {
        if (sceneSep->findChild(objects[objectName].visuReachableGrasps)<0)
        {
            if (objects[objectName].visuReachableGrasps->getNumChildren()==1)
            {
                // create visu
                GraspSetPtr rg = reachSpace->getReachableGrasps(grasps,objects[objectName].object);
                objects[objectName].visuReachableGrasps->addChild( CoinVisualizationFactory::CreateGraspSetVisualization(rg, currentEEF, Eigen::Matrix4f::Identity()) );
            }
            sceneSep->addChild(objects[objectName].visuReachableGrasps);
        }
    }

    if (!enableReachable)
    {
        // remove grasps visu
        if (sceneSep->findChild(objects[objectName].visuReachableGrasps)>=0)
        {
            sceneSep->removeChild(objects[objectName].visuReachableGrasps);
        }
    }
    unlock();
	redraw();
    return true;
}

bool SimoxRobotViewer::showGrasps( const std::string &objectName, GraspSetPtr grasps, bool enable )
{
    if (!sceneSep)
        return false;

    lock();
    if (objects.find(objectName) == objects.end())
    {
        VR_INFO << "Do not know object " << objectName << endl;
		unlock();
        return false;
    }
    // remove grasps visu
    if (sceneSep->findChild(objects[objectName].visuReachableGrasps)>=0)
    {
        sceneSep->removeChild(objects[objectName].visuReachableGrasps);
    }

    if (enable)
    {
        if (sceneSep->findChild(objects[objectName].visuReachableGrasps)<0)
        {
            // create visu
            objects[objectName].visuReachableGrasps->addChild( CoinVisualizationFactory::CreateGraspSetVisualization(grasps, currentEEF, Eigen::Matrix4f::Identity(), SceneObject::Full) );
            sceneSep->addChild(objects[objectName].visuReachableGrasps);
        }
    }
	unlock();
    redraw();
    return true;
}


bool SimoxRobotViewer::showGrasp( const std::string &objectName, VirtualRobot::GraspPtr grasp, bool enable )
{
	std::vector<VirtualRobot::GraspPtr> grasps;
	grasps.push_back(grasp);
	return showGrasp(objectName,grasps,enable);
}

bool SimoxRobotViewer::showGrasp( const std::string &objectName, std::vector<VirtualRobot::GraspPtr> grasps, bool enable )
{
    if (!sceneSep)
        return false;

    lock();
    if (objects.find(objectName) == objects.end())
    {
        VR_INFO << "Do not know object " << objectName << endl;
		unlock();
        return false;
    }
    // remove grasps visu
    if (sceneSep->findChild(objects[objectName].visuReachableGrasps)>=0)
    {
        // remove all except the first SoMatrixTransform
        while (objects[objectName].visuReachableGrasps->getNumChildren()>=2)
            objects[objectName].visuReachableGrasps->removeChild(1);
        sceneSep->removeChild(objects[objectName].visuReachableGrasps);
    }

    if (enable)
    {
		for (size_t i=0;i<grasps.size();i++)
		{
				// create visu
		        //cout << "objects[objectName].visuReachableGrasps:" << objects[objectName].visuReachableGrasps->getNumChildren() << endl;
			    objects[objectName].visuReachableGrasps->addChild( CoinVisualizationFactory::CreateGraspVisualization(grasps[i], currentEEF, Eigen::Matrix4f::Identity(), SceneObject::Full) );
				//cout << "objects[objectName].visuReachableGrasps:" << objects[objectName].visuReachableGrasps->getNumChildren() << endl;
		}
		if (sceneSep->findChild(objects[objectName].visuReachableGrasps)<0)
		{
			sceneSep->addChild(objects[objectName].visuReachableGrasps);
		}
    }
    unlock();
	redraw();
    return true;
}

bool SimoxRobotViewer::showGrasp( const std::string &objectName, const std::vector<std::string> &graspNames, bool enable )
{
	//GraspPtr g;
	lock();
	if (objects.find(objectName) == objects.end())
    {
        VR_INFO << "Do not know object " << objectName << endl;
		unlock();
        return false;
    }
	VirtualRobot::GraspSetPtr gs = objects[objectName].object->getGraspSet(currentEEF);
	std::vector< VirtualRobot::GraspPtr > gr;
	for (size_t i=0;i<graspNames.size();i++)
	{
		if (gs && gs->hasGrasp(graspNames[i]))
		{
			gr.push_back(gs->getGrasp(graspNames[i]));
		}
	}
	unlock();
	return showGrasp(objectName,gr,enable);
}

bool SimoxRobotViewer::showGrasp( const std::string &objectName, const std::string &graspName, bool enable )
{
	std::vector<std::string> grasps;
	grasps.push_back(graspName);
	return showGrasp(objectName,grasps,enable);
}

void SimoxRobotViewer::redraw()
{
    lock();
    if (viewer)
        viewer->scheduleRedraw();
    this->update();
    if (viewer)
        viewer->scheduleRedraw();
    unlock();
    //this->activateWindow();
}

bool SimoxRobotViewer::setObjectPose( const std::string &objectName, const Eigen::Matrix4f &pose, const std::string coordSystem )
{
    if (!sceneSep)
        return false;

    lock();

	Eigen::Matrix4f p = pose;
	if (objects.find(objectName) == objects.end())
    {
        VR_INFO << "Do not know object " << objectName << endl;
		unlock();
        return false;
    }

	if (!coordSystem.empty() && robot && robot->hasRobotNode(coordSystem))
	{
		RobotNodePtr rn = robot->getRobotNode(coordSystem);
		if (rn)
			p = rn->toGlobalCoordinateSystem(p);
	}

    objects[objectName].object->setGlobalPose(p);
    SoNode *n = objects[objectName].visuGrasps->getChild(0);
    SoMatrixTransform *mat = dynamic_cast<SoMatrixTransform*>(n);
    if (mat)
        mat->matrix.setValue(CoinVisualizationFactory::getSbMatrix(p));
    n = objects[objectName].visuReachableGrasps->getChild(0);
    mat = dynamic_cast<SoMatrixTransform*>(n);
    if (mat)
        mat->matrix.setValue(CoinVisualizationFactory::getSbMatrix(p));
    // delete reachable grasps visu, it may not be valid any more
    if (objects[objectName].visuReachableGrasps->getNumChildren()==2)
        objects[objectName].visuReachableGrasps->removeChild(1);
    if (sceneSep->findChild(objects[objectName].visuReachableGrasps)>=0)
        sceneSep->removeChild(objects[objectName].visuReachableGrasps);
    unlock();
    redraw();
    return true;
}

Eigen::Matrix4f SimoxRobotViewer::getObjectPose( const std::string &objectName, const std::string coordSystem )
{
 	lock();
    if (objects.find(objectName) == objects.end())
    {
        VR_INFO << "Do not know object " << objectName << endl;
		unlock();
        return Eigen::Matrix4f::Identity();
    }
    Eigen::Matrix4f r = objects[objectName].object->getGlobalPose();
	if (!coordSystem.empty() && robot && robot->hasRobotNode(coordSystem))
	{
		RobotNodePtr rn = robot->getRobotNode(coordSystem);
		if (rn)
			r = rn->toLocalCoordinateSystem(r);
	}
	unlock();
	return r;
}

void SimoxRobotViewer::showReachability( bool enable, const Eigen::Vector3f &tcpDir )
{
    if (!sceneSep)
        return;

    if (enable && reachSpace)
    {
        lock();
        if (sceneSep->findChild(reachSep)<0)
        {
            if (reachSep->getNumChildren()==0)
            {
                VR_INFO << "Building reachability visualization. This may take a while...";
#if 1
                // create visu
                SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace,VirtualRobot::ColorMap::eRed,tcpDir,true);
#endif
#if 0
                // different visualization
                SoNode* visualisationNode = CoinVisualizationFactory::getCoinVisualization(reachSpace,VirtualRobot::ColorMap::eRed,true);
#endif
                if (visualisationNode)
                    reachSep->addChild(visualisationNode);
                cout << " finished." << endl;
            }
            sceneSep->addChild(reachSep);
        }
        unlock();
    }

    if (!enable)
    {
        lock();
        // remove grasps visu
        if (sceneSep->findChild(reachSep)>=0)
        {
            sceneSep->removeChild(reachSep);
        }
        unlock();
    }
    redraw();
}

void SimoxRobotViewer::viewAll()
{
    if (viewer)
    {
        viewer->viewAll();
    }
}

void SimoxRobotViewer::lock()
{
    if (viewer)
        viewer->lock();
}

void SimoxRobotViewer::unlock()
{
    if (viewer)
        viewer->unlock();
}

bool SimoxRobotViewer::showCoordSystem( const std::string &jointName, bool enable )
{
    if (!robot)
        return false;
    lock();
	if (!robot->hasRobotNode(jointName))
    {
        VR_ERROR << "Robot does not know RobotNode with name " << jointName << endl;
		unlock();
        return false;
    }
    RobotNodePtr rn = robot->getRobotNode(jointName);
    if (!rn)
	{
		unlock();
        return false;
	}

    rn->showCoordinateSystem(enable);

    unlock();
    setRobot(robot);
    return true;
}

bool SimoxRobotViewer::setJointLimit( const std::string &robotNode, float min, float max )
{
	if (!robot)
	{
		VR_ERROR << "No robot..." << endl;
		return false;
	}
	lock();
	RobotNodePtr rn = robot->getRobotNode(robotNode);
	if (!rn)
	{
		VR_ERROR << "No robot node with name " << robotNode << endl;
		unlock();
		return false;
	}

	rn->setJointLimits(min,max);
	unlock();
	return true;
}
bool SimoxRobotViewer::setJointLimits( const std::string &robotNodeSet, std::vector<float> &min, std::vector<float> &max )
{
	if (!robot)
	{
		VR_ERROR << "No robot..." << endl;
		return false;
	}
	lock();
	RobotNodeSetPtr rns = robot->getRobotNodeSet(robotNodeSet);
	if (!rns)
	{
		VR_ERROR << "No robot node set with name " << robotNodeSet << endl;
		unlock();
		return false;
	}
	if (rns->getSize()!=min.size() || rns->getSize()!=max.size())
	{
		VR_ERROR << "Wrong sizes. RNS (" << robotNodeSet << ") : " << rns->getSize() <<", minSize:" << min.size() << ", maxSize:" << max.size() << endl;
		unlock();
		return false;
	}
	for (size_t i=0;i<min.size();i++)
	{
		RobotNodePtr rn = rns->getNode(i);
		if (!rn)
			return false;
		rn->setJointLimits(min[i],max[i]);
	}
	unlock();
	return true;
}

bool SimoxRobotViewer::selectEEFPreshape( const std::string &eefPreshape )
{
    if (!currentEEF || !viewer)
        return false;
    lock();
    bool res = currentEEF->setPreshape(eefPreshape);
    unlock();
    redraw();
    return res;
}

bool SimoxRobotViewer::setEEFVisu( const std::string &eefVisuName, const std::string &eef, SceneObject::VisualizationType visu )
{
    removeEEFVisu(eefVisuName);

    if (eef=="" || !robot)
        return false;
	lock();
    EndEffectorPtr eefp = robot->getEndEffector(eef);

    if (!eefp)
	{
		unlock();
        return false;
	}
    SoSeparator *sepEEF = CoinVisualizationFactory::CreateEndEffectorVisualization(eefp,visu);
    unlock();
    return setEEFVisu(eefVisuName,sepEEF);
}


bool SimoxRobotViewer::setEEFVisu( const std::string &eefVisuName, SoSeparator *vis )
{
    removeEEFVisu(eefVisuName);
    if (!vis)
        return false;
    lock();
    SoSeparator *v = new SoSeparator;
    SoMatrixTransform *mat = new SoMatrixTransform;
    mat->matrix.setValue(SbMatrix::identity());
    v->addChild(mat);
    v->addChild(vis);
    eefVisu[eefVisuName] = v;
    eefVisu[eefVisuName]->ref();
    unlock();
    showEEFVisu(eefVisuName,true);
    return true;
}

bool SimoxRobotViewer::removeEEFVisu( const std::string &eefVisuName )
{
    if (eefVisu.find(eefVisuName) != eefVisu.end())
    {
        VR_INFO << "removing eef visu " << eefVisuName << endl;
        showEEFVisu(eefVisuName,false); // disable visualization
        lock();
        eefVisu[eefVisuName]->unref();
        eefVisu.erase(eefVisuName);
        unlock();
        return true;
    }
    return false;
}

bool SimoxRobotViewer::showEEFVisu( const std::string &eefVisuName, bool enable /*= true*/ )
{
	if (!sceneSep)
		return false;
    
    lock();
	if (eefVisu.find(eefVisuName) == eefVisu.end())
    {
        VR_INFO << "Do not know eef visu " << eefVisuName << endl;
        return false;
    }
  
    if (enable)
    {
        if (sceneSep->findChild(eefVisu[eefVisuName])<0)
        {
            sceneSep->addChild(eefVisu[eefVisuName]);
        }
    }

    if (!enable)
    {
        // remove visus
        if (sceneSep->findChild(eefVisu[eefVisuName])>=0)
        {
            sceneSep->removeChild(eefVisu[eefVisuName]);
        }
    }
    unlock();
	redraw();
    return true;
}

bool SimoxRobotViewer::setEEFVisuPose( const std::string &eefVisuName, const Eigen::Matrix4f &pose )
{
    lock();
    if (eefVisu.find(eefVisuName) == eefVisu.end())
    {
        VR_INFO << "Do not know eef visu " << eefVisuName << endl;
		unlock();
        return false;
    }
 
    SoNode *n = eefVisu[eefVisuName]->getChild(0);
    SoMatrixTransform *mat = dynamic_cast<SoMatrixTransform*>(n);
    if (mat)
        mat->matrix.setValue(CoinVisualizationFactory::getSbMatrix(pose));
    unlock();
    redraw();
    return true;
}

bool SimoxRobotViewer::showObjectCoordSystem( const std::string &objectName, bool enable )
{
    lock();
	if (objects.find(objectName) == objects.end())
    {
        VR_INFO << "Do not know object " << objectName << endl;
		unlock();
        return false;
    }

    objects[objectName].object->showCoordinateSystem(enable);
    unlock();
    return true;
}

bool SimoxRobotViewer::showPath( const std::string &pathName, VirtualRobot::TrajectoryPtr path, VirtualRobot::VisualizationFactory::Color c )
{
	removePath(pathName);
	if (!path)
		return false;
	lock();
	VirtualRobot::RobotNodeSetPtr rns = path->getRobotNodeSet();
	if (!rns)
	{
		VR_ERROR << "No rns" << endl;
        unlock();
		return false;
	}
	RobotNodePtr tcp = rns->getTCP();
	if (!tcp)
	{
		VR_ERROR << "No tcp" << endl;
		unlock();
		return false;
	}
	SoSeparator *sep = new SoSeparator;
	sep->addChild(VirtualRobot::CoinVisualizationFactory::getCoinVisualization(path));
	//Saba::CoinRrtWorkspaceVisualization cv(robot,cspace,tcp->getName());
	//cv.setCustomColor(c.r,c.g,c.b);
	//cv.addCSpacePath(path,Saba::CoinRrtWorkspaceVisualization::eCustom);
	//SoSeparator *sep = cv.getCoinVisualization();
	if (!sep)
	{
		unlock();
		return false;
	}
	pathData pd;
	pd.trajectory = path;
	pd.visu = sep;
	paths[pathName] = pd;

	// add visu
	sceneSep->addChild(sep);
	unlock();
	return true;
}

bool SimoxRobotViewer::showPath( const std::string &pathName, std::vector< std::vector<float> > path, const std::string &rns, VirtualRobot::VisualizationFactory::Color c )
{
	removePath(pathName);
	lock();
	if (path.size()==0 || !robot || !robot->hasRobotNodeSet(rns))
	{
		unlock();
		return false;
	}
	VirtualRobot::RobotNodeSetPtr rnsP = robot->getRobotNodeSet(rns);
	
	int joints = rnsP->getSize();
	if (joints<=0)
	{
		unlock();
		return false;
	}

	VirtualRobot::TrajectoryPtr tr(new VirtualRobot::Trajectory(rnsP,pathName));

	for (size_t i=0;i<path.size();i++)
	{
		if (path[i].size() != joints)
		{
			cout << "internal error. Config " << i << " has wrong size:" << path[i].size() << ". Expecting " << joints << endl;
			continue;
		}
		Eigen::VectorXf c(joints);
		for (int j=0;j<joints;j++)
			c[j] = path[i][j];
		
		//cspacePath->addPathPoint(c);
		tr->addPoint(c);
	}
	unlock();
	return showPath(pathName,tr,c);
}

bool SimoxRobotViewer::removePath( const std::string &pathName )
{
	lock();
	if (paths.find(pathName) == paths.end())
	{
		//VR_INFO << "Do not know path visu " << pathName << endl;
		unlock();
		return false;
	}
	SoSeparator* sep = paths[pathName].visu;
	// remove visus
	if (sceneSep->findChild(sep)>=0)
	{
		sceneSep->removeChild(sep);
	}
	paths.erase(pathName);
	unlock();

	return true;
}

bool SimoxRobotViewer::showVector( const std::string &vecName, const Eigen::Vector3f &pos, const Eigen::Vector3f &ori, float scaling )
{
	removeVector(vecName);

	lock();
	SoSeparator* sep = new SoSeparator();
	sep->addChild(CoinVisualizationFactory::CreateVertexVisualization(pos,5,0,1,0,0));
	if (ori.norm()>1e-9 && scaling>0)
	{
		SoTranslation* t = new SoTranslation();
		//cout << "ori:\n" << ori << endl;
		t->translation.setValue(pos[0],pos[1],pos[2]);
		sep->addChild(t);
		SoSeparator* sepA = CoinVisualizationFactory::CreateArrow(ori,50.0f*scaling,2.0f*scaling,VirtualRobot::VisualizationFactory::Color::Blue());
		sep->addChild(sepA);
	}

	SoSeparator* sText = CoinVisualizationFactory::CreateText(vecName);
	if (sText)
		sep->addChild(sText);
	vectors[vecName] = sep;

	// add visu
	sceneSep->addChild(sep);

	unlock();
	return true;
}

bool SimoxRobotViewer::removeVector( const std::string &vecName )
{
	lock();
	if (vectors.find(vecName) == vectors.end())
	{
		//VR_INFO << "Do not know path visu " << pathName << endl;
		unlock();
		return false;
	}
	SoSeparator* sep = vectors[vecName];
	// remove visus
	if (sceneSep->findChild(sep)>=0)
	{
		sceneSep->removeChild(sep);
	}
	vectors.erase(vecName);
	unlock();

	return true;
}


bool SimoxRobotViewer::showPlane( const std::string &planeName, const Eigen::Vector3f &pos, const Eigen::Vector3f &n )
{
	removePlane(planeName);

	lock();
	SoSeparator* sep = new SoSeparator();
	sep->addChild(CoinVisualizationFactory::CreatePlaneVisualization(pos,n,4000.0f,0.5f));

	planes[planeName] = sep;

	// add visu
	sceneSep->addChild(sep);

	unlock();
	return true;
}

bool SimoxRobotViewer::removePlane( const std::string &planeName )
{
	lock();
	if (planes.find(planeName) == planes.end())
	{
		//VR_INFO << "Do not know path visu " << pathName << endl;
		unlock();
		return false;
	}
	SoSeparator* sep = planes[planeName];
	// remove visus
	if (sceneSep->findChild(sep)>=0)
	{
		sceneSep->removeChild(sep);
	}
	planes.erase(planeName);
	unlock();

	return true;
}

bool SimoxRobotViewer::convertCoords( const std::string &from, const std::string &to, Eigen::Matrix4f &m )
{
	if (!robot)
		return false;

	bool fromGlobal = from.empty();
	bool toGlobal = to.empty();
	if (fromGlobal && toGlobal)
		return true;
	lock();
	VirtualRobot::RobotNodePtr f;
	if (!fromGlobal)
	{
		if (!robot->hasRobotNode(from))
		{
			VR_ERROR << "no robot node with name " << from << endl;
			unlock();
			return false;
		}
		f = robot->getRobotNode(from);
	}
	VirtualRobot::RobotNodePtr t;
	if (!toGlobal)
	{
		if (!robot->hasRobotNode(to))
		{
			VR_ERROR << "no robot node with name " << to << endl;
			unlock();
			return false;
		}
		t = robot->getRobotNode(to);
	}

	if (fromGlobal)
	{
		if (!t)
		{
			unlock();
			return false;
		}
		m = t->toLocalCoordinateSystem(m);
		unlock();
		return true;
	}

	if (toGlobal)
	{
		if (!f)
		{
			unlock();
			return false;
		}
		m = f->toGlobalCoordinateSystem(m);
		unlock();
		return true;
	}

	// convert from f to t
	if (!f || !t)
	{
		unlock();
		return false;
	}
	m = f->toGlobalCoordinateSystem(m);
	m = t->toLocalCoordinateSystem(m);
	unlock();
	return true;
}

bool SimoxRobotViewer::saveScene(const std::string &filename, const std::string &sceneName)
{
	lock();

	VirtualRobot::ScenePtr s(new VirtualRobot::Scene(sceneName));
	if (robot)
		s->registerRobot(robot);

	std::map< std::string, objectData >::const_iterator i = objects.begin();
	while (i!=objects.end())
	{
		s->registerManipulationObject(i->second.object);
		i++;
	}

	std::map< std::string, pathData >::const_iterator j = paths.begin();
	while (j!=paths.end())
	{
		s->registerTrajectory(j->second.trajectory);
		j++;
	}
	bool res = VirtualRobot::SceneIO::saveScene(s,filename);

	unlock();

	return res;
}

bool SimoxRobotViewer::resetObjects()
{
	lock();


	while (objects.size()>0)
	{
		std::map< std::string, objectData >::const_iterator i = objects.begin();
		if (!removeObject(i->first))
			break;
	}

	while (paths.size()>0)
	{
		std::map< std::string, pathData >::const_iterator i = paths.begin();
		if (!removePath(i->first))
			break;
	}

	while (vectors.size()>0)
	{
		std::map< std::string, SoSeparator* >::const_iterator i = vectors.begin();
		if (!removeVector(i->first))
			break;
	}

	while (planes.size()>0)
	{
		std::map< std::string, SoSeparator* >::const_iterator i = planes.begin();
		if (!removePlane(i->first))
			break;
	}


	while (eefVisu.size()>0)
	{
		std::map< std::string, SoSeparator* >::const_iterator i = eefVisu.begin();
		if (!removeEEFVisu(i->first))
			break;
	}


	unlock();

	return true;
}



