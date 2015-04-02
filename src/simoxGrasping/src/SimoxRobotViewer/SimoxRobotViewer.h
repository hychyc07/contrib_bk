#ifndef __Simox_Robot_Viewer_h__
#define __Simox_Robot_Viewer_h__


#include <stdio.h>
#include <yarp/os/Time.h>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>
#include <VirtualRobot/Visualization/Visualization.h>
#include <VirtualRobot/Trajectory.h>

#include <string.h>
#include <QtCore/QtGlobal>
#include <QtGui/QtGui>
#include <QtCore/QtCore>

#include <Inventor/sensors/SoTimerSensor.h>
#include <Inventor/nodes/SoEventCallback.h>
#include <Inventor/Qt/viewers/SoQtExaminerViewer.h>
#include <Inventor/Qt/SoQt.h>
#include <Inventor/nodes/SoSeparator.h>
#include <vector>


#include "CoinViewer.h"

#include <yarp/os/ResourceFinder.h>

// forward declaration, avoids to include the generated ui header file
namespace Ui {
	class MainWindowSimoxRobotViewer;
}
/*!
	\defgroup SimoxRobotViewer SimoxRobotViewer
	\ingroup SimoxGrasping

	\brief A Qt/Coin3D viewer for simox robot models, objects and grasps.

 With SimoxRobotViewer one can open a 3D window and display a robot and/or objects. Further grasping 
 and reachability information can be displayed. To allow a smooth integration you must consider 
 several things, as shown below.
 An exemplary setup can be found in SimoxRobotViewerModule.cpp, showing how yarp is queried to get 
 the current joint values of iCub and how to send these values to the viewer.h.
 The following points have to be considered:

 1. Initialize the Coin3D / Qt / SoQt framework, e.g. in your Constructor.
	\code
	myYarpModule::myYarpModule()
	{
		if (!SoDB::isInitialized())
			SoDB::init();
		SoQt::init("myRobotViewer");
	}
	\endcode

 2. Create a viewer, e.g. in the configure() method. 
	The rf is queried for the following entries:
	* robot: The robot (standard: icubSim)
	* SimoxDataPath: An additional path to a location where Simox searches files. Initially the standard data paths are set, e.g. SimoxDir/VirtualRobot/data
	* RobotFile: A robot XML filename, relative to one of the data paths.
	* EndEffector: optional, an eef which is used for visualizing grasps
	* RobotNodeSet_LeftArm,RobotNodeSet_RightArm,RobotNodeSet_LeftHand,
	  RobotNodeSet_RightHand,RobotNodeSet_Torso,RobotNodeSet_Head,RobotNodeSet_LeftLeg,RobotNodeSet_RightLeg:
	  Strings, defining the robot node set corresponding to the robot's XML definition. With these strings the joints of the robot can be updated.
	Have a look at the ini file for SimoxRobotViewerModule, located at contrib/src/SimoxGrasping/app/SimoxRobotViewer/conf
	\code
	bool myYarpModule::configure( yarp::os::ResourceFinder &rf )
	{
		...
		// viewer is of type SimoxRobotViewerPtr
		viewer.reset(new SimoxRobotViewer("Simox iCub Viewer",rf));
	}
	\endcode

 3. Update the viewer and the Qt environment wihtin your main loop:
	\code
	bool myYarpModule::updateModule()
	{
		// check if viewer was closed
		if (viewer->wasClosed())
		{
			cout << "Viewer was closed, quitting..." << endl;
			return false;
		}
		...
		// upodate joint values
		viewer->setJoints(nodeSetName,jointValueVector);
		...
 
		// send alive signal
		viewer->ping();

                // Important: update Qt.
		// Without calling the processEvents method your Qt surface won't be repainted.
		if (qApp)
		{
			qApp->processEvents();
		}
		return true;
	}
	\endcode

 4. When exiting close the viewer
	\code
	bool myYarpModule::close()
	{
		if (viewer)
			viewer->quit();
	}
	\endcode
*/
class SimoxRobotViewer : public QMainWindow
{
	Q_OBJECT
public:
    
	/*!
		Constructor. Initialize from a resource finder object.
		Checking for the following entries:
		SimoxDataPath: A data search path for simox. Files can be specified relatively to this path. (optional)
		RobotFile: The robot's xml definition. This robot is loaded and displayed.
		EndEffector: This end effector is used. the string must be defined in the robot's xml file. When not set, the first eef is selected. (optional)
		ReachabilityFile: Optional definition of a reachability file. The reachability data can be displayed in the viewer. (optional)
		*/
    SimoxRobotViewer(const std::string &title, yarp::os::ResourceFinder &rf);

	/*!
		Constructor. Use this constructor for custom initialization.
	*/
	SimoxRobotViewer(const std::string &title);


    virtual ~SimoxRobotViewer();


	/*!
		Executes the SoQt mainLoop. You need to call this in order to execute the application.
		If you don't want to give the control to the Qt mainloop, be sure to call qApp->processEvents() in your custom mainloop.
		This ensures that all Qt events and painting are executed correctly. 
	*/
	int main();


	void showViewer(bool enable);

	/*!
		Load a robot from file.
	*/
	bool loadRobot (const std::string &filename);

	/*!
		Set the robot. Be sure, this class has exclusive render access to this robot. 
		Don't access the robot from outside this class (unless you lock/unlock the robot access).
		Or pass a cloned robot and access the joint values with setJoints().
	*/
	bool setRobot(VirtualRobot::RobotPtr robot);

	/*!
		Set the visualization: Full, Collision, CollisionData.
	*/
	void showRobot(bool enable, VirtualRobot::SceneObject::VisualizationType visu = VirtualRobot::SceneObject::Full);

	/*!
		Set joint values.
	*/
	bool setJoints(const std::string &robotNodeSet, std::vector<float> jointValues);

	//! Set joint limits for a node [rad].
	bool setJointLimit(const std::string &robotNode, float min, float max);
	bool setJointLimits(const std::string &robotNodeSet, std::vector<float> &min, std::vector<float> &max);

	//! Send a ping signal to this viewer
	void ping();

	//! Indicates if the viewer was closed
	bool wasClosed();

	/*!
		Add an object to visualization.
		\param objectName A string to identify the object. if an object with the same name is already present it is quietly replaced.
		\param filename The filename of the ManipulationObject XML file. If file is not present or "", the object is deleted.
		\param c Optionally you can colorize the object.
		\return True if object is present after call, false if not.
	*/
	bool setObject(const std::string &objectName, std::string filename, VirtualRobot::ColorPtr c = VirtualRobot::ColorPtr());
	bool setObject(const std::string &objectName, VirtualRobot::ManipulationObjectPtr object, VirtualRobot::ColorPtr c = VirtualRobot::ColorPtr());
	bool removeObject( const std::string &objectName);
	bool showObject(const std::string &objectName, bool enable = true);

	/*!
		Show grasps, that are stored with the object for the current eef.
		If reachability data is loaded the grasps can be filtered.

		@see setObject()
		@see selectEEF()
		@see setReachability()
	*/
	bool showGrasps(const std::string &objectName, bool enableAll, bool enableReachable);
	bool showGrasps(const std::string &objectName, VirtualRobot::GraspSetPtr grasps, bool enable);

	/*!
		Show a specific grasp, the grasp must be stored with the ManipulationObject

		@see setObject()
		@see selectEEF()
	*/
	bool showGrasp( const std::string &objectName, const std::string &graspName, bool enable );
	bool showGrasp( const std::string &objectName, const std::vector<std::string> &graspNames, bool enable );
	bool showGrasp( const std::string &objectName, VirtualRobot::GraspPtr grasp, bool enable );
	bool showGrasp( const std::string &objectName, std::vector<VirtualRobot::GraspPtr> grasps, bool enable );

	/*!
		Show/Hide a coordinate system.
	*/
	bool showCoordSystem( const std::string &jointName, bool enable );
	bool showObjectCoordSystem( const std::string &objectName, bool enable );

	
	/*!
		Move object to a pose. If coordSystem is empty, the global coordinate system is assumed. Otherwise the the local coordinate system defined by the RobotNode with name coordSystem is used.
	*/
	bool setObjectPose(const std::string &objectName, const Eigen::Matrix4f &pose, const std::string coordSystem = "");

	/*!
		Returns the pose of the object.
		\param objectName The name that was used to register an object with setObject().
		\param coordSystem If coordSystem is empty, the global coordinate system is assumed. Otherwise the the local coordinate system defined by the RobotNode with name coordSystem is used.
		\return The object's pose.
	*/
	Eigen::Matrix4f getObjectPose(const std::string &objectName, const std::string coordSystem = "");

	/*!
		Show a path in workspace.
		\param pathName The string associates a path. If a path with same name is already present, it is quietly replaced.
		\param path The path to display. The path is queried for rns.
		\param c Optionally you can specify a color.
	*/
	bool showPath( const std::string &pathName, VirtualRobot::TrajectoryPtr path, VirtualRobot::VisualizationFactory::Color c = VirtualRobot::VisualizationFactory::Color::Blue() );
	/*!
		Show a path in workspace.
		\param pathName The string associates a path. If a path with same name is already present, it is quietly replaced.
		\param path The path to display. 
		\param rns The RobtoNodeSet that should be used.
		\param c Optionally you can specify a color.
	*/
	bool showPath( const std::string &pathName, std::vector< std::vector<float> > path, const std::string &rns, VirtualRobot::VisualizationFactory::Color c = VirtualRobot::VisualizationFactory::Color::Blue() );

	//! disable path visualization
	bool removePath(const std::string &pathName);
	/*!
		Select the end-effector that should be used to visualize grasps.
		This eef must be part of the robot definition.
	*/
	bool selectEEF(const std::string &eef);

	/*!
		Set the end-effector to a preshape (must be defined in the robot's XML definition)
	*/
	bool selectEEFPreshape(const std::string &eefPreshape);
	bool selectRNS(const std::string &rns);


	/*!
		Add a visualization of the EEF to visualization.
		\param eefVisuName A string to identify the eef visualization. if an eef visu with the same name is already present it is quietly replaced.
		\param eef The eef as defined in the robot's xml definition. If it is not present or "", the visualization is deleted.
		\return True if eef visu is present after call, false if not.
	*/
	bool setEEFVisu(const std::string &eefVisuName, const std::string &eef, VirtualRobot::SceneObject::VisualizationType visu = VirtualRobot::SceneObject::Full);
	bool setEEFVisu(const std::string &eefVisuName, SoSeparator *vis);
	bool removeEEFVisu( const std::string &eefVisuName);
	bool showEEFVisu(const std::string &eefVisuName, bool enable = true);
	/*!
		Move the eef visu to a pose (given in global coord system).
		Note, that the EEF's tcp coord system is used for positioning.
	*/
	bool setEEFVisuPose(const std::string &eefVisuName, const Eigen::Matrix4f &pose);

	/*!
		Use this method to show a vector at given position pos with given orientation ori.
		If a vector with name vecName is already present, its pose is silently updated.

		\param vecName To identify this visualizartion. The text is also displayed in the viewer
		\param pos The position that is displayed
		\param ori If ori.norm() != 0 a vector is displayed at pos showing in direction ori.
		\param scaling Scales the vector visualization.
	*/
	bool showVector(const std::string &vecName, const Eigen::Vector3f &pos, const Eigen::Vector3f &ori, float scaling = 1.0f );
	bool removeVector( const std::string &vecName);

	bool showPlane(const std::string &planeName, const Eigen::Vector3f &pos, const Eigen::Vector3f &n);
	bool removePlane( const std::string &planeName);


	/*!
		Load reachability information from file.
	*/
	bool loadReachability( const std::string &filename );
	/*!
		Set reachability object.
	*/
	bool setReachability(VirtualRobot::ReachabilityPtr reachabilityData);

	/*!
		Enable/Disable visualization of reachability. The orientation within each voxel is displayed as an arrow, 
		which is aligned with tcpDir.
	*/
	void showReachability(bool enable, const Eigen::Vector3f &tcpDir);

	/*!
		Uses current robot visualization to transform coordinates.
		If <from> or <to> is empty global coordinate system is assumed. The result is stored in place to matrix m.
		\param from The RobotNode that indicates the source coordinate system. If empty the global coord system is assumed.
		\param from The RobotNode that indicates the target coordinate system If empty the global coord system is assumed.
		\param m The pose is transformed in place. Assuming [mm] position.
		\return True or false indicating success or faiulure.
	*/
	bool convertCoords(const std::string &from, const std::string &to, Eigen::Matrix4f &m);

	/*!
		Save current scene to VirtualRobot's xml scene-file format. 
		The robot at its current configuration together with all objects and paths are stored.
		\param filename A valid filename. If a file with the given name exists, it is silently overwritten.
		\param sceneName A string that is stored 
	*/
	bool saveScene(const std::string &filename, const std::string &sceneName = "SimoxRobotViewerScene");

	void viewAll();

	/*!
		When accessing robot or models from another thread you must ensure that no redrawing is done, otherwise Qt/Coin will crash from time to time!
		Redrawing is protected by a mutex, that can be locked/unlocked here.
	*/
	void lock();
	void unlock();

	/*!
		Remove all objects, paths, vectors etc from viewer. 
	*/
	bool resetObjects();

public slots:
	/*! Closes the window and exits SoQt runloop. */
	void quit();

	/*!< Overriding the close event, so we know when the window was closed by the user. */
	void closeEvent(QCloseEvent *event);

	/*!
		Schedule a redraw event.
	*/
	void redraw();

protected:

	void setupUI(const std::string &title);
	VirtualRobot::SceneObject::VisualizationType visualisationTypeRobot;
	bool visualizationRobotEnabled;

	Ui::MainWindowSimoxRobotViewer *UI;

	SoSeparator *sceneSep;
	SoSeparator *robotSep;
	SoSeparator *reachSep;
	CoinViewer *viewer;
	bool enableViewer;

	int pingValue;
	VirtualRobot::RobotPtr robot;

	bool isClosed;
	VirtualRobot::EndEffectorPtr currentEEF;

	VirtualRobot::ReachabilityPtr reachSpace;
	VirtualRobot::RobotNodeSetPtr currentRNS;

	struct objectData 
	{
		VirtualRobot::ManipulationObjectPtr object;
		VirtualRobot::ColorPtr c;
		SoSeparator* visuObject;
		SoSeparator* visuGrasps;
		SoSeparator* visuReachableGrasps;
	};

	struct pathData 
	{
		VirtualRobot::TrajectoryPtr trajectory;
		SoSeparator* visu;
	};

	std::map< std::string, objectData > objects;
	std::map< std::string, SoSeparator* > eefVisu;
	std::map< std::string, pathData > paths;
	std::map< std::string, SoSeparator* > vectors;
	std::map< std::string, SoSeparator* > planes;
};

typedef boost::shared_ptr<SimoxRobotViewer> SimoxRobotViewerPtr;

#endif
