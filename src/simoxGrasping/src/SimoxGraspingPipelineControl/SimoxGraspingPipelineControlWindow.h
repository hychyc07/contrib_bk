#ifndef __Simox_Grasping_Pipeline_Control_Window_h__
#define __Simox_Grasping_Pipeline_Control_Window_h__

#include <stdio.h>
#include <yarp/os/Time.h>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualizationNode.h>
#include <VirtualRobot/Visualization/CoinVisualization/CoinVisualization.h>

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

#include <yarp/os/RpcClient.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Network.h>

#include <Eigen/Core>

// forward declaration, avoids to include the generated ui header file
namespace Ui {
	class MainWindowGraspingPipeline;
}
class SimoxGraspingPipelineControlModule;

class SimoxGraspingPipelineControlWindow : public QMainWindow
{
	Q_OBJECT
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
	/*!
		Constructor. Initialize from a resource finder object.
	*/
    //SimoxGraspingPipelineControlWindow(/*yarp::os::ResourceFinder &rf*/);

	/*!
		Constructor. Use this constructor for custom initialization.
	*/
	SimoxGraspingPipelineControlWindow(SimoxGraspingPipelineControlModule *communication);


    virtual ~SimoxGraspingPipelineControlWindow();


	/*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
	int main();

	void ping();

	//! Indicates if the viewer was closed
	bool wasClosed();

public slots:

	void setObject();
	
	void removeObject();


	void setObjectPose();

	void searchIK();

	void selectGrasp(int nr);
	void selectObject(int nr);

	void showReachableGrasps();
	void showGraspingMotion();
	void setupRobotVisu();

	void executeIk();

	void checkTableModel();

	void handEyeOffsetChanged();

	void getOffsetFromHandEyeCalib();
	void buildPath();
	void executePath();
	void solutionSliderMoved();
	
	void threshSliderMoved();

	void executeSolutionPos();

	void searchLego();

	void openHand();
	void closeHand();
	void liftHand();

	
	/*! Closes the window and exits SoQt runloop. */
	void quit();

	/*!< Overriding the close event, so we know when the window was closed by the user. */
	void closeEvent(QCloseEvent *event);

	void goToStartPose();

	void saveScene();

	void stopMovement();

	void lookTable();
	void lookAhead();
	void checkObstacleModel();

public:

	/*!
		Add an object to visualization.
		\param objectName A string to identify the object. if an object with the same name is already present it is quietly replaced.
		\param filename The filename of the MnaipulationObject XML file. If file is not present or "" the object is deleted.
		\return True if object is present after call, false if not.
	*/
	bool setObject(const std::string &objectName, const std::string &filename);
	bool removeObject( const std::string &objectName);

	bool searchIK( const std::string &objectName, const std::string &graspName );


	bool setObjectPose(const std::string &objectName, Eigen::Matrix4f &pose);
	Eigen::Matrix4f getObjectPose(const std::string &objectName);
	void updateReachableGrasps();

	void selectGrasp( const std::string &objectName, const std::string &graspName);
	void showReachableGrasps(const std::string &objectName, bool enable);


	void setupUI();

	//! Update the GUI with current object pose
	bool updateObjectPose( );
	int getObjectIndex( const std::string &objectName );
	void setIkQueryText(bool ikChecked, bool ikOk, const std::string &grasp, float quality, float timeMS);
	int getGraspIndex( const std::string &graspName );
	void invalidateIk();
	void invalidateReachableGrasps();
	bool sampleSolutionPath(std::vector< std::vector<float> > &sol, float sampleDist_cspace);
	Eigen::Matrix4f getOffsetFromGui();
	void moveObject(const std::string &objectStr, const Eigen::Matrix4f &m);
	Ui::MainWindowGraspingPipeline *UI;
	SimoxGraspingPipelineControlModule *com;

	int pingValue;

	bool isClosed;

	bool ikValid;
	std::vector<float> ik_solution;
	std::vector< std::vector<float> > grasping_motion; // joint values
	std::map<std::string, Eigen::Matrix4f, std::less<std::string>, 
		Eigen::aligned_allocator<std::pair<const std::string, Eigen::Matrix4f> > > objectPose;

	std::map< std::string, float> reachable_grasps; // grasps and (optionally) the manipulability
	
};

typedef boost::shared_ptr<SimoxGraspingPipelineControlWindow> SimoxGraspingPipelineControlWindowPtr;

#endif
