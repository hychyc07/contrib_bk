#ifndef __Simox_Hand_Eye_Calib_Window_h__
#define __Simox_Hand_Eye_Calib_Window_h__

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

// forward declaration, avoids to include the generated ui header file
namespace Ui {
	class MainWindowHandEyeCalibration;
}
class SimoxHandEyeCalibrationGui;

/*!
    This class handles the Qt window. All Qt-callbacks are handled and executed by calling the appropriate methods of
	SimoxHandEyeCalibrationGui.

	@see SimoxHandEyeCalibrationGui
*/
class SimoxHandEyeCalibrationWindow : public QMainWindow
{
	Q_OBJECT
public:

    //! A struct defining a tuple with name and 7d-pose
        struct handPose
        {
                std::string name;
                std::vector<float> cartPos_root; // 7d:x,y,z,axis and angle
        };
        //! A struct defining a tuple with name and 9d joint values
            struct preshape
            {
                    std::string name;
                    std::vector<float> fingerJV; // 9d joint values
            };

	/*!
		Constructor. Use this constructor for custom initialization.
	*/
        SimoxHandEyeCalibrationWindow(SimoxHandEyeCalibrationGui *communication, std::vector<handPose> &handPoses, std::vector<preshape> &preshapes);


    virtual ~SimoxHandEyeCalibrationWindow();


	/*!< Executes the SoQt mainLoop. You need to call this in order to execute the application. */
	int main();

    //!Send alive signal
	void ping();

	//! Indicates if the viewer was closed
	bool wasClosed();

	//! setup all UI elements and viewer
	void firstRun();

    //! update status of UI
    void updateWindow();

    //! update text on UI 
    void updateInfoTcpPose(bool model, std::vector<float> pose7D);

     //! update text on UI
   void updateInfoJoints(std::vector<float> jointsArm, std::vector<float> jointsHand, std::vector<float> jointsHip, std::vector<float> jointsHead);

public slots:
	
	/*! Closes the window and exits SoQt runloop. */
	void quit();

	/*!< Overriding the close event, so we know when the window was closed by the user. */
	void closeEvent(QCloseEvent *event);
	void processAllBlobs();
	void executeArmConfig();
	void selectArmConfig(int i);
    void selectPreshape( int i );
	void updateVisuOptions();
	void searchFingerTips();
	void segThresholdChanged(int i);
	void updateSegmentationInfo();
	void updateInfo();
	void moveHead();

protected:

	QString createUIString(const std::string &start,std::vector<float> &vec);
	void setupUI();
	bool sendToHandTracker(yarp::os::Bottle &cmd);
	bool sendToHandTracker(yarp::os::Bottle &cmd, yarp::os::Bottle &response);
	//! Update the GUI with current object pose
	Ui::MainWindowHandEyeCalibration *UI;
	SimoxHandEyeCalibrationGui *com;
	std::vector<handPose> handPoses;
    std::vector<preshape> preshapes;
	int pingValue;
	double timeUpdateWindowS;
	double lastWindowUpdateS;	

	bool isClosed;
};

typedef boost::shared_ptr<SimoxHandEyeCalibrationWindow> SimoxHandEyeCalibrationWindowPtr;

#endif
