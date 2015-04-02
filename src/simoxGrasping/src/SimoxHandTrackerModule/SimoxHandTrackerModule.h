#ifndef __Simox_SimoxHandTracker_Module_h__
#define __Simox_SimoxHandTracker_Module_h__


#include <stdio.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/Time.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <string.h>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <VirtualRobot/VirtualRobot.h>
#include <VirtualRobot/Workspace/Reachability.h>
#include <VirtualRobot/Robot.h>
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/RobotNodeSet.h>
#include <VirtualRobot/ManipulationObject.h>
#include <VirtualRobot/EndEffector/EndEffector.h>
#include <VirtualRobot/IK/GenericIKSolver.h>
#include <VirtualRobot/Grasping/Grasp.h>
#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RpcClient.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/PolyDriver.h>
#include "../SimoxRobotViewer/SimoxRobotViewer.h"

#include <cv.h>

/*!
    This module handles the hand tracking for hand-eye calibration.
    It shows the current robot state as well as the tracking results in a 3D SimoxRobotViewer window.
    The images are retreived from the lumaChroma module and result images are offered to view the
    segmentation and the blob detection.
*/
class SimoxHandTrackerModule : public yarp::os::RFModule
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            SimoxHandTrackerModule();

    double getPeriod();

    /*
    * This is our main function. Will be called periodically every getPeriod() seconds.
    */
    bool updateModule();

    /*
    * Message handler. Just echo all received messages.
    */
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply); 


    /* 
    * Configure function. Receive a previously initialized
    * resource finder object. Use it to configure your module.
    * Open port and attach it to message handler.
    */
    bool configure(yarp::os::ResourceFinder &rf);

    /*
    * Interrupt function.
    */
    bool interruptModule();

    /*
    * Close function, to perform cleanup.
    */
    bool close();

    // print info
    void print();



protected:

	/*!
		Searches blobs and prints info to std::out
	*/
	void printBlobInfo();

    bool loadRobot( const std::string &filename );
    bool setJointValues(const std::string &rns, std::vector< float> &jv);

    bool setupViewer();

    bool processImages();

    /*!
        Search for blobs (onlz img pixels == 255 are considered) and if result!=NULL rects are painted on result image (must be a three channel image)
        Input image (gray) is modified. All entries==255 are changed to black.
        The blob centeres are stored in points2d (vector is cleared).
        Returns true if at least one blob was found.
      */
    bool searchBlobs(IplImage *gray, std::vector< CvPoint > &points2d, int w_offset, int h_offset, int minPixels = 5, int maxPixels = 100, IplImage *result = NULL);

    /*!
        Searches first trackRobotNodes.size() best entries of filterBlobs3D w.r.t. minimum distance to one of the trackRobotNodes (the positions in our model).
        All blobs with minDistance to one of the robotNodes > maxModelDistanceMM are ignored.
        Stores result to filterBlobs3D.
        If storeIndexMapping!=NULL, the used indices of inputBlobs3D are storedd here.
        Return false if inputBlobs3D.size < trackRobotNodes.size()
     */
    bool getBestMatchesDistance(const std::vector< Eigen::Vector3f> &inputBlobs3D, std::vector< Eigen::Vector3f > &filteredBlobs3D, float maxModelDistanceMM, std::vector<int> *storeIndexMapping=NULL);

    //! returns minimum distance of pos (given in mm, in rootCoord system) to one of the trackedRobotNodes (model), in MM
    float getMinDistToModelNodes(Eigen::Vector3f &pos);

    bool paint2dBlobSelection(IplImage* img, std::vector< CvPoint > &allBlobs, std::vector<int> &indexMapping, int size=2);

	bool get3DPositionMono(bool leftCam, float distCamMM, const std::vector< CvPoint > &points2d, std::vector< Eigen::Vector3f > &storeBlobs3D);
	bool get3DPositionStereo(const std::vector< CvPoint > &points2d_l, const std::vector< CvPoint > &points2d_r, std::vector< Eigen::Vector3f > &storeBlobs3D);

    bool grabImages();

    bool resetTracking();
    bool updateTracking(const std::vector< Eigen::Vector3f > &positions);

    //! Tries to match trackedPositions to current position of RobotNodes.
    bool matchPositions();

    void showTrackedBlobs3DVisu(bool enable);

    void showTrackedTCPVisu(bool enable);

    // get 3d point in rootCoordSystem [mm]
    bool get3DPointMono(bool leftCam, int cx, int cy, float distCamMM, Eigen::Vector3f &storePoint3d);
	//bool get3DPointStereo(int cx, int cy, Eigen::Vector3f &storePoint3d);

    float getMinDist(std::vector<float> &point3D, int &storeIndex);
    
    /* assuming positions are on a line in cam image: re-order blobs and mapping accordingy*/
	bool sortPositionsLine(const std::vector< CvPoint > &points2d, const std::vector< Eigen::Vector3f > &inputBlobs3D, std::vector< Eigen::Vector3f > &blobs3D, std::vector<int> &indexMapping);

    //! Stores 3d model values to currentModelPositions_rootCoord, performs coordinate transformation to rootCoordSystem.
    bool updateCurrentModelPosition();
    Eigen::Vector3f getMean(const std::vector< Eigen::Vector3f> &p);
    Eigen::Matrix4f getTransformationMatrix(const std::vector< Eigen::Vector3f> &p1, const std::vector< Eigen::Vector3f> &p2);
    bool updateTrackingVisu( );
    Eigen::Matrix4f estimateTCP(std::vector<Eigen::Vector3f > &positionsFingers);
    void createPermutationsVisit(std::vector<int> &processing, int k, std::vector< std::vector<int> > &storeResult);
    std::vector< std::vector<int> > createPermutations(unsigned int size);
    bool orderTrackedPositions();
    float getPointsToFingertipsDistance(std::vector< Eigen::Vector3f> &points, bool lockMutex);
    void showTrackedPositionsVisu(bool enable);
	/*! 
		Checks if new images are available.
		Searches all blobs and stores them to points2d_left, points2d_right (in cam coords)
		Uses expected distance to left and right camera (distCamMM_left and distCamMM_right) to estimate 3D positions.
		The 3D positions are stored in leftBlobs3D_rootCoord and rightBlobs3D_rootCoord.
	*/
	bool processAllBlobs(float distCamMM_left,float distCamMM_right, std::vector< CvPoint > &points2d_left, std::vector< CvPoint > &points2d_right, std::vector< Eigen::Vector3f > &leftBlobs3D_rootCoord, std::vector< Eigen::Vector3f > &rightBlobs3D_rootCoord, IplImage* resultCVL = NULL, IplImage* resultCVR = NULL, double window_ratio = 0.6);
	VirtualRobot::RobotPtr robot;

	float getFingerToCamDist(bool leftCam);
	/*!
        The coordinate systems of these RobotNodes are tracked.
    */
    std::vector< VirtualRobot::RobotNodePtr > trackRobotNodes;
    std::vector< std::string > trackedObjectVisuNames;

    VirtualRobot::RobotNodePtr rootCoordSystem;
    VirtualRobot::RobotNodePtr tcpCoordSystem;
    VirtualRobot::EndEffectorPtr eef;

	bool localizationUseStereo;	//! Use mono or stereo approach for 3d object localization
    std::vector< Eigen::Vector3f > currentTrackedPositions_rootCoord;     //!< in root coord system
    std::vector< Eigen::Vector3f > currentModelPositions_rootCoord;		//!< in root coord system
	std::vector< Eigen::Vector3f > blobs3D_left_rootCoord;                //!< in root coord system [mm] (positions retrieved with a mono approach, the z distance is guessed)
	std::vector< Eigen::Vector3f > blobs3D_right_rootCoord;               //!< in root coord system [mm] (positions retrieved with a mono approach, the z distance is guessed)
	std::vector< Eigen::Vector3f > blobs3D_stereo_rootCoord;              //!< in root coord system [mm] (positions retrieved with a stereo approach)
    Eigen::Matrix4f currentTrackedTCP_rootCoord;

    SimoxRobotViewerPtr viewer;
	
	Eigen::Matrix4f currentCalibrationResult;

    float imgThreshold;

    bool showTrackedRobotNodes,showTrackedTCP,showTrackedEEF,showBlobs3D;

    std::string moduleName;
    std::string robotBase;
    std::string handlerPortName;
    std::string robotFile;
	yarp::os::Port handlerPort; //a port to handle messages

	//yarp::os::BufferedPort<yarp::os::Bottle> 
	yarp::os::RpcClient stereoDisparityRPC;


    double timeUpdateEncodersS;
    double lastEncoderUpdateS;
    std::string rnsArmLeft,rnsArmRight,rnsHandLeft,rnsHandRight,rnsTorso,rnsLegLeft,rnsLegRight,rnsHead;
    yarp::sig::Vector jointValuesLeftArm,jointValuesRightArm,jointValuesHead,jointValuesTorso;
    yarp::dev::PolyDriver robotDeviceLeftArm,robotDeviceRightArm,robotDeviceHead,robotDeviceTorso;
    yarp::dev::IEncoders *encLeftArm,*encRightArm,*encHead,*encTorso;

    yarp::os::BufferedPort<yarp::sig::Image> inImgLeftPort;
    yarp::os::BufferedPort<yarp::sig::Image> inImgRightPort;
    yarp::os::Port                           outImgLeftPort;	
    yarp::os::Port                           outImgRightPort;
    yarp::os::Port                           outBlobImgLeftPort;	
    yarp::os::Port                           outBlobImgRightPort;
    bool imgsReady;
    yarp::sig::Image currentImageLeft,currentImageRight;
    CvMemStorage* cvStorage;
    CvFont font;

    //! gaze control
    yarp::dev::PolyDriver iCubGazeClient;
    yarp::dev::IGazeControl *iCubGazeControl;
    bool isLeft;

    yarp::os::Semaphore mutexImgAccess;
    yarp::os::Semaphore mutexRobotAccess;
};


typedef boost::shared_ptr<SimoxHandTrackerModule> SimoxHandTrackerModulePtr;

#endif
