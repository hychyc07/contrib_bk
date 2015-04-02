
#include "SimoxGraspingPipelineControlWindow.h"
#include <VirtualRobot/XML/RobotIO.h>
#include <VirtualRobot/VirtualRobotException.h>
#include <VirtualRobot/SceneObject.h>
#include "Inventor/actions/SoLineHighlightRenderAction.h"
#include <VirtualRobot/RuntimeEnvironment.h>
#include <VirtualRobot/XML/ObjectIO.h>


#include "ui_SimoxGraspingPipelineControl.h"

#include "SimoxGraspingPipelineControlModule.h"

// disable head movements (used for simulator)
//#define START_HEAD_LOOK_AHEAD

//#define USE_GATE
#define USE_XWING

using namespace VirtualRobot;
using namespace yarp::os;

SimoxGraspingPipelineControlWindow::SimoxGraspingPipelineControlWindow(SimoxGraspingPipelineControlModule *communication)
:QMainWindow(NULL)
{
	pingValue = 0;
	isClosed = false;
	ikValid = false;

	VR_ASSERT_MESSAGE(communication,"Need a communication interface...");
	com = communication;

	setupUI();

	getOffsetFromHandEyeCalib();
	
#ifdef START_HEAD_LOOK_AHEAD
	com->moveHeadStandardPose(0);
#else 
	com->moveHeadStandardPose();
#endif
	SoQt::show(this);
}

SimoxGraspingPipelineControlWindow::~SimoxGraspingPipelineControlWindow()
{
	delete UI;
}

void SimoxGraspingPipelineControlWindow::lookTable()
{
	com->moveHeadStandardPose();
}
void SimoxGraspingPipelineControlWindow::lookAhead()
{
	com->moveHeadStandardPose(0);
}

void SimoxGraspingPipelineControlWindow::setupUI()
{
	UI = new Ui::MainWindowGraspingPipeline;
	
	UI->setupUi(this);

	connect(UI->pushButtonAddObj, SIGNAL(clicked()), this, SLOT(setObject()));
	connect(UI->pushButtonRemoveObject, SIGNAL(clicked()), this, SLOT(removeObject()));
	connect(UI->pushButtonMoveObj, SIGNAL(clicked()), this, SLOT(setObjectPose()));
	connect(UI->pushButtonSearchIK, SIGNAL(clicked()), this, SLOT(searchIK()));
	connect(UI->pushButtonExecuteIK, SIGNAL(clicked()), this, SLOT(executeIk()));
	connect(UI->pushButtonBuildPath, SIGNAL(clicked()), this, SLOT(buildPath()));
	connect(UI->pushButtonExecutePath, SIGNAL(clicked()), this, SLOT(executePath()));
	//connect(UI->pushButtonGetOffset, SIGNAL(clicked()), this, SLOT(getOffsetFromHandEyeCalib()));
	connect(UI->pushButtonStartPose, SIGNAL(clicked()), this, SLOT(goToStartPose()));
	connect(UI->pushButtonExecutePos, SIGNAL(clicked()), this, SLOT(executeSolutionPos()));
	connect(UI->pushButtonLocalizeLego, SIGNAL(clicked()), this, SLOT(searchLego()));
	connect(UI->pushButtonSave, SIGNAL(clicked()), this, SLOT(saveScene()));
	connect(UI->pushButtonExecutePathSTOP, SIGNAL(clicked()), this, SLOT(stopMovement()));
	connect(UI->pushButtonOpenHand, SIGNAL(clicked()), this, SLOT(openHand()));
	connect(UI->pushButtonCloseHand, SIGNAL(clicked()), this, SLOT(closeHand()));
	connect(UI->pushButtonLiftHand, SIGNAL(clicked()), this, SLOT(liftHand()));
	connect(UI->pushButtonLookTable, SIGNAL(clicked()), this, SLOT(lookTable()));
	connect(UI->pushButtonLookAhead, SIGNAL(clicked()), this, SLOT(lookAhead()));

	connect(UI->comboBoxReachableGrasps, SIGNAL(activated(int)), this, SLOT(selectGrasp(int)));
	connect(UI->comboBoxObject, SIGNAL(activated(int)), this, SLOT(selectObject(int)));

	connect(UI->checkBoxVisuReachableGrasps, SIGNAL(clicked()), this, SLOT(showReachableGrasps()));
	connect(UI->checkBoxVisuGraspingMotion, SIGNAL(clicked()), this, SLOT(showGraspingMotion()));
	connect(UI->checkBoxTable, SIGNAL(clicked()), this, SLOT(checkTableModel()));
	connect(UI->checkBoxObstacle, SIGNAL(clicked()), this, SLOT(checkObstacleModel()));


	connect(UI->checkBoxUseOffset, SIGNAL(clicked()), this, SLOT(handEyeOffsetChanged()));
	connect(UI->doubleSpinBoxOffsetX, SIGNAL(valueChanged(double)), this, SLOT(handEyeOffsetChanged()));
	connect(UI->doubleSpinBoxOffsetY, SIGNAL(valueChanged(double)), this, SLOT(handEyeOffsetChanged()));
	connect(UI->doubleSpinBoxOffsetZ, SIGNAL(valueChanged(double)), this, SLOT(handEyeOffsetChanged()));
	connect(UI->doubleSpinBoxOffsetRoll, SIGNAL(valueChanged(double)), this, SLOT(handEyeOffsetChanged()));
	connect(UI->doubleSpinBoxOffsetPitch, SIGNAL(valueChanged(double)), this, SLOT(handEyeOffsetChanged()));
	connect(UI->doubleSpinBoxOffsetYaw, SIGNAL(valueChanged(double)), this, SLOT(handEyeOffsetChanged()));

	connect(UI->radioButtonRobotVisuCurrent, SIGNAL(clicked()), this, SLOT(setupRobotVisu()));
	connect(UI->radioButtonRobotVisuIk, SIGNAL(clicked()), this, SLOT(setupRobotVisu()));
	connect(UI->radioButtonRobotVisuGraspingMotion, SIGNAL(clicked()), this, SLOT(setupRobotVisu()));
	connect(UI->horizontalSliderGraspingMotion, SIGNAL(valueChanged(int)), this, SLOT(solutionSliderMoved()));

	connect(UI->horizontalSliderThreshold, SIGNAL(valueChanged(int)), this, SLOT(threshSliderMoved()));
	//UI->lineEditObjName->setText(QString("LegoGate"));
	UI->lineEditObjName->setText(QString("LegoXWing"));
	//UI->lineEditObjPath->setText(QString("objects/riceBox_iCub_gr0.25_right.xml"));
	//UI->lineEditObjPath->setText(QString("objects/iCub/LegoGate_Grasps_RightHand_200.xml"));
	UI->lineEditObjPath->setText(QString("objects/iCub/LegoXWing2_RightHand_300.xml"));

	UI->doubleSpinBoxPosRoll->setValue(0.0);
	UI->doubleSpinBoxPosPitch->setValue(0.0);
	UI->doubleSpinBoxPosYaw->setValue(M_PI/2.0);

}


void SimoxGraspingPipelineControlWindow::closeEvent(QCloseEvent *event)
{
	quit();
	QMainWindow::closeEvent(event);
}


void SimoxGraspingPipelineControlWindow::quit()
{
	std::cout << "SimoxGraspingPipelineControlWindow: Closing" << std::endl;
	this->close();
	isClosed = true;
	SoQt::exitMainLoop();
}


int SimoxGraspingPipelineControlWindow::main()
{
	SoQt::show(this);
	SoQt::mainLoop();
	return 0;
}

void SimoxGraspingPipelineControlWindow::ping()
{
	pingValue++;
	if (pingValue>100)
	{
		pingValue = 0;
		
		// todo: need a mutex for reconnection
		//com->checkConnections(true);

		UI->checkBoxConnected->setChecked(com->getConnectionStatus());
	}
}

bool SimoxGraspingPipelineControlWindow::wasClosed()
{
	return isClosed;
}

bool SimoxGraspingPipelineControlWindow::setObjectPose( const std::string &objectName, Eigen::Matrix4f &pose )
{
	invalidateReachableGrasps();
	if (getObjectIndex(objectName)<0)
	{
		VR_WARNING << "No entry for object " << objectName << ", aborting..." << endl;
	}
	objectPose[objectName] = pose;
	moveObject(objectName,pose);
	//bool res =  com->setObjectPose(objectName,pose);
	// check displacement and send object pose to com accordingly 
	//handEyeOffsetChanged();
	bool res = true;

	//updateObjectPose();
	return res;
}

void SimoxGraspingPipelineControlWindow::invalidateReachableGrasps()
{
	reachable_grasps.clear();
	invalidateIk();
}

void SimoxGraspingPipelineControlWindow::invalidateIk()
{
	ikValid = false;
	grasping_motion.clear();
	ik_solution.clear();
	UI->horizontalSliderExecutePath->setSliderPosition(0);
	UI->radioButtonRobotVisuIk->setChecked(false);
	UI->radioButtonRobotVisuGraspingMotion->setChecked(false);
	UI->radioButtonRobotVisuCurrent->setChecked(true);
	UI->pushButtonExecutePos->setEnabled(false);
	UI->pushButtonExecutePath->setEnabled(false);
	UI->horizontalSliderGraspingMotion->setEnabled(false);
	UI->horizontalSliderGraspingMotion->setSliderPosition(0);
	setupRobotVisu();

	showGraspingMotion(); // remove any grasping motion visus
}

bool SimoxGraspingPipelineControlWindow::searchIK( const std::string &objectName, const std::string &graspName )
{
	invalidateIk();
	
	std::vector<float> jointValues;
	float graspQuality = 0.0f;
	clock_t startT = clock();
	bool res =  com->searchIK(objectName, graspName, jointValues, graspQuality);
	clock_t endT = clock();
	long timeMS = (long)(((float)(endT - startT) / (float)CLOCKS_PER_SEC) * 1000.0);

	if (res)
	{
		cout << "Found IK solution..." << endl;
		cout << "Joint values:" << endl;
		for (size_t i=0;i<jointValues.size();i++)
			cout << jointValues[i] << ",";
		cout << endl;
		cout << "Manipulability Quality:" << graspQuality << endl;
		ikValid = true;
		ik_solution = jointValues;
		grasping_motion.clear(); // no motion until now
		setupRobotVisu();
	} else
	{
		cout << "IK search failed..." << endl;
		invalidateIk();
	}
	setIkQueryText(true,res,graspName,graspQuality,timeMS);

	return res;
}

void SimoxGraspingPipelineControlWindow::searchIK()
{
	invalidateIk();
	if (UI->comboBoxObject->currentIndex()<0)
		return;
	if (UI->comboBoxReachableGrasps->currentIndex()<0)
		return;

	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();

	/*QString text2 = UI->comboBoxReachableGrasps->currentText();
	std::string text2S = text2.toStdString();*/
	std::map<std::string,float>::iterator i = reachable_grasps.begin();
	for (int j=0;j<UI->comboBoxReachableGrasps->currentIndex();j++)
	{
		if (i==reachable_grasps.end())
			return;
		i++;
	}
	if (i==reachable_grasps.end())
		return;
	std::string text2S = i->first;

	searchIK(textS,text2S);
}


void SimoxGraspingPipelineControlWindow::selectGrasp(int nr)
{
	invalidateIk();
	if (UI->comboBoxObject->currentIndex()<0)
		return;
	if (UI->comboBoxReachableGrasps->currentIndex()<0)
		return;

	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();

	if (nr<0 || nr>=(int)reachable_grasps.size())
	{
		VR_ERROR << "nr=" << nr << " too large ( reachable_grasps.size()=" << reachable_grasps.size() << ")" << endl;
		return;
	}
	std::map<std::string,float>::iterator i = reachable_grasps.begin();
	for (int j=0;j<nr;j++)
		i++;
	//i+=(unsigned int)nr;


	std::string text2S = i->first;
	//QString text2 = UI->comboBoxReachableGrasps->currentText();
	//std::string text2S = text2.toStdString();

	selectGrasp(textS,text2S);

}


void SimoxGraspingPipelineControlWindow::selectObject(int nr)
{
	invalidateReachableGrasps();
	if (UI->comboBoxObject->currentIndex()<0 || nr>=UI->comboBoxObject->count())
		return;
	UI->comboBoxReachableGrasps->clear();
	UI->comboBoxObject->setCurrentIndex(nr);
	QString text = UI->comboBoxObject->itemText(nr);
	std::string textS = text.toStdString();

	Eigen::Matrix4f m = getObjectPose (textS);
	Eigen::Vector3f pos,rpy;
	MathTools::eigen4f2rpy(m,rpy);
	pos = MathTools::getTranslation(m);
	UI->doubleSpinBoxPosX->setValue((double)pos(0));
	UI->doubleSpinBoxPosY->setValue((double)pos(1));
	UI->doubleSpinBoxPosZ->setValue((double)pos(2));
	UI->doubleSpinBoxPosRoll->setValue((double)rpy(0));
	UI->doubleSpinBoxPosPitch->setValue((double)rpy(1));
	UI->doubleSpinBoxPosYaw->setValue((double)rpy(2));

	// update offset computation
	handEyeOffsetChanged();

	com->setCurrentObject(textS);
}

void SimoxGraspingPipelineControlWindow::selectGrasp( const std::string &objectName, const std::string &graspName)
{
	invalidateIk();
	setIkQueryText(false,true,graspName,0.0f,0.0f);
	
	int indx = UI->comboBoxReachableGrasps->currentIndex();
	std::map<std::string,float>::iterator it = reachable_grasps.begin();
	int j=0;
	while (j<indx && it != reachable_grasps.end())
	{
		it++;
		j++;
	}

	std::string selectedGrasp = it->first;//UI->comboBoxReachableGrasps->currentText().toStdString();
	if (selectedGrasp != graspName)
	{
		int i = getGraspIndex(graspName);
		if (i>=0)
			UI->comboBoxReachableGrasps->setCurrentIndex(i);
	}
	if (!UI->checkBoxVisuReachableGrasps->isChecked())
		com->showGrasp(objectName,graspName);
}

void SimoxGraspingPipelineControlWindow::setObjectPose()
{
	invalidateReachableGrasps();
	if (UI->comboBoxObject->currentIndex()<0)
		return;

	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();
	float x[6];
	x[0] = (float)UI->doubleSpinBoxPosX->value();
	x[1] = (float)UI->doubleSpinBoxPosY->value();
	x[2] = (float)UI->doubleSpinBoxPosZ->value();	
	x[3] = (float)UI->doubleSpinBoxPosRoll->value();
	x[4] = (float)UI->doubleSpinBoxPosPitch->value();
	x[5] = (float)UI->doubleSpinBoxPosYaw->value();
	Eigen::Matrix4f m;
	MathTools::posrpy2eigen4f(x,m);
	setObjectPose(textS,m);
	updateObjectPose();
}

Eigen::Matrix4f SimoxGraspingPipelineControlWindow::getObjectPose( const std::string &objectName )
{
	Eigen::Matrix4f m = Eigen::Matrix4f::Identity();//com->getObjectPose(textS);
	if (objectPose.find(objectName) != objectPose.end())
	{
		m = objectPose[objectName];
	}
	return m;
}

void SimoxGraspingPipelineControlWindow::setObject( )
{
	invalidateReachableGrasps();
	std::string objectName = UI->lineEditObjName->text().toStdString();
	std::string filename = UI->lineEditObjPath->text().toStdString();
	setObject(objectName,filename);
}

bool SimoxGraspingPipelineControlWindow::setObject( const std::string &objectName, const std::string &filename )
{
	invalidateReachableGrasps();
	if (getObjectIndex(objectName)>=0)
	{
		cout << "Double entry, aborting..." << endl;
		return false;
	}
	com->setCurrentObject(objectName);
	bool res = com->setObject(objectName,filename);
	if (res)
	{
		QString text = objectName.c_str();
		cout << "Set Object OK" << endl;
		UI->comboBoxObject->addItem(text);
		UI->comboBoxObject->setCurrentIndex(UI->comboBoxObject->count()-1);
		//updateObjectPose();
	} else
	{
		cout << "Set Object FAILED" << endl;
	}
	return res;
}

bool SimoxGraspingPipelineControlWindow::updateObjectPose()
{
	invalidateReachableGrasps();
	if (UI->comboBoxObject->currentIndex()<0)
		return false;
	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();

	Eigen::Matrix4f m = getObjectPose (textS);
	Eigen::Vector3f pos,rpy;
	MathTools::eigen4f2rpy(m,rpy);
	pos = MathTools::getTranslation(m);
	UI->doubleSpinBoxPosX->setValue((double)pos(0));
	UI->doubleSpinBoxPosY->setValue((double)pos(1));
	UI->doubleSpinBoxPosZ->setValue((double)pos(2));
	UI->doubleSpinBoxPosRoll->setValue((double)rpy(0));
	UI->doubleSpinBoxPosPitch->setValue((double)rpy(1));
	UI->doubleSpinBoxPosYaw->setValue((double)rpy(2));
	updateReachableGrasps();
	return true;
}

bool SimoxGraspingPipelineControlWindow::removeObject( const std::string &objectName )
{
	invalidateReachableGrasps();
	int i = getObjectIndex(objectName);
	if (i>=0)
		UI->comboBoxObject->removeItem(i);
	else
		VR_WARNING << "Did not find object " << objectName << " in GUI's combo box..." << endl;
	if (objectPose.find(objectName) != objectPose.end())
	{
		objectPose.erase(objectName);
	}
	com->setCurrentObject("");
	return com->removeObject(objectName);
}

void SimoxGraspingPipelineControlWindow::removeObject()
{
	invalidateReachableGrasps();
	if (UI->comboBoxObject->currentIndex()<0)
		return;

	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();
	bool ok = removeObject(textS);
	if (ok)
	{
		cout << "Remove OK" << endl;
	} else
		cout << "Remove failed" << endl;
}

int SimoxGraspingPipelineControlWindow::getObjectIndex( const std::string &objectName )
{
	QString text = objectName.c_str();
	for (int i=0;i<UI->comboBoxObject->count();i++)
	{
		if (UI->comboBoxObject->itemText(i) == text)
		{
			return i;
		}
	}
	return -1;
}

int SimoxGraspingPipelineControlWindow::getGraspIndex( const std::string &graspName )
{

	std::map<std::string,float>::iterator i = reachable_grasps.begin();
	int j = 0;
	while (i!=reachable_grasps.end())
	{
		if (i->first == graspName)
		{
			return j;
		}
		j++;
		i++;
	}
	
	/*	
	QString text = graspName.c_str();

	for (int i=0;i<UI->comboBoxReachableGrasps->count();i++)
	{
		if (UI->comboBoxReachableGrasps->itemText(i) == text)
		{
			return i;
		}
	}*/
	return -1;
}

void SimoxGraspingPipelineControlWindow::showReachableGrasps(const std::string &objectName, bool enable)
{
	com->showReachableGrasps(objectName, reachable_grasps, enable);
	UI->checkBoxVisuReachableGrasps->setChecked(enable);
}

void SimoxGraspingPipelineControlWindow::showGraspingMotion()
{
	// enable, disable visu for motion
	bool enable = UI->checkBoxVisuGraspingMotion->isChecked();
	if (grasping_motion.size()==0)
		enable = false;
	com->showMotion(enable,grasping_motion);
}

void SimoxGraspingPipelineControlWindow::setupRobotVisu()
{
	if (UI->radioButtonRobotVisuCurrent->isChecked())
	{
		UI->horizontalSliderGraspingMotion->setEnabled(false);
		//UI->pushButtonExecutePos->setEnabled(false);
		com->showCurrentRobotState();
	} else if (UI->radioButtonRobotVisuIk->isChecked())
	{
		UI->horizontalSliderGraspingMotion->setEnabled(false);
		//UI->pushButtonExecutePos->setEnabled(false);
		if (!ikValid || ik_solution.size()==0)
		{	
			cout << "No ik solution!!!" << endl;
			return;
		}
		com->showConfiguration(ik_solution);
	} else if (UI->radioButtonRobotVisuGraspingMotion->isChecked())
	{
		if (grasping_motion.size()!=0)
		{
			UI->horizontalSliderGraspingMotion->setEnabled(true);
			//UI->pushButtonExecutePos->setEnabled(true);
		} else
		{
			UI->horizontalSliderGraspingMotion->setEnabled(false);
			//UI->pushButtonExecutePos->setEnabled(false);
		}
	}	
}

void SimoxGraspingPipelineControlWindow::showReachableGrasps()
{
	if (UI->comboBoxObject->currentIndex()<0)
		return;
	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();
	showReachableGrasps(textS,UI->checkBoxVisuReachableGrasps->isChecked());
	
}
void SimoxGraspingPipelineControlWindow::updateReachableGrasps()
{
	reachable_grasps.clear();
	UI->comboBoxReachableGrasps->clear();
	if (UI->comboBoxObject->currentIndex()<0)
		return;
	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();
	std::vector<std::string> gr;
	std::vector<float> qual;
	bool ok = com->getReachableGrasps(textS,gr,qual);
	for (int i=0;i<(int)gr.size();i++)
	{
		QString en = gr[i].c_str();
		en += " : ";
		en += QString::number(qual[i]);
		UI->comboBoxReachableGrasps->addItem(en);
		reachable_grasps[gr[i]] = qual[i];
	}
	
	showReachableGrasps();
	selectGrasp(0); // repaints selected grasp
}


void SimoxGraspingPipelineControlWindow::setIkQueryText(bool ikChecked, bool ikOk, const std::string &grasp, float quality, float timeMS)
{
	std::stringstream ss;
	ss << "IK Query for <" << grasp << ">: ";
	if (ikChecked)
	{
		if (ikOk)
			ss << "OK";
		else
			ss << "FAILED";
	} else
		ss << "no checked";
	UI->labelIKSolution->setText(QString(ss.str().c_str()));

	ss.str("");
	ss << "Quality: " << quality;
	UI->labelIKSolutionQuality->setText(QString(ss.str().c_str()));

	ss.str("");
	ss << "Query time: " << timeMS << " ms";
	UI->labelIKSolutionQuery->setText(QString(ss.str().c_str()));

}

void SimoxGraspingPipelineControlWindow::executeIk()
{
	if (!ikValid || ik_solution.size()==0)
		return;
	com->stopMotionExecution(true,false);
	com->goToPoseHipArm(ik_solution);
}

void SimoxGraspingPipelineControlWindow::executeSolutionPos()
{
	if (!ikValid || ik_solution.size()==0 || grasping_motion.size()==0)
		return;

	float pos = (float)UI->horizontalSliderGraspingMotion->value() / 1000.0f;
	int vPos = (float)(grasping_motion.size()-1) * pos;
	if (vPos<0)
		vPos = 0;
	if (vPos>=(int)grasping_motion.size())
		vPos = (int)grasping_motion.size() -1;
		return;
	std::vector<float> j = grasping_motion[vPos];
	com->stopMotionExecution(true,false);
	com->goToPoseHipArm(j);
}

Eigen::Matrix4f SimoxGraspingPipelineControlWindow::getOffsetFromGui()
{
	float x = (float)UI->doubleSpinBoxOffsetX->value();
	float y = (float)UI->doubleSpinBoxOffsetY->value();
	float z = (float)UI->doubleSpinBoxOffsetZ->value();	
	float ro = (float)UI->doubleSpinBoxOffsetRoll->value() * M_PI/180.0;;
	float pi = (float)UI->doubleSpinBoxOffsetPitch->value() * M_PI/180.0;;
	float ya = (float)UI->doubleSpinBoxOffsetYaw->value() * M_PI/180.0;;
	Eigen::Matrix4f displacement = Eigen::Matrix4f::Identity();
	MathTools::rpy2eigen4f(ro,pi,ya,displacement);
	displacement(0,3) = x;
	displacement(1,3) = y;
	displacement(2,3) = z;
	return displacement;
}

void SimoxGraspingPipelineControlWindow::handEyeOffsetChanged()
{
	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();	
	Eigen::Matrix4f m = getObjectPose(textS);
	moveObject(textS,m);
}

void SimoxGraspingPipelineControlWindow::moveObject(const std::string &objectStr, const Eigen::Matrix4f &m)
{
	Eigen::Matrix4f ma = m;
	if (UI->checkBoxUseOffset->isChecked())
	{	
		Eigen::Matrix4f displacement = getOffsetFromGui();
		ma = displacement*ma;
	}
	com->setObjectPose(objectStr,ma);
	updateReachableGrasps();
}

void SimoxGraspingPipelineControlWindow::getOffsetFromHandEyeCalib()
{
	if (com)
	{
		float x,y,z,ro,pi,ya;
		QString text = UI->comboBoxObject->currentText();
		std::string textS = text.toStdString();

		if (com->queryHandEyeOffset(textS,x,y,z,ro,pi,ya))
		{
			UI->doubleSpinBoxOffsetX->setValue((double)x);
			UI->doubleSpinBoxOffsetY->setValue((double)y);
			UI->doubleSpinBoxOffsetZ->setValue((double)z);
			UI->doubleSpinBoxOffsetRoll->setValue((double)ro);
			UI->doubleSpinBoxOffsetPitch->setValue((double)pi);
			UI->doubleSpinBoxOffsetYaw->setValue((double)ya);
			handEyeOffsetChanged();
		}
	}
}

void SimoxGraspingPipelineControlWindow::checkObstacleModel()
{
	bool objPresent = UI->comboBoxObject->count()>0;
	int indx = 0;
	if (objPresent)
		indx = UI->comboBoxObject->currentIndex();
	bool enable = UI->checkBoxObstacle->isChecked();
	const std::string tab = "Obstacle";
	const std::string file = "objects/WaterBottleSmall.xml";
	if (!enable)
	{
		removeObject(tab);
		return;
	}
	setObject(tab,file);
	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	if (objPresent)
	{
		QString text = UI->comboBoxObject->itemText(indx);
		std::string textS = text.toStdString();
		pose = objectPose[textS];


		// z pose
#ifdef USE_GATE
		pose(2,3) -= 125.0f;
#else
		// USE_XWING
		pose(2,3) -= 30.0f;
#endif
	}
	// bottle height
	pose(2,3) += 120.0f;
	setObjectPose(tab,pose);
	if (objPresent)
	{
		selectObject(indx);
	}
}

void SimoxGraspingPipelineControlWindow::checkTableModel()
{
	bool enable = UI->checkBoxTable->isChecked();
	const std::string tab = "Table";
	if (!enable)
	{
		com->removeObject(tab);
		return;
	}
	com->setObject(tab,"objects/box1000x1000.xml");

	Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
	if (UI->comboBoxObject->count()>0)
	{
		QString text = UI->comboBoxObject->currentText();
		std::string textS = text.toStdString();
		pose = objectPose[textS];
	}
#ifdef USE_GATE
	pose(2,3) -= 125.0f;
#else
	// USE_XWING
	pose(2,3) -= 110.0f;
#endif
	if (UI->checkBoxUseOffset->isChecked())
	{	
		Eigen::Matrix4f displacement = getOffsetFromGui();
		pose = pose*displacement;
	}
	com->setObjectPose(tab,pose);
}

void SimoxGraspingPipelineControlWindow::goToStartPose()
{
	com->stopMotionExecution(true,false);
	com->goToInitPose();
}

void SimoxGraspingPipelineControlWindow::buildPath()
{
	if (!ikValid)
	{
		cout << "No ik solution..." << endl;
		return;
	}
	UI->horizontalSliderExecutePath->setEnabled(false);
	UI->horizontalSliderExecutePath->setSliderPosition(0);
	cout << "Building path from current config to ik solution" << endl;
	std::vector<float> currentConfig;
	if (!com->getCurrentConfigArm(currentConfig,true))
	{
		cout << "Could not get current config..." << endl;
		return;
	}

	cout << "Start pose:";
	MathTools::print(currentConfig);
	cout << "Goal pose:";
	MathTools::print(ik_solution);

	grasping_motion.clear();
	com->stopMotionExecution();
	if (!com->planMotion(currentConfig,ik_solution,grasping_motion))
	{
		VR_ERROR << "Failed planning collision-free motion..." << endl;
		grasping_motion.clear();
		return;
	}

	// create intermediate samples if needed
	sampleSolutionPath(grasping_motion,0.05f);

	// if here: we have a valid grasping motion
	UI->horizontalSliderExecutePath->setEnabled(true);
	UI->horizontalSliderGraspingMotion->setValue(0);

	UI->pushButtonExecutePos->setEnabled(true);
	UI->pushButtonExecutePath->setEnabled(true);

	// check if we should repaint motion visu
	showGraspingMotion();
}

void SimoxGraspingPipelineControlWindow::solutionSliderMoved()
{
	// VISU
	if (grasping_motion.size()==0)
		return;
	//UI->horizontalSliderExecutePath->setValue(UI->horizontalSliderGraspingMotion->value());
	float pos = (float)UI->horizontalSliderGraspingMotion->value() / 1000.0f;
	int vPos = grasping_motion.size() * pos;
	if (vPos<0 || vPos>=(int)grasping_motion.size())
		return;
	std::vector<float> j = grasping_motion[vPos];
	com->showConfiguration(j);

}

void SimoxGraspingPipelineControlWindow::threshSliderMoved()
{
	float pos = (float)UI->horizontalSliderThreshold->value();
	com->setSegThresh(pos);
}

bool SimoxGraspingPipelineControlWindow::sampleSolutionPath(std::vector< std::vector<float> > &sol, float sampleDist_cspace)
{
	std::vector< std::vector<float> > newMotion;
	for (size_t i=1;i<sol.size();i++)
	{
		std::vector<float> d;
		float dist = 0;
		for (size_t j=0;j<sol[i].size();j++)
		{
			float e = sol[i][j]-sol[i-1][j];
			d.push_back(e);
			dist += e*e;
		}
		dist = sqrtf(dist);

		newMotion.push_back(sol[i-1]);
		std::vector<float> tmpEntry;
		int steps = (int)(dist / sampleDist_cspace) +1;
		for (int k=1;k<=steps;k++)
		{
			tmpEntry.clear();
			float currentStep = (float)k/(float)steps;
			for (size_t j=0;j<sol[i].size();j++)
			{
				float e = sol[i-1][j] + d[j]*currentStep;
				tmpEntry.push_back(e);
			}
			newMotion.push_back(tmpEntry);
		}
	}
	sol = newMotion;
	return true;
}

void SimoxGraspingPipelineControlWindow::executePath()
{
	float pos = (float)UI->horizontalSliderExecutePath->value() / 1000.0f;
	int vPos = (int)((float)grasping_motion.size() * pos + 0.5f);
	if (vPos<=0)
		return;
	if (vPos>(int)grasping_motion.size())
		vPos = (int)grasping_motion.size();

	//build path
	std::vector< std::vector<float> > newPath;
	for (int i=0;i<vPos;i++)
	{
		newPath.push_back(grasping_motion[i]);
	}	
	com->startMotionExecution(newPath);
}

void SimoxGraspingPipelineControlWindow::stopMovement()
{
	com->stopMotionExecution();
}

void SimoxGraspingPipelineControlWindow::searchLego()
{
	cout << "Searching lego bricks..." << endl;
	int nrLoops = UI->spinBoxLocalizations->value();
	nrLoops--;
	Bottle cmd;
	cmd.addString("localize");
	Eigen::Matrix4f lego_global;
	Eigen::Matrix4f obst_global;

	bool searchObstacle = UI->checkBoxObstacle->isChecked();


	if (!com->localizeLego(false,lego_global,searchObstacle,obst_global))
	{
		VR_ERROR << "Could not determine pose of lego..." << endl;
		return;
	}

	for (int i=0;i<nrLoops;i++)
		com->localizeLego(true,lego_global,searchObstacle,obst_global);	

	// update visu
	if (UI->comboBoxObject->currentIndex()<0)
		return;

	QString text = UI->comboBoxObject->currentText();
	std::string textS = text.toStdString();
	setObjectPose(textS,lego_global);

	// implicit assumption: the obstacle is named "Obstacle"
	if (searchObstacle && UI->checkBoxObstacle->isChecked())
	{
		std::string textSO = "Obstacle";
		setObjectPose(textSO,obst_global);
	}
}


void SimoxGraspingPipelineControlWindow::saveScene()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer [120];
	time ( &rawtime );
	timeinfo = localtime ( &rawtime );
	strftime (buffer,120,"graspingPipeline_%y_%m_%d__%H_%M_%S.xml",timeinfo);
	std::string filename = buffer;
	com->saveScene(filename);
}

void SimoxGraspingPipelineControlWindow::openHand()
{
	com->openHand();
}

void SimoxGraspingPipelineControlWindow::closeHand()
{
	com->closeHand();
}

void SimoxGraspingPipelineControlWindow::liftHand()
{
	com->liftHand();
}
