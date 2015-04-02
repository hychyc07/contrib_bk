#include "bridgeThread.h"

void BridgeThread::setHome(double x, double y, double angle)
{
    mutex_home.wait();
    ikart_home.x=x;
    ikart_home.y=y;
    ikart_home.t=angle;
    mutex_home.post();
}

void BridgeThread::setHome()
{
    mutex_home.wait();
    ikart_home.x=ikart_current_position.x;
    ikart_home.y=ikart_current_position.y;
    ikart_home.t=ikart_current_position.t;
    mutex_home.post();
}

void BridgeThread::setUserTarget(int id, double x, double y, double angle)
{
    if (id<10)
    {
        user_target[id].x= x;
        user_target[id].y= y;
        user_target[id].t= angle;
    }
}

void BridgeThread::getHome(double &x, double &y, double &angle)
{
    mutex_home.wait();
    x=ikart_home.x;
    y=ikart_home.y;
    angle=ikart_home.t;
    mutex_home.post();
}

int BridgeThread::setGoal(double x, double y, double angle)
{
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "home";
    goal.target_pose.header.stamp = ros::Time::now();
    geometry_msgs::Quaternion goal_quat= tf::createQuaternionMsgFromYaw(angle/180.0*M_PI);

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation = goal_quat;

   // ac->cancelAllGoals();
    ac->sendGoal(goal);
    return 0;
}

int BridgeThread::navigationStop()
{
    ac->cancelAllGoals();
    return 0;    
}

string BridgeThread::getNavigationStatus()
{
    string s = "UNKNOWN";
    int i = ac->getState().state_;
    switch (i)
    {
        case actionlib::SimpleClientGoalState::PENDING:
        s = "PENDING"; break;
        case actionlib::SimpleClientGoalState::ACTIVE:
        s = "ACTIVE"; break;
        case actionlib::SimpleClientGoalState::PREEMPTED:
        s = "PREEMPTED"; break;
        case actionlib::SimpleClientGoalState::SUCCEEDED:
        s = "SUCCEEDED"; break;
        case actionlib::SimpleClientGoalState::ABORTED:
        s = "ABORTED"; break;
        case actionlib::SimpleClientGoalState::REJECTED:
        s = "REJECTED"; break;
        /*case actionlib::SimpleClientGoalState::PREEMPTING:
        s = "PREEMPTING"; break;
        case actionlib::SimpleClientGoalState::RECALLING:
        s = "RECALLING"; break;*/
        case actionlib::SimpleClientGoalState::RECALLED:
        s = "RECALLED"; break;
        case actionlib::SimpleClientGoalState::LOST:
        s = "LOST"; break;     
    }
    return s;
}

int BridgeThread::getGoal(double &x, double &y, double &angle)
{
    return 0;
}

void BridgeThread::getLocalizedPos(double &x, double &y, double &angle)
{
    mutex_localiz.wait();
    x=ikart_current_position.x;
    y=ikart_current_position.y;
    angle=ikart_current_position.t;
    mutex_localiz.post();
}

void BridgeThread::printStats()
{   
    static int life=0;
    life++; 
    int max_tpt = int(1000.0/thread_period);
    fprintf (stdout," TIMEOUTS: thread:%3d(%3d) tot:%3d", timeout_thread,max_tpt,timeout_thread_tot);
    fprintf (stdout,"     laser:%3d(%3d) tot:%3d", timeout_laser,   max_tpt,timeout_laser_tot);
    fprintf (stdout,"     odometry:%3d(%3d) tot:%3d", timeout_odometry,max_tpt,timeout_odometry_tot);
    fprintf (stdout,"     odometer:%3d(%3d) tot:%3d", timeout_odometer,max_tpt,timeout_odometer_tot);
    fprintf (stdout,"     life:%3d", life);
    fprintf (stdout,"\n");
    timeout_laser = 0;
    timeout_odometry = 0;
    timeout_odometer = 0;
    timeout_thread = 0;
}
