#include <iostream>
#include <yarp/os/Network.h>
#include "robot.hpp"

using namespace HandSome;
using namespace yarp::os;

int main(int argc, char *argv[])
{
	Network yarp;

    Property params;
    params.fromCommand(argc, argv);

    if (!params.check("robot"))
    {
        cout<<"Please specify the name of the robot!"<<endl;
        cout<<"--robot name (e.g. icub)"<<endl;
        return -1;
    }
    
    Robot* robot = new Robot(0);
    robot->init(params.find("robot").asString().c_str());
    
    cout<<robot->get_part(LEFT_ARM)<<endl;
    cout<<robot->get_part(RIGHT_ARM)<<endl;
    cout<<robot->get_part(NECK)<<endl;
    
    cout<<endl;
    
    cout<<robot->get_driver(LEFT_ARM)<<endl;
    cout<<robot->get_driver(RIGHT_ARM)<<endl;
    //cout<<robot->get_driver(NECK)<<endl;
    
    cout<<endl;
    
    cout<<robot->get_limb(LEFT_ARM)<<endl;
    cout<<robot->get_limb(RIGHT_ARM)<<endl;
    //cout<<robot->get_limb(NECK)<<endl;
    
    cout<<endl;
    
    cout<<robot->get_torso()<<endl;
    
    cout<<endl;
    
    cout<<robot->get_dof(LEFT_ARM)<<endl;
    cout<<robot->get_dof(RIGHT_ARM)<<endl;
    //cout<<robot->get_dof(NECK)<<endl;
    
    return 0;
}
