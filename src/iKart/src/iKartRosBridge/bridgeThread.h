/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author: Marco Randazzo
 * email:  marco.randazzo@iit.it
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#ifndef BRIDGE_THREAD_H
#define BRIDGE_THREAD_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Stamp.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h> 
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h> 
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h> 
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include "odometer.h"

#include <sensor_msgs/PointCloud.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> tPointCloud;

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

class BridgeThread: public yarp::os::RateThread
{
    private:
    int    thread_period;
    
    double distance_traveled ;
    double angle_traveled ;
    
    bool   enable_odom_tf;
    int    laser_step;
    double last_laser[1080];
    int    laser_step2;
    double last_laser2[1080];

    double command_x  ;
    double command_y  ;
    double command_t  ;

    yarp::os::Semaphore mutex_command;
    yarp::os::Semaphore mutex_localiz;
    yarp::os::Semaphore mutex_home;
    
    
    
    class ikart_pose
    {
        public:
        double x;
        double y;
        double t;
        double vx;
        double vy;
        double w;
        ikart_pose() {x=0; y=0; t=0; vx=0; vy=0; w=0;}
    };
    ikart_pose ikart_home;
    ikart_pose user_target[10];

    ikart_pose ikart_odom;
    ikart_pose ikart_current_position;

    protected:
    ros::NodeHandle          *nh;
    ros::Publisher           laser_pub;
    ros::Publisher           laser_pub2;
    ros::Publisher           pcloud_pub;
    ros::Publisher           odometry_pub;
    ros::Publisher           footprint_pub;
    ros::Publisher           odometer_pub;
    ros::Publisher           marker_pub;
    ros::Subscriber          command_sub;
    tf::TransformBroadcaster *tf_broadcaster;
    tf::TransformListener    *tf_listener;
    ResourceFinder           &rf;
    Property                 iKartCtrl_options;
    BufferedPort<Bottle>     input_laser_port; 
    BufferedPort<Bottle>     input_laser_port2; 
    BufferedPort<Bottle>     input_odometry_port; 
    BufferedPort<Bottle>     input_odometer_port; 
    BufferedPort<ImageOf<PixelRgbFloat> >  input_pcloud_port; 
    BufferedPort<Bottle>     output_command_port; 
    BufferedPort<Bottle>     output_localization_port;
    BufferedPort<Bottle>     output_3d_localization_port;
    int                      timeout_thread;
    int                      timeout_thread_tot;
    int                      timeout_laser;
    int                      timeout_laser2;
    int                      timeout_odometry;
    int                      timeout_odometer;
    int                      timeout_laser_tot;
    int                      timeout_laser_tot2;
    int                      timeout_odometry_tot;
    int                      timeout_odometer_tot;
    int                      command_wdt;
    yarp::os::Stamp          timestamp_localization;
    yarp::os::Stamp          timestamp_command;

    geometry_msgs::PolygonStamped footprint;
    sensor_msgs::LaserScan        scan;
    sensor_msgs::LaserScan        scan2;
    visualization_msgs::Marker    marker;
    tPointCloud::Ptr*             pcloud;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> *ac;
    
    public:
    
    BridgeThread(unsigned int _period, ResourceFinder &_rf, Property options,
               string _remoteName, string _localName) :
               RateThread(_period),     rf(_rf),
               iKartCtrl_options (options)
    {
        thread_period = _period;
        command_x = 0.0;
        command_y = 0.0 ;
        command_t = 0.0 ;
        distance_traveled = 0.0;
        angle_traveled    = 0.0;
        timeout_thread   = 0;
        timeout_thread_tot   = 0;
        command_wdt = 100;
        timeout_laser = 0;
        timeout_odometry = 0;
        timeout_odometer = 0;
        timeout_laser_tot = 0;
        timeout_odometry_tot = 0;
        timeout_odometer_tot = 0;
        
        //printf("rf options: %s",rf.toString().c_str());
        laser_step = rf.check("laser_resample",Value(1)).asInt();
        enable_odom_tf = !(rf.check("no_odom_tf"));
        if (enable_odom_tf)
            printf ("publishing odom tf \n");
        else
            printf ("not publishing odom tf\n");
        if (laser_step<=0)
            laser_step = 1;
        printf ("Using %d laser measurments each scan (max: 1080).\n", 1080/laser_step);

	laser_step2 = laser_step;
        footprint.header.frame_id = "base_link";
	footprint.polygon.points.resize(12);
	double r = 0.38;
	for (int i=0; i< 12; i++)
	{
	   double t = M_PI*2/12* i;
           footprint.polygon.points[i].x = r*cos(t);
	   footprint.polygon.points[i].y = r*sin(t);
	}

	pcloud = new tPointCloud::Ptr (new tPointCloud);
	(*pcloud)->header.frame_id ="robot_root";
    }

    void setHome();  
    void setUserTarget(int id, double x, double y, double angle);
    void setHome(double x, double y, double angle);
    void getHome(double &x, double &y, double &angle);  
    int  setGoal(double x, double y, double angle);
    int  navigationStop();
    int  getGoal(double &x, double &y, double &angle);
    string getNavigationStatus();
    void getLocalizedPos(double &x, double &y, double &angle);
    void printStats();
    
    // void stringCallback(const std_msgs::StringConstPtr& message) {}
    
    void commandCallback(const geometry_msgs::Twist& event)
    {
        mutex_command.wait();
        command_x = -event.linear.y;
        command_y = +event.linear.x;
        command_t = -event.angular.z*180.0/M_PI;
        command_wdt = 100;
        mutex_command.post();
    }

    virtual bool threadInit()
    {
        int argc = 0;
        char** argv = 0;
        ros::init (argc, argv, "ikart_ros_bridge");
        printf("Starting1\n");
        nh = new ros::NodeHandle();
        printf("Starting2\n");
        ros::Time::init();
        pcloud_pub      = nh->advertise<tPointCloud>                    ("/ikart_ros_bridge/pcloud_out",      1);
        footprint_pub   = nh->advertise<geometry_msgs::PolygonStamped>  ("/ikart_ros_bridge/footprint",       1);
        laser_pub       = nh->advertise<sensor_msgs::LaserScan>         ("/ikart_ros_bridge/laser_out",       1);
        laser_pub2      = nh->advertise<sensor_msgs::LaserScan>         ("/ikart_ros_bridge/laser2_out",      1);
        odometer_pub    = nh->advertise<ikart_ros_bridge::odometer>     ("/ikart_ros_bridge/odometer_out",    1);
        odometry_pub    = nh->advertise<nav_msgs::Odometry>             ("/ikart_ros_bridge/odometry_out",    1);
        marker_pub      = nh->advertise<visualization_msgs::Marker>     ("/ikart_ros_bridge/marker_out",      1);
        tf_broadcaster  = new tf::TransformBroadcaster;
        tf_listener     = new tf::TransformListener;
        ac              = new actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ("move_base", true);
        printf("Starting3\n");
        //wait for the action server to come up  
        /*while(!ac->waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }*/

        //subscribe ros topics, open yarp ports and make connections
        command_sub = nh->subscribe("cmd_vel", 1, &BridgeThread::commandCallback, this);

        input_pcloud_port.open("/ikart_ros_bridge/stereo_img:i");
        input_laser_port.open("/ikart_ros_bridge/laser:i");
        input_laser_port2.open("/ikart_ros_bridge/laser2:i");
        input_odometry_port.open("/ikart_ros_bridge/odometry:i");
        input_odometer_port.open("/ikart_ros_bridge/odometer:i");
        output_command_port.open("/ikart_ros_bridge/command:o");
        output_localization_port.open("/ikart_ros_bridge/localization:o");
        output_3d_localization_port.open("/ikart_ros_bridge/localization3D:o");

        bool conn_lsr = Network::connect ("/ikart/laser:o","/ikart_ros_bridge/laser:i","udp");
        bool conn_odm = Network::connect ("/ikart/odometry:o","/ikart_ros_bridge/odometry:i","udp");
        bool conn_odt = Network::connect ("/ikart/odometer:o","/ikart_ros_bridge/odometer:i","udp");
        
        if (!conn_lsr || !conn_odm || !conn_odt)
        {
            printf("Connection to iKartCtrl failed\n");
        }

        //prepare here the maker message, to save time during the main loop
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker.lifetime = ros::Duration();
        marker.pose.position.x = 0;
        marker.pose.position.y = 0;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.frame_locked = true;
    
        //prepare here the laser scan message, to save time during the main loop
        int num_readings = 1080/laser_step;
        int laser_frequency = 1080/laser_step;        
        scan.header.frame_id = "base_laser";
        scan.angle_min = -2.35619;
        scan.angle_max =  2.35619;
        scan.angle_increment = 4.7123889 / num_readings;
        scan.time_increment = (1 / laser_frequency) / (num_readings);
        scan.range_min = 0.0;
        scan.range_max = 30; //m 
        scan.ranges.resize(num_readings);
        scan.intensities.resize(num_readings);    
        for (int i=0; i< 1080/laser_step; i++)
        {
            last_laser[i] = 0.0;
            scan.ranges[i] = 0.0;
            scan.intensities[i]=101;
        }
 
        num_readings = 1080/laser_step2;
        laser_frequency = 1080/laser_step2;        
        scan2.header.frame_id = "base_laser";
        scan2.angle_min = -2.35619;
        scan2.angle_max =  2.35619;
        scan2.angle_increment = 4.7123889 / num_readings;
        scan2.time_increment = (1 / laser_frequency) / (num_readings);
        scan2.range_min = 0.0;
        scan2.range_max = 30; //m 
        scan2.ranges.resize(num_readings);
        scan2.intensities.resize(num_readings);    
        for (int i=0; i< 1080/laser_step2; i++)
        {
            last_laser2[i] = 0.0;
            scan2.ranges[i] = 0.0;
            scan2.intensities[i]=101;
        }        
        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            printf("Bridge thread started successfully\n");
        else
            printf("Bridge thread did not start\n");
    }

    virtual void run()
    {
        //********************************************* TIMEOUT CHECK ******************************************
        static double wdt_old=Time::now();
        double wdt=Time::now();
        if (wdt-wdt_old > double(thread_period)/1000.0 + 0.10) 
        {
            timeout_thread++;
            timeout_thread_tot++;
        }     
        //printf("%f %f\n",wdt-wdt_old , double(thread_period)/1000.0 + 0.010);
        wdt_old=wdt;

	    //********************************************* FOOTPRINT PART *****************************************
	    footprint_pub.publish (footprint);

        //********************************************* LASER PART *********************************************
        Bottle *laser_bottle = 0;
        laser_bottle = input_laser_port.read(false);
        static ros::Time now;

        if (laser_bottle)
        {
           for(int j=0, i=0; j<1080/laser_step; j++, i=i+laser_step)
           {
                last_laser[j] = laser_bottle->get(i).asDouble();
                scan.ranges[j]=last_laser[j];
                //scan.intensities[j]=101;
                now = ros::Time::now();
                scan.header.stamp.sec = now.sec;
                scan.header.stamp.nsec = now.nsec;
           }
        }
        else  
        {    
           timeout_laser++;
           timeout_laser_tot++;
        }

        laser_pub.publish (scan);

        //********************************************* LASER2 (OPTIONAL) PART *********************************
        Bottle *laser_bottle2 = 0;
        laser_bottle2 = input_laser_port2.read(false);
        static ros::Time now2;

        if (laser_bottle2)
        {
           for(int j=0, i=0; j<1080/laser_step2; j++, i=i+laser_step2)
           {
                last_laser2[j] = laser_bottle2->get(i).asDouble();
                scan2.ranges[j]=last_laser2[j];
                //scan.intensities[j]=101;
           }
           now2 = ros::Time::now();
           scan2.header.stamp.sec = now2.sec;           
           scan2.header.stamp.nsec = now2.nsec;
        }
        else  
        {    
           timeout_laser2++;
           timeout_laser_tot2++;
        }

        laser_pub2.publish (scan2);

	    //********************************************* PCL (OPTIONAL) PART ***************************************
        ImageOf<PixelRgbFloat> *pcloud_image = 0;
        pcloud_image = input_pcloud_port.read(false);

        if (pcloud_image)
        { 
	       int w_size =  pcloud_image->width();
	       int h_size =  pcloud_image->height();
           int c=0;
	       //printf ("pcl_size: %d %d\n", w_size, h_size);
           (*pcloud)->points.clear();
	       for (int x=0; x<w_size; x++)
           {
		      for (int y=0; y<h_size; y++)
		      {
			     yarp::sig::PixelRgbFloat p = pcloud_image->pixel(x,y);
                 //printf ("pcl_size: %f %f %f\n", p.r,p.g,p.b);
			     if (p.r!=0 && p.g!=0 && p.b!=0)
			     {
                  (*pcloud)->points.push_back (pcl::PointXYZ(p.r,p.g,p.b)); 
		 	      c++;
                 }
		      }
    	   }
           (*pcloud)->height = 1;
	       (*pcloud)->width = c;
        }

	    pcloud_pub.publish (*pcloud);
        
        //********************************************* CREATE NEW TF *********************************************
        tf::StampedTransform ikart_trans(tf::Transform(tf::createQuaternionFromYaw(-90/180.0*M_PI), tf::Vector3(0.0,0.0,0.0)),now, "base_link", "ikart_root");
        tf_broadcaster->sendTransform(ikart_trans);

        tf::StampedTransform laser_trans(tf::Transform(tf::createQuaternionFromYaw(+90/180.0*M_PI), tf::Vector3(0.000,0.245,0.2)),now, "ikart_root", "base_laser");
        tf_broadcaster->sendTransform(laser_trans);

        tf::StampedTransform robot_trans(tf::Transform(tf::createQuaternionFromYaw(-90/180.0*M_PI), tf::Vector3(0.000,0.111,0.9)),now, "ikart_root", "robot_root");
        tf_broadcaster->sendTransform(robot_trans);

        mutex_home.wait();
        tf::StampedTransform home_trans(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(ikart_home.x,ikart_home.y,0.0)),now, "map", "home");
        tf_broadcaster->sendTransform(home_trans);
        mutex_home.post();

        //mutex_home.wait();
        tf::StampedTransform user_trans1(tf::Transform(tf::createQuaternionFromYaw(user_target[0].t/180.0*M_PI), tf::Vector3(user_target[0].x,user_target[0].y,0.0)),now, "map", "user1");
        tf_broadcaster->sendTransform(user_trans1);
        //mutex_home.post();

        //mutex_home.wait();
        tf::StampedTransform user_trans2(tf::Transform(tf::createQuaternionFromYaw(user_target[1].t/180.0*M_PI), tf::Vector3(user_target[1].x,user_target[1].y,0.0)),now, "map", "user2");
        tf_broadcaster->sendTransform(user_trans2);
        //mutex_home.post();
        
        //********************************************* READ TF      **********************************************
        tf::StampedTransform stamp_loc_trans;
        bool loc_running = tf_listener->canTransform ("/home", "/base_link", ros::Time(0), NULL); 
        timestamp_localization.update();
                
        if (loc_running)
        {       
            // The following try-catch has been added just for teaching purposues, since the block is
            // already protected by the tf_listener->canTransform() check.
            try
            {
                 tf_listener->lookupTransform("/home", "/base_link", ros::Time(0), stamp_loc_trans);
            }
            catch (tf::TransformException ex)
            {
                 ROS_ERROR("%s",ex.what()); 
            }

            geometry_msgs::TransformStamped loc_trans;
            tf::transformStampedTFToMsg (stamp_loc_trans, loc_trans);
            
            mutex_localiz.wait();
            ikart_current_position.x = loc_trans.transform.translation.x;
            ikart_current_position.y = loc_trans.transform.translation.y;
            ikart_current_position.t = tf::getYaw(loc_trans.transform.rotation)*180/M_PI;
            mutex_localiz.post();
            
            if (output_localization_port.getOutputCount()>0)
            {
                output_localization_port.setEnvelope(timestamp_localization);
                Bottle &m = output_localization_port.prepare();
                m.clear();
                m.addDouble(ikart_current_position.x);
                m.addDouble(ikart_current_position.y);
                m.addDouble(ikart_current_position.t);
                m.addDouble(ikart_current_position.vx);
                m.addDouble(ikart_current_position.vy);
                m.addDouble(ikart_current_position.w);
                output_localization_port.write();
            }

            if (output_3d_localization_port.getOutputCount()>0)
            {
                output_3d_localization_port.setEnvelope(timestamp_localization);
                Bottle &m = output_3d_localization_port.prepare();
                m.clear();
                m.addDouble(ikart_current_position.x);
                m.addDouble(ikart_current_position.y);
                m.addDouble(0.0);
                m.addDouble(0.0);
                m.addDouble(0.0);
                m.addDouble(ikart_current_position.t);
                output_3d_localization_port.write();
            }
        }
        else
        {
            printf("ERROR: tf_listener->canTransform\n");
        }
         
        //********************************************* COMMAND PART  *********************************************
        timestamp_command.update();
        mutex_command.wait();        
        command_wdt--;
        if (command_wdt<0)
        {
            //if no commands are received, than turn off control
            command_x = 0;
            command_y = 0;
            command_t = 0;
            command_wdt = 0;
        }
        if (output_command_port.getOutputCount()>0)
        {
            output_command_port.setEnvelope(timestamp_command);
            Bottle &b=output_command_port.prepare();
            b.clear();
            b.addInt(3);
            b.addDouble(command_x);
            b.addDouble(command_y);
            b.addDouble(command_t);
            output_command_port.write();
        }
        mutex_command.post();

        //********************************************* MAKER PART *********************************************
        marker_pub.publish(marker);
        
        //********************************************* ODOMETER PART *********************************************
        Bottle *odometer_bottle = 0;
        odometer_bottle = input_odometer_port.read(false);
        if (odometer_bottle)
        {
            distance_traveled = odometer_bottle->get(0).asDouble();
            angle_traveled = odometer_bottle->get(1).asDouble();
        }
        else
        {
            timeout_odometer++;
            timeout_odometer_tot++;
        }        
        ikart_ros_bridge::odometer odometer_msg;
        odometer_msg.distance=distance_traveled;
        odometer_msg.angle=angle_traveled;
        odometer_pub.publish(odometer_msg);

        //********************************************* ODOMETRY PART *********************************************
        Bottle *odometry_bottle = 0;
        odometry_bottle = input_odometry_port.read(false);

//#define ODOMETRY_DEBUG
#ifdef ODOMETRY_DEBUG
        ikart_odom.x  = 7;  //m
        ikart_odom.y  = 5;  //m
        ikart_odom.t  = 30; //deg
        ikart_odom.vx = 0;        
        ikart_odom.vy = 0;
        ikart_odom.vt = 0;
#else	
        geometry_msgs::TransformStamped odom_trans;
        nav_msgs::Odometry odom;
        if (odometry_bottle)
        {
            ikart_odom.x  = odometry_bottle->get(1).asDouble();
            ikart_odom.y  = -odometry_bottle->get(0).asDouble();
            ikart_odom.t  = -odometry_bottle->get(2).asDouble();
            ikart_odom.vx = odometry_bottle->get(4).asDouble();
            ikart_odom.vy = -odometry_bottle->get(3).asDouble();
            ikart_odom.w = -odometry_bottle->get(5).asDouble();
            odom_trans.header.stamp.sec = now.sec;
            odom_trans.header.stamp.nsec = now.nsec;
            odom.header.stamp.sec = now.sec;
            odom.header.stamp.nsec = now.nsec;
        }
        else
        {
            timeout_odometry++;
            timeout_odometry_tot++;
        }

#endif
        geometry_msgs::Quaternion odom_quat= tf::createQuaternionMsgFromYaw(ikart_odom.t/180.0*M_PI);
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = ikart_odom.x;
        odom_trans.transform.translation.y = ikart_odom.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        if (enable_odom_tf)
        {
           tf_broadcaster->sendTransform(odom_trans);
        }

        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = ikart_odom.x;
        odom.pose.pose.position.y = ikart_odom.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = ikart_odom.vx;
        odom.twist.twist.linear.y = ikart_odom.vy;
        odom.twist.twist.angular.z = ikart_odom.w/180.0*M_PI;
        odometry_pub.publish (odom);

        ros::spinOnce (); //@@@@@@@@@@
    }

    virtual void threadRelease()
    {
        input_pcloud_port.interrupt();
        input_pcloud_port.close();
        input_laser_port.interrupt();
        input_laser_port.close();
        input_odometry_port.interrupt();
        input_odometry_port.close();
        input_odometer_port.interrupt();
        input_odometer_port.close();
        output_command_port.interrupt();
        output_command_port.close();
        output_localization_port.interrupt();
        output_localization_port.close();    
        output_3d_localization_port.interrupt();
        output_3d_localization_port.close();        
    }

};

#endif
