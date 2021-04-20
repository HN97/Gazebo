/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
// local position and velocity from FCU
#include <geometry_msgs/PoseStamped.h>
// supports matrix and vector support
#include <eigen3/Eigen/Core>
// changing arming status
#include <mavros_msgs/CommandBool.h>
// Set FCU operation mode
#include <mavros_msgs/SetMode.h>
// source of arming event
#include <mavros_msgs/State.h>
// including tf library
#include <tf/tf.h>
// supports transformations quaternions
#include <eigen3/Eigen/Geometry>
// imu data orientation computed by FCU
#include <sensor_msgs/Imu.h>
// file input  output stream
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <sstream>
#include <cstring> 
#include <string>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>

/*this provides scope for identifiers*/
using namespace std;
using namespace Eigen;

/*declaring variables*/
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped gps_pose;
mavros_msgs::State current_state;
/*variable*/

static char var_active_status[20];
static int STATE_CHECK = 1;

/*declaring a 3*3 matrix*/
Matrix3f R;

/*getting the state into a pointer*/
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

/*vectors to store position before and after*/
Vector3f var_offset_pose;
Vector3f positionbe;
Vector3f positionaf;

/*storing gps data in pointer*/
void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    gps_pose=*msg;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    double x,y,z,w;

    x =msg->orientation.x;
    y =msg->orientation.y;
    z =msg->orientation.z;
    w =msg->orientation.w;
    
    /*making a quaternion of position*/
    Quaternionf quat;
    quat=Eigen::Quaternionf(w,x,y,z);
  
    /*making rotation matrix from quaternion*/
    R=quat.toRotationMatrix();
    // cout << "R=" << endl << R << endl;
}


void posecallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
}

void set_target_position_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    // if(msg->header.frame_id == "base_link")
    // {
    //     ROS_INFO("Control Follow Body FLU frame");


    // }
    // else
    // {
/*      For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X
*/
        ROS_INFO("Control Follow Local ENU frame");
        pose.pose.position.x= msg->pose.position.x;
        pose.pose.position.y= msg->pose.position.y;
        pose.pose.position.z= msg->pose.position.z;
        cout << "X: " << pose.pose.position.x << "\t" << "Y: " << pose.pose.position.y << "\t" << "Z: " << pose.pose.position.z << endl;
    // }
}

void set_target_yaw_callback(const std_msgs::Float32::ConstPtr& msg)
{

}

void custom_activity_callback(const std_msgs::String::ConstPtr& msg)
{
    strcpy(var_active_status, msg->data.c_str());
    cout << var_active_status << endl;
}

int main(int argc, char **argv)
{

// cout<< "______  __   __    ___     _____    _____ " <<endl;
// cout<< "| ___ \ \ \ / /   /   |   / ___ \  | ___ \" <<endl;
// cout<< "| |_/ /  \ V /   / /| |   | | | |  | |_/ |" <<endl;
// cout<< "|  __/   /   \  / /_| |   | | | |  |  __ /" <<endl;
// cout<< "| |     / /^\ \ \___  |   | |_| |  | |_/ \" <<endl;
// cout<< "\_|     \/   \/     |_/   \_____/  \_____/" <<endl;
    cout << "Staring mode OFFBOARD CONTROL ..........." << endl;
    /* Init position */
    cout << "PLEASE ENTER POSITION INIT Follow Local ENU frame [x y z]: ";
    cin  >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z ;
    cout << "pose : "<< pose.pose.position.x << "  "<< pose.pose.position.y <<"  "<< pose.pose.position.z << endl;
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("/mavros/imu/data",10,imuCallback);
    ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose",100,gpsCallback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    // ros::Subscriber pose_sub = nh.subscribe("/tf_list", 10, get_params_cb);

    ros::Subscriber position_target_sub = nh.subscribe<geometry_msgs::PoseStamped>("cmd/set_pose/position1",30,set_target_position_callback);
    ros::Subscriber yaw_target_sub = nh.subscribe<std_msgs::Float32>("cmd/set_pose/orientation",10,set_target_yaw_callback);
    ros::Subscriber custom_activity_sub = nh.subscribe<std_msgs::String>("cmd/set_activity/type",10,custom_activity_callback);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    /*wait for FCU connection*/
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    /*send a few setpoints before starting*/
    cout << "LOADING";
    for(int i = 20; ros::ok() && i > 0; --i)
    {

        cout <<".";
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    cout<<"100%"<<endl;

    mavros_msgs::SetMode offb_set_mode, offset_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offset_mode.request.custom_mode = "AUTO.LAND";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if (STATE_CHECK == 1)
        {
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                {
                    ROS_INFO("Offboard enabled");
                }

                last_request = ros::Time::now();
            } 
            else
            {
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
                {
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success)
                    {
                        ROS_INFO("Vehicle armed");
                    }

                    last_request = ros::Time::now();
                }
            }
        }

        if (strcmp(var_active_status,"LAND") == 0)
        {
            last_request = ros::Time::now();
            // ROS_INFO("Detected Marker & Landing ....!!!");
            if( current_state.mode != "AUTO.LAND" )
            {
                offset_mode.request.custom_mode = "AUTO.LAND";
                if( set_mode_client.call(offset_mode) && offset_mode.response.mode_sent)
                {
                    ROS_INFO("AUTO LAND enabled");
                    STATE_CHECK = 0;
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
