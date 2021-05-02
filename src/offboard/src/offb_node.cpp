/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with PX4 1.10.1
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Core>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/TwistStamped.h>
#include <signal.h>
#include <cstdlib>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "MiniPID.h"

#define LOCAL    1
#define PID      2
#define PRECISION(x)    round(x * 100) / 100

/*this provides scope for identifiers*/
using namespace std;
using namespace Eigen;

/*declaring variables*/
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped vlocal_pose;
geometry_msgs::TwistStamped var_velocity;

time_t baygio = time(0);
tm *ltime       = localtime(&baygio);
ofstream outfile0;

mavros_msgs::State current_state;
/* variable */

static char var_active_status[20];
static int STATE_CHECK = 1;

/* declaring a 3*3 matrix */
Matrix3f R;

/* getting the state into a pointer */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

/* vectors to store position before and after */
Vector3f var_offset_pose;
Vector3f positionbe;
Vector3f positionaf;

/* storing gps data in pointer */
void mavrosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    vlocal_pose=*msg;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    double x,y,z,w;

    x =msg->orientation.x;
    y =msg->orientation.y;
    z =msg->orientation.z;
    w =msg->orientation.w;
    
    /* making a quaternion of position */
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
        // cout << "X: " << pose.pose.position.x << "\t" << "Y: " << pose.pose.position.y << "\t" << "Z: " << pose.pose.position.z << endl;
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

// MiniPID pid_x = MiniPID(.03, .0, .0, 0.1);
// MiniPID pid_y = MiniPID(.05, .0, .0);
// MiniPID pid_z = MiniPID(.128, .009, 0.05);
MiniPID pid_x = MiniPID(3.0, 0.2, 0.05, 0.15);
MiniPID pid_y = MiniPID(3.0, 0.2, 0.05, 0.1);
MiniPID pid_z = MiniPID(1.5, .012, 0.05, 0.1);

void signal_callback_handler(int signum)
{
    cout << "======================================="<< endl;
    outfile0.close();
    cout << "\nSaved File" << endl;
    cout << "EXIT programme " << endl;
    exit(signum);
}

int main(int argc, char **argv)
{
    int mode_controll;
    double output_x, output_y, output_z;

    outfile0.open("/home/nam97/Gazebo/Tool/gen_report/velocity.txt");
    outfile0 << "x " << "y " << "z " << "m " << "s" << endl;

    cout<< "______  __   __    ___     _____    _____ " << endl;
    cout<< "| ___ \\ \\ \\ / /   /   |   / ___ \\  | ___ \\" <<endl;
    cout<< "| |_/ /  \\ V /   / /| |   | | | |  | |_/ |" <<endl;
    cout<< "|  __/   /   \\  / /_| |   | | | |  |  __ /" <<endl;
    cout<< "| |     / /^\\ \\ \\___  |   | |_| |  | |_/ \\" <<endl;
    cout<< "\\_|     \\/   \\/     |_/   \\_____/  \\_____/\n\n" <<endl;

    cout << "-----Staring mode OFFBOARD CONTROL-----"<< endl;
    cout << "======================================="<< endl;
    cout << "| 1: Control follow local position    |"<< endl;
    cout << "| 2: Control follow PID               |"<< endl;
    cout << "======================================="<< endl;
    cout << " ENTER Option: ";

    cin >> mode_controll;
    /* Init position */
    switch(mode_controll)
    {
        case LOCAL:
            cout << "PLEASE ENTER POSITION INIT Follow Local ENU frame [x y z]: ";
            cin  >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z ;
            cout << "pose : "<< pose.pose.position.x << "  "<< pose.pose.position.y <<"  "<< pose.pose.position.z << endl;
            break;
        case PID:
            cout << "PID controller is chosen \nENTER Position Local ENU frame [x y z]: ";
            cin  >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z ;
            cout << "Time: " << endl;
            cout << "Axis x : " << pose.pose.position.x  << "m" << endl;
            cout << "Axis y : " << pose.pose.position.y << "m" << endl;
            cout << "Axis z : " << pose.pose.position.z << "m" << endl;
            break;
        default:
        {
            cout << "Not position" << endl;
            exit(0);
        }
    }

    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
            ("/mavros/imu/data",10,imuCallback);
    ros::Subscriber gps_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose",100,mavrosPoseCallback);
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
    ros::Publisher velocity_pub   = nh.advertise <geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 30 );
    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    pid_x.setOutputLimits(-0.5, 1.0);
    pid_y.setOutputLimits(-0.5, 1.0);
    pid_z.setOutputLimits(-1.0, 2.0);
    pid_x.setOutputRampRate(20);
    pid_y.setOutputRampRate(20);
    pid_z.setOutputRampRate(20);
    /* wait for FCU connection */
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    /* send a few setpoints before starting */
    cout << "LOADING";
    for(int i = 10; ros::ok() && i > 0; --i)
    {
        cout <<".";
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    cout<<"100%"<<endl;
    mavros_msgs::SetMode offb_set_mode, offset_mode;
    mavros_msgs::CommandBool arm_cmd;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offset_mode.request.custom_mode = "AUTO.LAND";
    arm_cmd.request.value = true;
    signal(SIGINT,signal_callback_handler);
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

        baygio = time(0);
        ltime = localtime(&baygio);
        outfile0 << PRECISION(vlocal_pose.pose.position.x) << " " << PRECISION(vlocal_pose.pose.position.y) << " " << PRECISION(vlocal_pose.pose.position.z) << " " << ltime->tm_min << " " << ltime->tm_sec << endl;

        output_x = pid_x.getOutput(PRECISION(vlocal_pose.pose.position.x), pose.pose.position.x);
        output_y = pid_y.getOutput(PRECISION(vlocal_pose.pose.position.y), pose.pose.position.y);
        output_z = pid_z.getOutput(PRECISION(vlocal_pose.pose.position.z), pose.pose.position.z);

        output_x = (int)(output_x*100);
        output_y = (int)(output_y*100);
        output_z = (int)(output_z*100);
        output_x = output_x/100;
        output_y = output_y/100;
        output_z = output_z/100;

        var_velocity.twist.linear.x = output_x;
        var_velocity.twist.linear.y = output_y;
        var_velocity.twist.linear.z = output_z;
        // cout << "V_X = "<< var_velocity.twist.linear.x << endl;
        // cout << "V_Y = "<< var_velocity.twist.linear.y << endl;
        // cout << "V_Z = "<< var_velocity.twist.linear.z << endl;
        // cout << "===================================================" << endl;
        switch(mode_controll)
        {
            case LOCAL:
                local_pos_pub.publish(pose);
                break;
            case PID:
                velocity_pub.publish(var_velocity);
                break;
            default:
                {
                    cout << "Not position" << endl;
                    exit(0);
                }
        }
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
