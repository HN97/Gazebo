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
#include "std_msgs/Int16.h"
#include "MiniPID.h"
#include <stdio.h>
#include <stdlib.h>
#include <mavros_msgs/PositionTarget.h>

/******************************************************************************* 

 *                               Definitions 

 ******************************************************************************/ 
#define LOCAL    1
#define PID      2
#define BOTH     3
#define PRECISION(x)    round(x * 100) / 100
#define CHECK_M         (CHECK_MARK="\033[0;32m\xE2\x9C\x94\033[0m")

/******************************************************************************* 

 *                                Namespace

 ******************************************************************************/ 
using namespace std;
using namespace Eigen;

/******************************************************************************* 

 *                                 Variables 

 ******************************************************************************/
time_t baygio          = time(0);
tm *ltime              = localtime(&baygio);
static int STATE_CHECK = 1;
ofstream outfile0;
char path[250];
static char var_active_status[20];
int16_t mode_select = LOCAL;
Matrix3f R;
Vector3f var_offset_pose;
Vector3f positionbe;
Vector3f positionaf;
mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped vlocal_pose;
geometry_msgs::TwistStamped var_velocity;

// MiniPID pid_x = MiniPID(3.0, 0.2, 0.0, 0.15);
// MiniPID pid_y = MiniPID(3.0, 0.2, 0.0, 0.1);
// MiniPID pid_z = MiniPID(1.5, .012, 0.0, 0.1);
MiniPID pid_x = MiniPID(0.4, 0.01, 0.0, 0.1);
MiniPID pid_y = MiniPID(0.4, 0.01, 0.0, 0.1);
MiniPID pid_z = MiniPID(0.4, 0.01, 0.0, 0.1);
/******************************************************************************* 

 *                                  Code 

 ******************************************************************************/ 
/* getting the state into a pointer */
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

/* storing gps data in pointer */
void mavrosPose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
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
        // ROS_INFO("Control Follow Local ENU frame");
        pose.pose.position.x= msg->pose.position.x;
        pose.pose.position.y= msg->pose.position.y;
        pose.pose.position.z= msg->pose.position.z;
        // cout << "X: " << pose.pose.position.x << "\t" << "Y: " << pose.pose.position.y << "\t" << "Z: " << pose.pose.position.z << endl;
    // }
}

// void set_target_yaw_callback(const std_msgs::Float32::ConstPtr& msg)
// {
//     float angle_yaw = msg.data;
// }

void both_mode_callback(const std_msgs::Int16::ConstPtr& msg)
{
    mode_select = msg->data;
}

void custom_activity_callback(const std_msgs::String::ConstPtr& msg)
{
    strcpy(var_active_status, msg->data.c_str());
}

/**
 * @brief
 * 
 * @param 
 *
 * @return 
 */
void signal_callback_handler(int signum)
{
    cout << "\n======================================="<< endl;
    outfile0.close();
    cout << "\nSaved File" << endl;
    cout << path << endl;
    cout << "EXIT programme " << endl;
    exit(signum);
}

void init_pose()
{
    cin  >> pose.pose.position.x >> pose.pose.position.y >> pose.pose.position.z ;
    cout << "\x1B[93mAxis x\033[0m : " << pose.pose.position.x << "m" << endl;
    cout << "\x1B[93mAxis y\033[0m : " << pose.pose.position.y << "m" << endl;
    cout << "\x1B[93mAxis z\033[0m : " << pose.pose.position.z << "m" << endl;
}

int main(int argc, char **argv)
{
    int mode_controll;
    char path_script[250];
    double output_x, output_y, output_z;
    getcwd(path, sizeof(path));
    strcpy(path_script, path);
    strcat(path, "/Tool/gen_report/velocity.txt");
    strcat(path_script, "/scripts/gui_script.sh");
    outfile0.open(path);
    outfile0 << "x " << "y " << "z " << "m " << "s" << endl;

    cout<< "______  __   __    ___     _____    _____ " << endl;
    cout<< "| ___ \\ \\ \\ / /   /   |   / ___ \\  | ___ \\" <<endl;
    cout<< "| |_/ /  \\ V /   / /| |   | | | |  | |_/ |" <<endl;
    cout<< "|  __/   /   \\  / /_| |   | | | |  |  __ /" <<endl;
    cout<< "| |     / /^\\ \\ \\___  |   | |_| |  | |_/ \\" <<endl;
    cout<< "\\_|     \\/   \\/     |_/   \\_____/  \\_____/\n\n" <<endl;
    cout << "-----Staring mode OFFBOARD CONTROL-----"<< endl;
    cout << "======================================="<< endl;
    cout << "\U0001F449 1: Control follow local position    |"<< endl;
    cout << "\U0001F449 2: Control follow PID               |"<< endl;
    cout << "\U0001F449 3: Control follow Pose & PID        |"<< endl;
    cout << "======================================="<< endl;
    cout << " \x1B[93m\u262D ENTER Option:\033[0m ";

    cin >> mode_controll;
#ifdef HITL
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 0;

#else
    /* Init position */
    switch(mode_controll)
    {
        case LOCAL:
            cout << "PLEASE ENTER POSITION INIT Follow Local ENU frame [x y z]: ";
            init_pose();
            break;
        case PID:
            cout << "PID controller is chosen \nENTER Position Local ENU frame [x y z]: ";
            init_pose();
        case BOTH:
            cout << "PID controller is chosen \nENTER Position Local ENU frame [x y z]: ";
            init_pose();
            break;
        default:
        {
            cout << "Not position" << endl;
            exit(0);
        }
    }
#endif /* HITL */
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("/mavros/imu/data",10,imuCallback);
    ros::Subscriber local_position_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose",100,mavrosPose_Callback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
    ros::Publisher velocity_pub   = nh.advertise <geometry_msgs::TwistStamped>
        ("/mavros/setpoint_velocity/cmd_vel", 30 );

    ros::Subscriber position_target_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("cmd/set_pose/position1",30,set_target_position_callback);
    // ros::Subscriber yaw_target_sub = nh.subscribe<std_msgs::Float32>
        // ("cmd/set_pose/orientation",10,set_target_yaw_callback);
    ros::Subscriber both_mode_sub = nh.subscribe<std_msgs::Int16>
        ("cmd/set_mode/mode",10,both_mode_callback);
    ros::Subscriber custom_activity_sub = nh.subscribe<std_msgs::String>
        ("cmd/set_activity/type",10,custom_activity_callback);
    // ros::Subscriber pose_sub = nh.subscribe
        // ("/tf_list", 10, get_params_cb);

    ros::Rate rate(20.0);
    if(mode_controll == PID || mode_controll == BOTH)
    {
        pid_x.setOutputLimits(-0.5, 5.0);
        pid_y.setOutputLimits(-0.5, 5.0);
        pid_z.setOutputLimits(-1.0, 1.0);
        pid_x.setOutputRampRate(10);
        pid_y.setOutputRampRate(10);
        pid_z.setOutputRampRate(10);
    }
    /* wait for FCU connection */
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    /* send a few setpoints before starting */
    cout << "Stable param\n" << endl;
    for(int i = 10; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        // rate.sleep();
    }
    // system("./scripts/gui_script.sh 0 0 \'init\'");
    // system("./scripts/gui_script.sh 30 0.75 \'configuration files\'");
    // system("./scripts/gui_script.sh 90 0.75 \'update registery\'");
    // system("./scripts/gui_script.sh 100 2 \'done\'");
    // system("printf \"\n\"");

    // system("./scripts/gui_script.sh 0 0 \'init\'");
    // system("./scripts/gui_script.sh 30 1 \'configuration files\'");
    // system("./scripts/gui_script.sh 60 1 \'download file\'");
    // system("./scripts/gui_script.sh 100 1 \'done\'");;
    // cout << "\n\u2705 Completed."<<endl;

    mavros_msgs::SetMode offb_set_mode, offset_mode;
    mavros_msgs::CommandBool arm_cmd;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    offset_mode.request.custom_mode = "AUTO.LAND";
    arm_cmd.request.value = true;
    signal(SIGINT,signal_callback_handler);
    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
#ifdef HITL
        system("echo -n \"\e[4mWaiting for activation mode\e[0m\n\"");
        system("echo -n \"OFFBOARD mode...\"");
        while(current_state.mode != "OFFBOARD");
        system("echo -e \"\\r\033[0;32m\xE2\x9C\x94\033[0m OFFBOARD ready!!!\"");
        system("echo -n \"OFFBOARD mode...\"");
        while(!current_state.armed);
        system("echo -e \"\\r\033[0;32m\xE2\x9C\x94\033[0m OFFBOARD ready!!!\"");
#else
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
#endif /* HITL */
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
        if(mode_controll == PID || mode_controll == BOTH)
        {
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
        }

        switch(mode_controll)
        {
            case LOCAL:
                local_pos_pub.publish(pose);
                break;
            case PID:
                velocity_pub.publish(var_velocity);
                break;
            case BOTH:
                if ( mode_select == LOCAL)
                {
                    local_pos_pub.publish(pose);
                }
                else if ( mode_select == PID)
                {
                    velocity_pub.publish(var_velocity);
                }
                else
                {

                }
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
