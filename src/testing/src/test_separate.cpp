#include "ros/ros.h"
#include <math.h>
#include <iostream>
#include <cstdint>
#include <stdlib.h>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <eigen3/Eigen/Core>
#include <mavros_msgs/CommandBool.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "Kalmanfiler.h"

/*============================================================================== 

 *                               Definitions

 =============================================================================*/ 
#define OFFSET      (0.1)
#define TEST(a)     (a = a+1)

/*============================================================================== 

 *                                Namespace

 =============================================================================*/ 
using namespace std;
using namespace Eigen;

/*============================================================================== 

 *                                  Topic

 =============================================================================*/ 


/*============================================================================== 

 *                                 Variables 

 =============================================================================*/ 
geometry_msgs::PoseStamped vlocal_pose;
geometry_msgs::PoseStamped pose;
mavros_msgs::State current_state;
bool semaphore_b = true;
time_t baygio = time(0);
tm *ltime = localtime(&baygio);
/*============================================================================== 

 *                                  Object

 =============================================================================*/ 


/*============================================================================== 

 *                                  Code 

 =============================================================================*/ 

bool semaphore_give(bool &sem)
{
    if(sem == false)
    {
        sem = true;
    }
    else
    {
        return false;
    }

    return true;
}

/*

 */

bool semaphore_take(bool &sem)
{

    ros::Time last_request = ros::Time::now();
    loop:
    if(sem == true)
    {
        sem = false;
    }
    else
    {
        if (ros::Time::now() - last_request > ros::Duration(10.0))
        {
            cout << "Time out \U0001F602" << endl;
            return false;
        }
        goto loop;
    }

    return true;
}

void mavrosPose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    vlocal_pose=*msg;
}

/*
* @b
*
*
*
*
 */

int main(int argc, char **argv)
{
    uint8_t xstep = 0;
    ros::init(argc, argv, "test_node");
    ros::NodeHandle test;

    ros::Subscriber local_position_sub = test.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose",10,mavrosPose_Callback);
    ros::Publisher local_position_pub = test.advertise<geometry_msgs::PoseStamped>
        ("cmd/set_pose/position1", 30);
    ros::Publisher custom_activity_pub = test.advertise<std_msgs::String>
        ("cmd/set_activity/type",10);
    ros::Rate rate(20.0);
    ros::spinOnce();
/* GUI */
    cout <<"\n==============\U0001F4E1 Test controller =============="<< endl;
    cout << "\U0001F449 Control with local position"<< endl;
#ifdef LOCAL
    /* Send Command */
    system("echo -n \"1: Sending pose X=0 | Y=0 | Z=5 ...\"");
    if (true == semaphore_take(semaphore_b))
    {
        pose.pose.position.x= 0;
        pose.pose.position.y= 0;
        pose.pose.position.z= 5;
        while(abs(0 - vlocal_pose.pose.position.x) > OFFSET || \
        abs(0 - vlocal_pose.pose.position.y) > OFFSET || \
        abs(5 - vlocal_pose.pose.position.z) > OFFSET )
        {
            local_position_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        sleep(1);
        system("echo \"\\r\u2714 1: Sent pose X=0 | Y=0 | Z=5 !!!    \"");
        TEST(xstep);
        semaphore_give(semaphore_b);
    }

    system("echo -n \"2: Sending pose X=3 | Y=0 | Z=5 ...\"");
    if (true == semaphore_take(semaphore_b))
    {
        pose.pose.position.x= 3;
        pose.pose.position.y= 0;
        pose.pose.position.z= 5;
        while(abs(3 - vlocal_pose.pose.position.x) > OFFSET || \
        abs(0 - vlocal_pose.pose.position.y) > OFFSET || \
        abs(5 - vlocal_pose.pose.position.z) > OFFSET )
        {
            local_position_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        sleep(1);
        system("echo \"\\r\u2714 2: Sent pose X=3 | Y=0 | Z=5 !!!    \"");
        TEST(xstep);
        semaphore_give(semaphore_b);
    }

    system("echo -n \"3: Sending pose X=3 | Y=3 | Z=5 ...\"");
    if (true == semaphore_take(semaphore_b))
    {
        pose.pose.position.x= 3;
        pose.pose.position.y= 3;
        pose.pose.position.z= 5;
        while(abs(3 - vlocal_pose.pose.position.x) > OFFSET || \
        abs(3 - vlocal_pose.pose.position.y) > OFFSET || \
        abs(5 - vlocal_pose.pose.position.z) > OFFSET )
        {
            local_position_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        sleep(1);
        system("echo \"\\r\u2714 3: Sent pose X=3 | Y=3 | Z=5 !!!    \"");
        TEST(xstep);
        semaphore_give(semaphore_b);
    }

    system("echo -n \"4: Sending pose X=0 | Y=0 | Z=2.5 ...\"");
    if (true == semaphore_take(semaphore_b))
    {
        pose.pose.position.x= 0;
        pose.pose.position.y= 0;
        pose.pose.position.z= 2.5;
        while(abs(0 - vlocal_pose.pose.position.x) > OFFSET || \
        abs(0 - vlocal_pose.pose.position.y) > OFFSET || \
        abs(2.5 - vlocal_pose.pose.position.z) > OFFSET )
        {
            local_position_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        sleep(1);
        system("echo \"\\r\u2714 4: Sent pose X=0 | Y=0 | Z=2.5 !!!    \"");
        TEST(xstep);
        semaphore_give(semaphore_b);
    }
#else
    cout << "\x1B[34mSkiped\033[0m" << endl;
#endif /* LOCAL */


    cout << "\U0001F449 Control with PID"<< endl;
#ifdef PID
    /* Send Command */
#else
    cout << "\x1B[34mSkiped\033[0m" << endl;
#endif /* PID */


    system("echo -n \"Landing ...\"");
    std_msgs::String msg;
    std::stringstream ss;
    ss << "LAND";
    msg.data = ss.str();
    custom_activity_pub.publish(msg);
    while(current_state.armed == true);
    system("echo \"\\r\u2714 Land !!!    \"");
    cout <<"\x1B[36m-------------------------------------------------------\033[0m"<< endl;
    cout << "\x1B[34mCompleted\033[0m"<<endl;
    cout <<"\x1B[36m-------------------------------------------------------\033[0m"<< endl;
    cout << "Total: 4" << endl;
    cout << "\x1B[93mPass\033[0m: " << unsigned(xstep) << "\t\t" << "\x1B[31mFailed\033[0m: "<< 4 - xstep << endl;


    ros::spinOnce();
    return 0;
}