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
#include "std_msgs/Int16.h"
#include <csignal>


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
ros::Publisher custom_activity_pub;

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

/**

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

void sig_handler( int sig )
{
    cout << "\nInterrupt signal (" << sig << ") received.\n";
    std_msgs::String msgs;
    std::stringstream ss;
    ss << "LAND";
    msgs.data = ss.str();
    cout << "Give mode AUTO.LAND" << endl;
    custom_activity_pub.publish(msgs);

    exit(sig);
}

void mavrosPose_Callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    vlocal_pose=*msg;
}


/**
* @b
*
*
*
*
 */

int main(int argc, char **argv)
{
    uint8_t xstep = 0;
    std_msgs::Int16 mode;
    
    signal(SIGTSTP, sig_handler);
    ros::init(argc, argv, "test_node");
    ros::NodeHandle test;

    ros::Subscriber local_position_sub = test.subscribe<geometry_msgs::PoseStamped>
        ("/mavros/local_position/pose",10,mavrosPose_Callback);
    ros::Publisher local_position_pub = test.advertise<geometry_msgs::PoseStamped>
        ("cmd/set_pose/position1", 30);
    custom_activity_pub = test.advertise<std_msgs::String>
        ("cmd/set_activity/type",10);
    ros::Publisher both_mode_pub = test.advertise<std_msgs::Int16>
        ("cmd/set_mode/mode",10);
    ros::Publisher tf_list_pub_  = test.advertise<tf2_msgs::TFMessage>
        ("/tf_list", 100);
    ros::Rate rate(20.0);
    ros::spinOnce();
/* GUI */
    cout <<"\n==============\U0001F4E1 Test controller =============="<< endl;
    cout << "\U0001F449 Control with local position"<< endl;

/**
 * @brief only use in case select control both pid and pose in off board 
 *        if mode.data =2 , the controller follow PID . otherwise Pose
 * @
 */
#ifdef PID
    cout << "PID Controller"<< endl;
    mode.data = 2;
#else
    cout << "\x1B[93mLOCAL Controller\033[0m"<< endl;
    mode.data = 1;
#endif
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
            both_mode_pub.publish(mode);
            local_position_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        sleep(3);
        system("echo \"\\r\u2714 1: Sent pose X=0 | Y=0 | Z=5 !!!    \"");
        TEST(xstep);
        semaphore_give(semaphore_b);
    }
#ifdef PID
    cout << "PID Controller"<< endl;
    mode.data = 2;
#else
    cout << "\x1B[93mLOCAL Controller\033[0m"<< endl;
    mode.data = 2;
#endif
    system("echo -n \"2: Sending pose X=3 | Y=0 | Z=5 ...\"");
    if (true == semaphore_take(semaphore_b))
    {
        both_mode_pub.publish(mode);
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
        sleep(3);
        system("echo \"\\r\u2714 2: Sent pose X=3 | Y=0 | Z=5 !!!    \"");
        TEST(xstep);
        semaphore_give(semaphore_b);
    }

#ifdef PID
    cout << "PID Controller"<< endl;
    mode.data = 2;
#else
    cout << "\x1B[93mLOCAL Controller\033[0m"<< endl;
    mode.data = 2;
#endif
    system("echo -n \"3: Sending pose X=3 | Y=3 | Z=5 ...\"");
    if (true == semaphore_take(semaphore_b))
    {
        both_mode_pub.publish(mode);
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
        sleep(3);
        system("echo \"\\r\u2714 3: Sent pose X=3 | Y=3 | Z=5 !!!    \"");
        TEST(xstep);
        semaphore_give(semaphore_b);
    }
#ifdef PID
    cout << "PID Controller"<< endl;
    mode.data = 2;
#else
    cout << "\x1B[93mLOCAL Controller\033[0m"<< endl;
    mode.data = 1;
#endif
    system("echo -n \"4: Sending pose X=0 | Y=0 | Z=2.5 ...\"");
    if (true == semaphore_take(semaphore_b))
    {
        both_mode_pub.publish(mode);
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
        sleep(7);
        system("echo \"\\r\u2714 4: Sent pose X=0 | Y=0 | Z=2.5 !!!    \"");
        TEST(xstep);
        semaphore_give(semaphore_b);
    }
// #else
//     cout << "\x1B[34mSkiped\033[0m" << endl;
// #endif /* LOCAL */


    cout << "\U0001F449 Control with PID"<< endl;
#ifdef PID
    /* Send Command */


#else
    cout << "\x1B[34mSkiped\033[0m" << endl;
#endif /* PID */

    /*landing with pub fixed position */
    cout << "\n\U0001F449 Test Landing"<< endl;
    mode.data = 1;
    system("echo -n \"4: Sending pose X=0 | Y=0 | Z=5 ...\"");
    if (true == semaphore_take(semaphore_b))
    {
        both_mode_pub.publish(mode);
        pose.pose.position.x= 0;
        pose.pose.position.y= 0;
        pose.pose.position.z= 8;
        while(abs(0 - vlocal_pose.pose.position.x) > OFFSET || \
        abs(0 - vlocal_pose.pose.position.y) > OFFSET || \
        abs(5 - vlocal_pose.pose.position.z) > OFFSET )
        {
            local_position_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        sleep(2);
        system("echo \"\\r\u2714 4: Sent pose X=0 | Y=0 | Z=5 !!!    \"");
        semaphore_give(semaphore_b);
    }
    cout << "\x1B[31mPlease open new terminal and run rosrun calculationps node\033[0m" << endl;
    cout << "\x1B[31mand then Press Enter to Continue\033[0m" << endl;
    cin.ignore();


    /*Publish TFs for each of the markers*/
    // static tf2_ros::TransformBroadcaster br;
    // auto stamp = ros::Time::now();

    // /*Create and publish tf message for each marker*/
    // tf2_msgs::TFMessage tf_msg_list;

    // geometry_msgs::TransformStamped tf_msg;
    // stringstream ss;


    // ss << "marker_id7";
    // tf_msg.child_frame_id          = ss.str();
    // tf_msg.transform.translation.x = 2;
    // tf_msg.transform.translation.y = 2;
    // while(abs(4 - vlocal_pose.pose.position.z) > OFFSET)
    // {
    //     tf_msg.header.stamp            = stamp;
    //     tf_msg.transform.translation.z = vlocal_pose.pose.position.z - 0.1;

    //     tf_msg_list.transforms.push_back(tf_msg);
    //     br.sendTransform(tf_msg);
    //     tf_list_pub_.publish(tf_msg_list);
    // }



    // system("echo -n \"Landing ...\"");
    // std_msgs::String msgs;
    // std::stringstream ss;
    // ss << "LAND";
    // msgs.data = ss.str();
    // custom_activity_pub.publish(msgs);
    // while(current_state.armed == true);
    // system("echo \"\\r\u2714 Land !!!    \"");

    cout <<"\x1B[36m-------------------------------------------------------\033[0m"<< endl;
    cout <<"\x1B[34mCompleted\033[0m"<<endl;
    cout <<"\x1B[36m-------------------------------------------------------\033[0m"<< endl;


    ros::spinOnce();
    return 0;
}