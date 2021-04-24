#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <mavros_msgs/CommandBool.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"

using namespace std;
using namespace Eigen;

ros::Publisher custom_activity_pub;

/*Variable*/
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped vlocal_pose;

time_t baygio = time(0);
tm *ltime       = localtime(&baygio);
ofstream outfile0, outfile1, outfile2;

static int LOCK                  = 10;
static int number_check          = 0;
static double var_offset_pose[3] = {0.0, 0.0, 0.0};
static char var_active_status[20];
static double x, y, z;
static bool LOCK_LAND            = false;
static bool vLand                = false;
static bool vend                 = false;

/*Declaring a 3*3 matrix*/
Matrix3f R, cv_rotation, cam2imu_rotation;
Vector3f positionbe, position_cam, positionaf, offset_marker;

void turn_off_motors(void);

/*storing gps data in pointer*/
void mavrosPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    vlocal_pose=*msg;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double x,y,z,w;
    Quaternionf quat;

    x = msg->orientation.x;
    y = msg->orientation.y;
    z = msg->orientation.z;
    w = msg->orientation.w;
    /*making a quaternion of position*/
    quat = Eigen::Quaternionf(w,x,y,z);
    /*making rotation matrix from quaternion*/
    R = quat.toRotationMatrix();
    tf2::Quaternion q;
    q.setValue(x, y, z, w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll  = roll*(180/3.14);
    pitch = pitch*(180/3.14);
    yaw   = yaw*(180/3.14);;
}

int vbegin = 2;
float minutes = 0;
float seconds = 0;

static void get_params_cb(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    if (LOCK_LAND == false)
    {
        if (vbegin == 1)
        {
            baygio = time(0);
            ltime = localtime(&baygio);
            cout << "Start:" << ltime->tm_min << ":";
            cout << ltime->tm_sec << endl;
            minutes = ltime->tm_min;
            seconds = ltime->tm_sec;
            vbegin = 0;
        }

        double xq,yq,zq,wq;
        Quaternionf quat;
        cam2imu_rotation << -0.0001 , -1 , 0 , -1 , 0 , 0 ,-0.0001 , 0 , -1;

        position_cam[0] = (msg->transforms[0].transform.translation.x);
        position_cam[1] = (msg->transforms[0].transform.translation.y);
        position_cam[2] = (msg->transforms[0].transform.translation.z);

        xq = msg->transforms[0].transform.rotation.x;
        yq = msg->transforms[0].transform.rotation.y;
        zq = msg->transforms[0].transform.rotation.z;
        wq = msg->transforms[0].transform.rotation.w;
        /* Aruco ----> Drone */
        positionbe = cam2imu_rotation*position_cam;
        /* Drone ----> NEU */
        positionaf = R*positionbe;
        /* update the position */
        if (LOCK > 0)
        {
            x = positionaf[0]+vlocal_pose.pose.position.x;
            y = positionaf[1]+vlocal_pose.pose.position.y;
            z = positionaf[2]+vlocal_pose.pose.position.z;
            /* Convert float round 2 */
            x = (int)(x*100);
            y = (int)(y*100);
            z = (int)(z*100);
            x = x/100;
            y = y/100;
            z = z/100;
        }

        if (abs(positionaf[0]) < 0.3 && abs(positionaf[1]) < 0.3)
        {
            /* maintain a llatitude of 2 m in z axis */
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            if (0.6 < vlocal_pose.pose.position.z)
            {
                if (0.5 >= pose.pose.position.z)
                {
                    pose.pose.position.z = 0.5;
                }
                else
                {
                    pose.pose.position.z = vlocal_pose.pose.position.z -2;
                }
            }
            else
            {
                vLand = true;
            }
        }
        else
        {
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = vlocal_pose.pose.position.z;
            ROS_INFO("Aligning........!");
            LOCK = 0;
        }
        number_check ++;
        if(number_check == 70)
        {
            LOCK = 1;
            number_check = 0;
        }
        baygio = time(0);
        ltime = localtime(&baygio);
        outfile0 << vlocal_pose.pose.position.x << "\t" << vlocal_pose.pose.position.y << "\t" << vlocal_pose.pose.position.z << '\t' << ltime->tm_min << " : " << ltime->tm_sec << endl;
        outfile1 << positionbe[0] <<'\t'<< positionbe[1] << '\t' << positionbe[2] << '\t' << ltime->tm_min << " : " << ltime->tm_sec << endl;
        outfile2 << x <<'\t'<< y << '\t' << z << '\t'<< ltime->tm_min << " : " << ltime->tm_sec << endl;

        cout<<"Aruco2Cam  : " << position_cam[0] <<'\t'<< position_cam[1] << '\t' << position_cam[2] << endl;
        cout<<"Aruco2Drone: " << positionbe[0] <<'\t'<< positionbe[1] << '\t' << positionbe[2] << endl;
        cout<<"Aruco2NEU  : " << x <<'\t'<< y << '\t' << z << endl;
        cout<<"Drone      : " << vlocal_pose.pose.position.x << " : " << vlocal_pose.pose.position.y << " : " << vlocal_pose.pose.position.z << endl;
    }
}

void local_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose.pose.position.x= msg->pose.position.x;
    pose.pose.position.y= msg->pose.position.y;
    pose.pose.position.z= msg->pose.position.z;
}

void turn_off_motors(void)
{
    std_msgs::String msg;
    std::stringstream ss;

    ss << "LAND";
    msg.data = ss.str();
    custom_activity_pub.publish(msg);
}

int main(int argc, char **argv)
{

    int sizeof_queue     = 10;
    // pose.pose.position.x = 2;
    // pose.pose.position.y = 3;
    // pose.pose.position.z = 7;

    /*in cac thanh phan cua cau truc tm struct.*/
    cout << "Nam: "<< 1900 + ltime->tm_year << endl;
    cout << "Thang: "<< 1 + ltime->tm_mon<< endl;
    cout << "Ngay: "<<  ltime->tm_mday << endl;
    cout << "Thoi gian: "<< ltime->tm_hour << ":";
    cout << ltime->tm_min << ":";
    cout << ltime->tm_sec << endl;
    outfile0.open("/home/nam97/data_file/gps.txt");
    outfile1.open("/home/nam97/data_file/Aruco2Drone.txt");
    outfile2.open("/home/nam97/data_file/Aruco2NEU.txt");

    ros::init(argc, argv, "subpose_node");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe
                            ("/mavros/imu/data",10,imuCallback);
    ros::Subscriber gps_sub = n.subscribe
                            ("/mavros/local_position/pose",100,mavrosPoseCallback);
    ros::Subscriber pose_sub = n.subscribe
                            ("/tf_list", 1000, get_params_cb);
    ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>
                            ("mavros/setpoint_position/local", 10, local_pose_callback);
    ros::Publisher local_pos_pub1 = n.advertise<geometry_msgs::PoseStamped>
                            ("cmd/set_pose/position1", 30);
    custom_activity_pub = n.advertise<std_msgs::String>
                            ("cmd/set_activity/type",10);
    ros::Rate rate(20.0);

    for(int i = 100; ros::ok() && i > 0; --i)
    {
        LOCK = 1;
        ros::spinOnce();
        rate.sleep();
    }
    vbegin = 1;
    while(ros::ok())
    {
        if (vLand == true)
        {
            LOCK_LAND = true;
            vLand     = false;
            vend      = true;
            baygio    = time(0);
            ltime     = localtime(&baygio);
            turn_off_motors();
            ROS_INFO("AUTO LANDING MODE is request");
            cout << "\n\t========================================"<< endl;
            cout << "\t|-----------Time AUTO LANDING-----------"<< endl;
            cout << "\t========================================"<< endl;
            cout << "\t| Start :" << minutes <<" : "<< seconds << endl;
            cout << "\t| END   :" << ltime->tm_min <<" : "<< ltime->tm_sec << endl;
            cout << "\t| Total :" << ltime->tm_min - minutes <<" minute "<< ltime->tm_sec - seconds << " Second" << endl;
            cout << "\t========================================"<< endl;
            cout<<"Drone : x = " << vlocal_pose.pose.position.x << " y = " << vlocal_pose.pose.position.y << " z = " << vlocal_pose.pose.position.z << endl;
            outfile0.close();
            outfile1.close();
            outfile2.close();
        }
        if (vend == false)
        {
            local_pos_pub1.publish(pose);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
