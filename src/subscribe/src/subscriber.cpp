#include "ros/ros.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <sstream>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <mavros_msgs/CommandBool.h>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Imu.h>
#include "std_msgs/Float32.h"


#define OFFSET_X           (-0.5)
#define OFFSET_Y           0
#define OFFSET_Z           0
#define OFFSET_RESET       0

using namespace std;
using namespace Eigen;

ros::Publisher custom_activity_pub;

/*variable*/
geometry_msgs::PoseStamped pose;
geometry_msgs::PoseStamped var_gps_pose;
static int LOCK                  = 10;
static int number_check          = 0;
static double var_offset_pose[3] = {0.0, 0.0, 0.0};
static char var_active_status[20];
static double x, y, z;
static bool LOCK_LAND = true;

time_t baygio = time(0);
tm *ltm = localtime(&baygio);
ofstream outfile0, outfile1, outfile2;

/*declaring a 3*3 matrix*/
Matrix3f R, cv_rotation, cam2imu_rotation;
Vector3f positionbe, position_cam, positionaf, offset_marker;

void turn_off_motors(void);

/*storing gps data in pointer*/
void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    var_gps_pose=*msg;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{

    double x,y,z,w;

    x=msg->orientation.x;
    y=msg->orientation.y;
    z=msg->orientation.z;
    w=msg->orientation.w;
    
    /*making a quaternion of position*/
    Quaternionf quat;
    quat = Eigen::Quaternionf(w,x,y,z);
  
    /*making rotation matrix from quaternion*/
    R = quat.toRotationMatrix();
    // auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
    // cout << "Euler from quaternion in roll, pitch, yaw"<< endl << euler << endl;
    tf2::Quaternion q;
    q.setValue(x, y, z, w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll  = roll*(180/3.14);
    pitch = pitch*(180/3.14);
    yaw   = yaw*(180/3.14);
    // cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << endl;
    // cout << "R=" << endl << R << endl;
}

// Eigen::Matrix3d rotation_from_euler(double roll, double pitch, double yaw){
//     // roll and pitch and yaw in radians
//     double su = sin(roll);
//     double cu = cos(roll);
//     double sv = sin(pitch);
//     double cv = cos(pitch);
//     double sw = sin(yaw);
//     double cw = cos(yaw);
//     Eigen::Matrix3d Rot_matrix(3, 3);
//     Rot_matrix(0, 0) = cv*cw;
//     Rot_matrix(0, 1) = su*sv*cw - cu*sw;
//     Rot_matrix(0, 2) = su*sw + cu*sv*cw;
//     Rot_matrix(1, 0) = cv*sw;
//     Rot_matrix(1, 1) = cu*cw + su*sv*sw;
//     Rot_matrix(1, 2) = cu*sv*sw - su*cw;
//     Rot_matrix(2, 0) = -sv;
//     Rot_matrix(2, 1) = su*cv;
//     Rot_matrix(2, 2) = cu*cv;
//     return Rot_matrix;
// }

static void get_params_cb(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    double xq,yq,zq,wq;
    cam2imu_rotation << -0.0001 , -1 , 0 , -1 , 0 , 0 ,-0.0001 , 0 , -1;
    offset_marker[0] = -0.5;
    offset_marker[1] = 0;
    offset_marker[2] = 0;
    Quaternionf quat;

    position_cam[0] = (msg->transforms[0].transform.translation.x);
    position_cam[1] = (msg->transforms[0].transform.translation.y);
    position_cam[2] = (msg->transforms[0].transform.translation.z);

    xq = msg->transforms[0].transform.rotation.x;
    yq = msg->transforms[0].transform.rotation.y;
    zq = msg->transforms[0].transform.rotation.z;
    wq = msg->transforms[0].transform.rotation.w;
    /* making a quaternion of position */
    /* set position to marker size largest */
    quat=Eigen::Quaternionf(wq,xq,yq,zq);
    cv_rotation = quat.toRotationMatrix();
    offset_marker = cv_rotation*offset_marker;
    if ( 3 >= var_gps_pose.pose.position.z)
    {
        position_cam = position_cam + offset_marker;
    }
    /**/
    /* Aruco ----> Drone */
    positionbe = cam2imu_rotation*position_cam;
    /* Drone ----> NEU */
    positionaf = R*positionbe;
    /* update the position */
    if (LOCK > 0)
    {
        x = positionaf[0]+var_gps_pose.pose.position.x;
        y = positionaf[1]+var_gps_pose.pose.position.y;
        z = positionaf[2]+var_gps_pose.pose.position.z;
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
        if (0.5 <= var_gps_pose.pose.position.z)
        {
            pose.pose.position.z = var_gps_pose.pose.position.z -2;
        }
    }
    else
    {
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = var_gps_pose.pose.position.z;
        ROS_INFO("Aligning........!");
        LOCK = 0;
    }
    number_check ++;
    if (number_check == 70)
    {
        LOCK = 1;
        number_check = 0;
    }
    baygio = time(0);
    ltm = localtime(&baygio);
    outfile0 << var_gps_pose.pose.position.x << "\t" << var_gps_pose.pose.position.y << "\t" << var_gps_pose.pose.position.z << '\t' << ltm->tm_min << " : " << ltm->tm_sec << endl;
    outfile1 << positionbe[0] <<'\t'<< positionbe[1] << '\t' << positionbe[2] << '\t' << ltm->tm_min << " : " << ltm->tm_sec << endl;
    outfile2 << x <<'\t'<< y << '\t' << z << '\t'<< ltm->tm_min << " : " << ltm->tm_sec << endl;

    cout<<"Aruco2Cam  : " << position_cam[0] <<'\t'<< position_cam[1] << '\t' << position_cam[2] << endl;
    cout<<"Aruco2Drone: " << positionbe[0] <<'\t'<< positionbe[1] << '\t' << positionbe[2] << endl;
    cout<<"Aruco2NEU  : " << x <<'\t'<< y << '\t' << z << endl;
    cout<<"Drone      : " << var_gps_pose.pose.position.x << " : " << var_gps_pose.pose.position.y << " : " << var_gps_pose.pose.position.z << endl;
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
    pose.pose.position.x = 2;
    pose.pose.position.y = 3;
    pose.pose.position.z = 7;
    // in cac thanh phan cua cau truc tm struct.
    cout << "Nam: "<< 1900 + ltm->tm_year << endl;
    cout << "Thang: "<< 1 + ltm->tm_mon<< endl;
    cout << "Ngay: "<<  ltm->tm_mday << endl;
    cout << "Thoi gian: "<< ltm->tm_hour << ":";
    cout << ltm->tm_min << ":";
    cout << ltm->tm_sec << endl;
    outfile0.open("/home/nam97/data_file/gps.txt");
    outfile1.open("/home/nam97/data_file/Aruco2Drone.txt");
    outfile2.open("/home/nam97/data_file/Aruco2NEU.txt");

    ros::init(argc, argv, "subpose_node");
    ros::NodeHandle n;

    ros::Subscriber imu_sub = n.subscribe("/mavros/imu/data",10,imuCallback);
    ros::Subscriber gps_sub = n.subscribe("/mavros/local_position/pose",100,gpsCallback);
    ros::Subscriber pose_sub = n.subscribe("/tf_list", 1000, get_params_cb);
    custom_activity_pub = n.advertise<std_msgs::String>("cmd/set_activity/type",10);
    ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10, local_pose_callback);
    ros::Publisher local_pos_pub1 = n.advertise<geometry_msgs::PoseStamped>("cmd/set_pose/position1", 30);

    ros::Rate rate(20.0);
    for(int i = 100; ros::ok() && i > 0; --i)
    {

        LOCK = 1;
        ros::spinOnce();
        rate.sleep();
    }
    while(ros::ok())
    {
        if (0.5 > var_gps_pose.pose.position.z && LOCK_LAND == true)
        {
            turn_off_motors();
            ROS_INFO("AUTO LANDING MODE is request");
            LOCK_LAND = false;
            outfile0.close();
            outfile1.close();
            outfile2.close();
        }

        local_pos_pub1.publish(pose);
        ros::spinOnce();
        // ros::spin();
        rate.sleep();
    }

    return 0;
}
