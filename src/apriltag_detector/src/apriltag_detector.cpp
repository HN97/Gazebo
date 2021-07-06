#include <csignal>
#include <iostream>
#include <vector>   /* used in hashmap */
#include <numeric>  /* used for summing a vector */
#include "ros/ros.h"
// #include "sensor_msgs/Image.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"
/* ROS transform */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_msgs/TFMessage.h>
/* ROS CvBridge */
#include "cv_bridge/cv_bridge.h"
/* Image Transport to publish output img */
#include <image_transport/image_transport.h>
/* OpenCV */
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/imgproc.hpp>
#include <stdint.h>
#include "opencv2/opencv.hpp"

extern "C" {
#include "apriltag.h"
#include "tag36h11.h"
#include "tag25h9.h"
#include "tag16h5.h"
#include "tagCircle21h7.h"
#include "tagCircle49h12.h"
#include "tagCustom48h12.h"
#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "common/getopt.h"
#include "apriltag_pose.h"
}

#define tagSIZE    0.355
#define FX         1179.598752
#define FY         1177.000389
#define CX         928.099247
#define CY         558.635461

apriltag_detector_t *td;
apriltag_detection_info_t info;

/******************************************************************************* 
 *                               Definitions 
 ******************************************************************************/ 
#define ROUND2(x)    std::round(x * 100) / 100
#define ROUND3(x)    std::round(x * 1000) / 1000
#define IDLOW    23
#define IDLARGE    25
#define SWITCH_ALTITUDE    3

using namespace std;
using namespace sensor_msgs;
using namespace cv;
/******************************************************************************* 
 *                                  Topic
 ******************************************************************************/ 
/* Publisher */
image_transport::Publisher read_frame_pub;
ros::Publisher tf_list_pub_;

/******************************************************************************* 
 *                                 Variables 
 ******************************************************************************/
/* Define global variables */
bool enable_blur;
int blur_window_size;
int image_fps;
int image_width = 1920;
int image_height = 1080;
uint8_t switch_ID = 25;
/* Offset bwt the center of markers in coordinate marker*/
float marker_size = tagSIZE;
string marker_tf_prefix;

/******************************************************************************* 
 *                                  Code 
 ******************************************************************************/ 

void init_handler(int x) {
    /* disconnect and exit gracefully */
    ros::shutdown();
    exit(0);
}

tf2::Vector3 cv_vector3d_to_tf_vector3(const Vec3d &vec)
{
    return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion cv_vector3d_to_tf_quaternion(const Vec3d &rotation_vector)
{
    // Mat rotation_matrix; 
    auto ax    = rotation_vector[0], ay = rotation_vector[1], az = rotation_vector[2];
    auto angle = sqrt(ax * ax + ay * ay + az * az);
    auto cosa  = cos(angle * 0.5);
    auto sina  = sin(angle * 0.5);
    auto qx    = ax * sina / angle;
    auto qy    = ay * sina / angle;
    auto qz    = az * sina / angle;
    auto qw    = cosa;
    tf2::Quaternion q;
    q.setValue(qx, qy, qz, qw);

    return q;
}

tf2::Transform create_transform(const Vec3d &tvec, const Vec3d &rotation_vector)
{
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion(rotation_vector));
    return transform;
}

void callback(const ImageConstPtr &image_msg)
{
    string frame_id = image_msg->header.frame_id;
    auto image = cv_bridge::toCvShare(image_msg)->image;    /* To process */
    vector<int> ids_m;
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    vector<vector<Point2f>> corners_cvt;

    /* Smooth the image to improve detection results */
    if (enable_blur)
    {
        GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0, 0);
    }

    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);

    // Make an image_u8_t header for the Mat data
    image_u8_t im =
    {
        .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
    };
    zarray_t *detections = apriltag_detector_detect(td, &im);

    // Draw detection outlines
    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t *det;
        apriltag_pose_t pose;
        zarray_get(detections, i, &det);
        if (det->id == 8)
        {
            Mat rotation_matrix(3, 3, CV_64F);
            Mat rvec1;
            Vec3d rotation_vector, translation_vector;

            printf("detection %3d: id %4d, hamming %d, size: %f\n", i, det->id, det->hamming, marker_size);
            info.det = det;
            double err = estimate_tag_pose(&info, &pose);
            cout << "Pose estimation:" << endl;
            cout << "x: " << pose.t->data[0] << endl;
            cout << "y: " << pose.t->data[1] << endl;
            cout << "z: " << pose.t->data[2] << endl;

            rotation_matrix.at<double>(0,0) = pose.R->data[0];
            rotation_matrix.at<double>(0,1) = pose.R->data[1];
            rotation_matrix.at<double>(0,2) = pose.R->data[2];
            rotation_matrix.at<double>(1,0) = pose.R->data[3];
            rotation_matrix.at<double>(1,1) = pose.R->data[4];
            rotation_matrix.at<double>(1,2) = pose.R->data[5];
            rotation_matrix.at<double>(2,0) = pose.R->data[6];
            rotation_matrix.at<double>(2,1) = pose.R->data[7];
            rotation_matrix.at<double>(2,2) = pose.R->data[8];

            /*
            float sy = sqrt(pose.R->data[0] * pose.R->data[0] +  pose.R->data[3] * pose.R->data[3] );
            bool singular = sy < 1e-6; // If
            float x, y, z;
            if (!singular)
            {
                x = atan2(pose.R->data[7] , pose.R->data[8]);
                y = atan2(-pose.R->data[6], sy);
                z = atan2(pose.R->data[3], pose.R->data[0]);
            }
            else
            {
                x = atan2(-pose.R->data[5], pose.R->data[4]);
                y = atan2(-pose.R->data[6], sy);
                z = 0;
            }
            x  = x*(180/3.14);
            y = y*(180/3.14);
            z   = z*(180/3.14);
            cout << "row: " << x << endl;
            cout << "pitch: " << y << endl;
            cout << "yaw: " << z << endl;
            */
            Rodrigues(rotation_matrix, rvec1);
            /* Convert Mat to Vec3d */
            rvec1.convertTo(rotation_vector, CV_64F);
            
            translation_vector[0] = pose.t->data[0];
            translation_vector[1] = pose.t->data[1];
            translation_vector[2] = pose.t->data[2];

           /*Publish TFs for each of the markers*/
            static tf2_ros::TransformBroadcaster br;
            auto stamp = ros::Time::now();

            /*Create and publish tf message for each marker*/
            tf2_msgs::TFMessage tf_msg_list;
            geometry_msgs::TransformStamped tf_msg;
            stringstream ss;

            auto translation_vector_s      = translation_vector;
            auto rotation_vector_s         = rotation_vector;
            auto transform                 = create_transform(translation_vector_s, rotation_vector_s);
            ss << marker_tf_prefix << det->id;
            tf_msg.header.stamp            = stamp;
            tf_msg.header.frame_id         = frame_id;
            tf_msg.child_frame_id          = ss.str();
            tf_msg.transform.translation.x = transform.getOrigin().getX();
            tf_msg.transform.translation.y = transform.getOrigin().getY();
            tf_msg.transform.translation.z = transform.getOrigin().getZ();
            tf_msg.transform.rotation.x    = transform.getRotation().getX();
            tf_msg.transform.rotation.y    = transform.getRotation().getY();
            tf_msg.transform.rotation.z    = transform.getRotation().getZ();
            tf_msg.transform.rotation.w    = transform.getRotation().getW();
            tf_msg_list.transforms.push_back(tf_msg);
            br.sendTransform(tf_msg);

            if(tf_msg_list.transforms.size())
            {
                tf_list_pub_.publish(tf_msg_list);
            }
        }
    }

    apriltag_detections_destroy(detections);
}

int main(int argc, char *argv[])
{
    signal(SIGINT, init_handler);
    getopt_t *getopt = getopt_create();

    getopt_add_bool(getopt, 'h', "help", 0, "Show this help");
    getopt_add_bool(getopt, 'd', "debug", 0, "Enable debugging output (slow)");
    getopt_add_bool(getopt, 'q', "quiet", 0, "Reduce output");
    getopt_add_string(getopt, 'f', "family", "tagCustom48h12", "Tag family to use");
    getopt_add_int(getopt, 't', "threads", "1", "Use this many CPU threads");
    getopt_add_double(getopt, 'x', "decimate", "2.0", "Decimate input image by this factor");
    getopt_add_double(getopt, 'b', "blur", "0.0", "Apply low-pass blur to input");
    getopt_add_bool(getopt, '0', "refine-edges", 1, "Spend more time trying to align edges of tags");
    getopt_add_int(getopt, 'c', "cam", "0", "Camera is used to get stream");

    if (!getopt_parse(getopt, argc, argv, 1) ||
            getopt_get_bool(getopt, "help")) {
        printf("Usage: %s [options]\n", argv[0]);
        getopt_do_usage(getopt);
        exit(0);
    }

    /* Initialize tag detector with options. */
    apriltag_family_t *tf = NULL;
    const char *famname = getopt_get_string(getopt, "family");
    if (!strcmp(famname, "tag36h11")) {
        tf = tag36h11_create();
    } else if (!strcmp(famname, "tag25h9")) {
        tf = tag25h9_create();
    } else if (!strcmp(famname, "tag16h5")) {
        tf = tag16h5_create();
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tf = tagCircle21h7_create();
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tf = tagCircle49h12_create();
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tf = tagStandard41h12_create();
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tf = tagStandard52h13_create();
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tf = tagCustom48h12_create();
    } else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        exit(-1);
    }

    td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = getopt_get_double(getopt, "decimate");
    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = getopt_get_int(getopt, "threads");
    td->debug = getopt_get_bool(getopt, "debug");
    td->refine_edges = getopt_get_bool(getopt, "refine-edges");

    /*Initalize ROS node*/
    int queue_size = 10;
    ros::init(argc, argv, "apriltag_detect_node");
    ros::NodeHandle nh("~");
    string rgb_topic, rgb_info_topic, dictionary_name;

    nh.getParam("camera", rgb_topic);
    nh.getParam("marker_size", marker_size);
    nh.getParam("image_fps", image_fps);
    nh.getParam("image_width", image_width);
    nh.getParam("image_height", image_height);
    nh.getParam("tf_prefix", marker_tf_prefix);
    nh.getParam("enable_blur", enable_blur);
    nh.getParam("blur_window_size", blur_window_size);

    /* First create an apriltag_detection_info_t struct using your known parameters. */
    info.tagsize = marker_size;
    info.fx = FX;
    info.fy = FY;
    info.cx = CX;
    info.cy = CY;

    /* camera */
    ros::Subscriber rgb_sub = nh.subscribe(rgb_topic.c_str(), queue_size, callback);

    /*Publisher:*/
    image_transport::ImageTransport it(nh);
    read_frame_pub  = it.advertise("/camera/color/image_raw", 10);
    tf_list_pub_    = nh.advertise<tf2_msgs::TFMessage>("/tf_list", 10);
    ros::spin();

    /* Free Memory */
    apriltag_detector_destroy(td);

    if (!strcmp(famname, "tag36h11")) {
        tag36h11_destroy(tf);
    } else if (!strcmp(famname, "tag25h9")) {
        tag25h9_destroy(tf);
    } else if (!strcmp(famname, "tag16h5")) {
        tag16h5_destroy(tf);
    } else if (!strcmp(famname, "tagCircle21h7")) {
        tagCircle21h7_destroy(tf);
    } else if (!strcmp(famname, "tagCircle49h12")) {
        tagCircle49h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard41h12")) {
        tagStandard41h12_destroy(tf);
    } else if (!strcmp(famname, "tagStandard52h13")) {
        tagStandard52h13_destroy(tf);
    } else if (!strcmp(famname, "tagCustom48h12")) {
        tagCustom48h12_destroy(tf);
    }
    getopt_destroy(getopt);

    return 0;
}
