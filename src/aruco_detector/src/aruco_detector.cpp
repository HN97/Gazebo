/**************************************************************************//**
   @author  Markus Lamprecht
   @date    March 2019
   @link    www.simact.de/about_me
   @Copyright (c) 2019 Markus Lamprecht. BSD
 *****************************************************************************/

#include <csignal>
#include <iostream>
#include <map>      /* used for hashmap to give certainty */
#include <vector>   /* used in hashmap */
#include <numeric>  /* used for summing a vector */
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Int16.h"
#include <image_geometry/pinhole_camera_model.h>
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
/******************************************************************************* 

 *                               Definitions 

 ******************************************************************************/ 
#define SSTR(x)         static_cast<std::ostringstream&>(std::ostringstream() << std::dec << x).str()
#define ROUND2(x)       std::round(x * 100) / 100
#define ROUND3(x)       std::round(x * 1000) / 1000
#define IDLOW              23
#define IDLARGE            25
#define SWITCH_ALTITUDE    4
/******************************************************************************* 

 *                                Namespace

 ******************************************************************************/ 
using namespace std;
using namespace sensor_msgs;
using namespace cv;
/******************************************************************************* 

 *                                  Topic

 ******************************************************************************/ 
/* Publisher */
image_transport::Publisher result_img_pub_;
image_geometry::PinholeCameraModel camera_model;
ros::Publisher tf_list_pub_;
/******************************************************************************* 

 *                                 Variables 

 ******************************************************************************/
/* Define global variables */
bool camera_model_computed = false;
bool show_detections;
bool enable_blur           = true;
int blur_window_size       = 7;
int image_fps              = 14;
int image_width            = 1920;
int image_height           = 1080;
/* Offset bwt the center of markers in coordinate marker*/
float marker_size;
string marker_tf_prefix;
Mat distortion_coefficients;
Matx33d intrinsic_matrix;
Ptr<aruco::DetectorParameters> detector_params;
Ptr<cv::aruco::Dictionary> dictionary;
/**/
std::ostringstream vector_to_marker;


// hashmap used for uncertainty:
int num_detected   = 10;  // =0 -> not used
int min_prec_value = 80; // min precentage value to be a detected marker.
int switch_ID      = 25;
map<int,  std::vector<int>  > ids_hashmap;   // key: ids, value: number within last 100 imgs

/******************************************************************************* 

 *                                  Code 

 ******************************************************************************/ 
void int_handler(int x) {
    /* disconnect and exit gracefully */
    if(show_detections)
    {
        cv::destroyAllWindows();
    }

    ros::shutdown();
    exit(0);
}

tf2::Vector3 cv_vector3d_to_tf_vector3(const Vec3d &vec)
{
    return {vec[0], vec[1], vec[2]};
}

tf2::Quaternion cv_vector3d_to_tf_quaternion(const Vec3d &rotation_vector)
{
    Mat rotation_matrix;

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
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    roll  = roll*(180/3.14);
    pitch = pitch*(180/3.14);
    yaw   = yaw*(180/3.14);
    // cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << endl;

    return q;
}

tf2::Transform create_transform(const Vec3d &tvec, const Vec3d &rotation_vector)
{
    tf2::Transform transform;
    transform.setOrigin(cv_vector3d_to_tf_vector3(tvec));
    transform.setRotation(cv_vector3d_to_tf_quaternion(rotation_vector));
    return transform;
}

void callback_camera_info(const CameraInfoConstPtr &msg)
{
    if (camera_model_computed)
    {
        return;
    }
    camera_model.fromCameraInfo(msg);
    camera_model.distortionCoeffs().copyTo(distortion_coefficients);
    intrinsic_matrix = camera_model.intrinsicMatrix();
    camera_model_computed = true;
    ROS_INFO("camera model is successfully computed");
}

void update_params_cb(const std_msgs::Empty &msg)
{

} 

void callback(const ImageConstPtr &image_msg)
{
    string frame_id = image_msg->header.frame_id;
    auto image = cv_bridge::toCvShare(image_msg)->image;
    cv::Mat display_image(image);
    vector<int> ids_m;
    vector<int> ids;
    vector<vector<Point2f>> corners, rejected;
    vector<vector<Point2f>> corners_cvt ;

    if (!camera_model_computed)
    {
        ROS_INFO("camera model is not computed yet");
        return;
    }
    /* Smooth the image to improve detection results */
    if (enable_blur)
    {
        GaussianBlur(image, image, Size(blur_window_size, blur_window_size), 0, 0);
    }
    /* Detect the markers */
    aruco::detectMarkers(image, dictionary, corners, ids_m, detector_params, rejected);

    cv::line(display_image, cv::Point(960, 0), cv::Point(960, 1080), cv::Scalar(245, 7, 96), 2);    /* y */
    cv::line(display_image, cv::Point(0, 540), cv::Point(1920, 540), cv::Scalar(11, 220, 93), 2);   /* x */
 
    /* Show image if no markers are detected */
    if (ids_m.empty())
    {
        cv::putText(display_image, "Markers not found", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 2);
        if (show_detections)
        {
            if (result_img_pub_.getNumSubscribers() > 0)
            {
                result_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", display_image).toImageMsg());
            }
        }
    }

    if(ids_m.size()>0)
    {
        for(int i = 0;i<ids_m.size();i++)
        {
            if (ids_m[i] == switch_ID)
            {
                ROS_INFO("Marker ID: [%d]", ids_m[i]);
                ROS_INFO("Marker size: [%f]", marker_size);
                ids.push_back(ids_m[i]);
                corners_cvt.push_back(corners[i]);
            }
        }
        /*Compute poses of markers*/
        vector<Vec3d> rotation_vectors, translation_vectors;
        aruco::estimatePoseSingleMarkers(corners_cvt, marker_size, intrinsic_matrix, distortion_coefficients,
                                         rotation_vectors, translation_vectors);

        for (auto i = 0; i < rotation_vectors.size(); ++i)
        {
            aruco::drawAxis(image, intrinsic_matrix, distortion_coefficients,
                            rotation_vectors[i], translation_vectors[i], marker_size * 0.5f);
            /* Display Translation vector */
            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) << std::fixed 
                             << "x: " << std::setw(8) << translation_vectors[i](0) << " m";
            cv::putText( display_image, vector_to_marker.str(),
                        cv::Point(10, 95), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        CV_RGB(255, 255, 0), 2);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) << std::fixed 
                             << "y: " << std::setw(8) << translation_vectors[i](1) << " m";
            cv::putText( display_image, vector_to_marker.str(),
                        cv::Point(10, 120), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        CV_RGB(255, 255, 0), 2);

            vector_to_marker.str(std::string());
            vector_to_marker << std::setprecision(4) << std::fixed 
                             << "z: " << std::setw(8) << translation_vectors[i](2) << " m";
            cv::putText( display_image, vector_to_marker.str(),
                        cv::Point(10, 145), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                        CV_RGB(255, 255, 0), 2);

            if (SWITCH_ALTITUDE > translation_vectors[0](2))
            {
                switch_ID = IDLOW;
                marker_size = 0.2;
            }
            else
            {
                switch_ID = IDLARGE;
                marker_size = 0.5;
            }
            ROS_INFO("x: [%f]", translation_vectors[i](0));
            ROS_INFO("y: [%f]", translation_vectors[i](1));
            ROS_INFO("z: [%f]", translation_vectors[i](2));
            // vector<Point2f> topLeft, bottomRight;
            // int cX, cY;
            // topLeft.push_back(corners_cvt[0][0]);
            // // bottomRight = corners_cvt[0][3];
            // cout << corners_cvt[0][1] << endl;
            // cout << topLeft[0]<< endl;

            // cX = int((topLeft[0].x + bottomRight[0].x)/2.0);
            // cY = int((topLeft[0].y + bottomRight[0].y)/2.0);
            // circle( display_image, Point(cX, cY), 4, Scalar(255,0,255), 3, LINE_AA);
        }
        /*Draw marker poses*/
        if (show_detections)
        {
            aruco::drawDetectedMarkers(display_image, corners_cvt, ids);
        }
        ROS_INFO("%d", result_img_pub_.getNumSubscribers());
        if (result_img_pub_.getNumSubscribers() > 0)
        {
            putText(display_image, ""+SSTR(image_fps)+"FPS m. size: "+SSTR(marker_size)+" m", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, CV_RGB(255, 255, 0), 2);
            for(int i = 0; i<ids.size();i++)
            {
                    Vec3d distance_z_first = translation_vectors[i];
                    double distance_z = ROUND3(distance_z_first[2]);
                    putText(display_image, "id: "+SSTR(ids[i])+" z dis: "+SSTR(distance_z)+" m", cv::Point(10, 70+i*30), cv::FONT_HERSHEY_SIMPLEX, 0.9, CV_RGB(0, 255, 0), 2);
                    result_img_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", display_image).toImageMsg());
            }
        }
        /*Publish TFs for each of the markers*/
        static tf2_ros::TransformBroadcaster br;
        auto stamp = ros::Time::now();

        /*Create and publish tf message for each marker*/
        tf2_msgs::TFMessage tf_msg_list;

        for (auto i = 0; i < rotation_vectors.size(); ++i)
        {
                geometry_msgs::TransformStamped tf_msg;
                stringstream ss;

                auto translation_vector = translation_vectors[i];
                auto rotation_vector    = rotation_vectors[i];
                auto transform          = create_transform(translation_vector, rotation_vector);
                ss << marker_tf_prefix << ids[i];
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
        }
        
        if( tf_msg_list.transforms.size() )
        {
            tf_list_pub_.publish(tf_msg_list);
        }
    }
}

/*TODO: slider extension mach ne hashmap von int,array*/

int main(int argc, char **argv)
{
    map<string, aruco::PREDEFINED_DICTIONARY_NAME> dictionary_names;
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_50", aruco::DICT_4X4_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_100", aruco::DICT_4X4_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_250", aruco::DICT_4X4_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_4X4_1000", aruco::DICT_4X4_1000));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_50", aruco::DICT_5X5_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_100", aruco::DICT_5X5_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_250", aruco::DICT_5X5_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_5X5_1000", aruco::DICT_5X5_1000));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_50", aruco::DICT_6X6_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_100", aruco::DICT_6X6_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_250", aruco::DICT_6X6_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_6X6_1000", aruco::DICT_6X6_1000));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_50", aruco::DICT_7X7_50));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_100", aruco::DICT_7X7_100));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_250", aruco::DICT_7X7_250));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_7X7_1000", aruco::DICT_7X7_1000));
    dictionary_names.insert(pair<string, aruco::PREDEFINED_DICTIONARY_NAME>("DICT_ARUCO_ORIGINAL", aruco::DICT_ARUCO_ORIGINAL));

    signal(SIGINT, int_handler);

    /*Initalize ROS node*/
    int queue_size = 10;
    ros::init(argc, argv, "aruco_detect_node");
    ros::NodeHandle nh("~");
    string rgb_topic, rgb_info_topic, dictionary_name;

    nh.param("camera", rgb_topic, string("/camera/color/image_raw"));
    nh.param("camera_info", rgb_info_topic, string("/camera/color/camera_info"));
    nh.param("show_detections", show_detections, true);
    nh.param("tf_prefix", marker_tf_prefix, string("marker"));
    nh.param("marker_size", marker_size, 0.5f);
    nh.param("enable_blur", enable_blur, true);
    nh.param("blur_window_size", blur_window_size, 7);
    nh.param("image_fps", image_fps, 14);
    nh.param("image_width", image_width, 1920);
    nh.param("image_height", image_height, 1080);
    nh.param("num_detected", num_detected, 50);
    nh.param("min_prec_value", min_prec_value, 80);

    detector_params = aruco::DetectorParameters::create();
    detector_params->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    nh.param("dictionary_name", dictionary_name, string("DICT_6X6_250"));   /* default DICT_6X6_250 */
    nh.param("aruco_adaptiveThreshWinSizeStep", detector_params->adaptiveThreshWinSizeStep, 4);
    /* Configure ARUCO marker detector */
    dictionary = aruco::getPredefinedDictionary(dictionary_names[dictionary_name]);
    ROS_DEBUG("%f", marker_size);
    /* gazebo plugin */
    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", queue_size, callback);
    ros::Subscriber rgb_info_sub = nh.subscribe("/camera/color/camera_info", queue_size, callback_camera_info);
    /* camera */
    // ros::Subscriber rgb_sub = nh.subscribe(rgb_topic.c_str(), queue_size, callback);
    // ros::Subscriber rgb_info_sub = nh.subscribe(rgb_info_topic.c_str(), queue_size, callback_camera_info);
    // ros::Subscriber parameter_sub = nh.subscribe("/update_params", queue_size, update_params_cb);
    /*Publisher:*/
    image_transport::ImageTransport it(nh);
    result_img_pub_ = it.advertise("/result_img", 10);
    tf_list_pub_    = nh.advertise<tf2_msgs::TFMessage>("/tf_list", 1000);
    ros::spin();
    return 0;
}
