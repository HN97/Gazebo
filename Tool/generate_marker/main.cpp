// Import the aruco module in OpenCV
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <iostream>

using namespace std;

int main()
{
    string filename;
    int id;
    int size;
    cout << "Please enter infor4"<< endl;
    cout <<"ID Marker - Size: ";
    cin >> id >> size;
    cout << "Iamge name.png: ";
    cin >> filename;
    //cout << *filename << endl;
   cv::Mat markerImage;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::drawMarker (dictionary, id , size , markerImage, 1);
    cv::imwrite(filename, markerImage);

    return 0;
}

//g++ main.cpp -o out `pkg-config --cflags --libs opencv4`
