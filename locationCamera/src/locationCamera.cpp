
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include<stdio.h>
#include <math.h>
#include <unistd.h>

int FLowH = 0;
int FHighH = 10;

int FLowS = 68;
int FHighS = 210;

int FLowV = 84;
int FHighV = 201;

int LLowH = 95;
int LHighH = 145;

int LLowS = 0;
int LHighS = 205;

int LLowV = 43;
int LHighV = 207;

tf::TransformListener* listenerPtr;
tf::TransformBroadcaster* brPtr;
//sensor_msgs::CameraInfo storedInfo;

image_geometry::PinholeCameraModel model_;

void imageCB(sensor_msgs::ImageConstPtr im, sensor_msgs::CameraInfoConstPtr& cam_info){
    model_.fromCameraInfo(cam_info);
    //get the image from the message
    cv::Mat image =  cv_bridge::toCvShare(im, "bgr8")->image;
    cv::imshow("in",image);
    cv::waitKey(1);

    // threshold image for red and blue
    cv::Mat imgHSV,FimgThresholded, LimgThresholded;
    cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    cv::inRange(imgHSV, cv::Scalar(FLowH, FLowS, FLowV), cv::Scalar(FHighH,FHighS, FHighV), FimgThresholded); //Threshold the image
    cv::inRange(imgHSV, cv::Scalar(LLowH, LLowS, LLowV), cv::Scalar(LHighH,LHighS, LHighV), LimgThresholded); //Threshold the image
    cv::imshow("Front",FimgThresholded);
    cv::imshow("Light",LimgThresholded);
    cv::waitKey(1);


    //get transform for this timestamp to the world marker
    tf::StampedTransform transform;
    bool useIteration = 0;
    try{
    listenerPtr->lookupTransform("/camera", "/ar_marker_3",
                             ros::Time(0), transform);
    useIteration = 1;
    }catch(tf2::LookupException){

    }


    //find location on the plane of the red and blue blobs

    //pair up red and blue

    //construct transforms for pairs

    //publish transforms

}

//void imageCalibCB(sensor_msgs::CameraInfoConstPtr info){
//    storedInfo = *info;
//}



int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_location");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    image_transport::Subscriber imSub = it_.subscribe("/camera/image_raw",1, imageCB);
    listenerPtr = new tf::TransformListener();
    brPtr = new tf::TransformBroadcaster();
    //ros::Subscriber calibSub = node.subscribe("/camera/camera_info",1,imageCalibCB);

    cv::namedWindow("Control", CV_WINDOW_AUTOSIZE);
    //    //Create trackbars in "Control" window
    cvCreateTrackbar("FLowH", "Control", &FLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("FHighH", "Control", &FHighH, 179);

    cvCreateTrackbar("FLowS", "Control", &FLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("FHighS", "Control", &FHighS, 255);

    cvCreateTrackbar("FLowV", "Control", &FLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("FHighV", "Control", &FHighV, 255);
    cvCreateTrackbar("LLowH", "Control", &LLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("LHighH", "Control", &LHighH, 179);

    cvCreateTrackbar("LLowS", "Control", &LLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("LHighS", "Control", &LHighS, 255);

    cvCreateTrackbar("LLowV", "Control", &LLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("LHighV", "Control", &LHighV, 255);
    while(ros::ok()){
        ros::spinOnce();
    }



    return 0;
}
