#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Time.h>
#include <sensor_msgs/Joy.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>
#include <visualization_msgs/Marker.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <thread>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>


#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include<stdio.h>
#include <math.h>
#include <unistd.h>
cv::Mat unDistIm ;
int correctedReady = 0;
boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_L,cinfo_R;
void pubImage(image_transport::CameraPublisher& image_pub_, cv::Mat image,int&ready, std::string ID,boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_,int shouldUndist){
    cv_bridge::CvImage out_msg;

    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));



    out_msg.encoding = sensor_msgs::image_encodings::RGB8;

    out_msg.image = image;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = ID;
    ci->header.frame_id = out_msg.header.frame_id;
    ci->header.stamp = out_msg.header.stamp;


    image_pub_.publish(*out_msg.toImageMsg(),*ci);

    if(shouldUndist){
        cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
        cv::Mat distCoeff = cv::Mat(ci->D.size(),1,CV_32FC1);
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                intrinsic.at<float>(i,j) = ci->K[3*i+j];
            }
        }
        for(int i = 0; i < ci->D.size(); i++){
            distCoeff.at<float>(i) = ci->D[i];
        }
        cv::undistort(image,unDistIm,intrinsic,distCoeff);

    }
    ready = 1;
    correctedReady = 1;
    return;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "singleCamManager");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);


    image_transport::CameraPublisher imagePubL,imagePubR;
    imagePubL = it_.advertiseCamera("camera/image/", 1);
    imagePubR = it_.advertiseCamera("camera_corrected/image/", 1);

    cv::VideoCapture cap(0);

    if(!cap.isOpened())  // check if we succeeded
        return -1;


    std::string camera_info_urlL, camera_name_L, frame_id_L,camera_info_urlR, camera_name_R, frame_id_R;

    node.param<std::string>("camera_info_url",camera_info_urlL,"");
    node.param<std::string>("frame_id", frame_id_L, "camera");
    node.param("camera_name", camera_name_L, std::string("camera"));
    std::stringstream cinfo_nameL;
    cinfo_nameL << "camera";
    cinfo_L.reset(new camera_info_manager::CameraInfoManager(ros::NodeHandle("camera"), camera_name_L, camera_info_urlL));

    if (!cinfo_L->isCalibrated())
       {
         cinfo_L->setCameraName(camera_name_L);
         sensor_msgs::CameraInfo camera_info;
         camera_info.header.frame_id = frame_id_L;
         camera_info.width = 640;
         camera_info.height = 480;
         cinfo_L->setCameraInfo(camera_info);
       }


    node.param<std::string>("camera_info_url",camera_info_urlR,"");
    node.param<std::string>("frame_id", frame_id_R, "camera");
    node.param("camera_name", camera_name_R, std::string("camera_corrected"));
    std::stringstream cinfo_nameR;
    cinfo_nameR << "camera_corrected";
    cinfo_R.reset(new camera_info_manager::CameraInfoManager(ros::NodeHandle("camera_corrected"), camera_name_R, camera_info_urlR));
    if (!cinfo_R->isCalibrated())
       {
         cinfo_R->setCameraName(camera_name_R);
         sensor_msgs::CameraInfo camera_info;
         camera_info.header.frame_id = frame_id_R;
         camera_info.width = 640;
         camera_info.height = 480;
         cinfo_R->setCameraInfo(camera_info);
       }



    cv::Mat frameL;

    cv::waitKey(1);
    int LeftReady = 1;
    int correctedPub = 1;

    int frameNum = 0;
    int lastFrame = 0;
    while(ros::ok()){
        frameNum++;
        cap >> frameL; // get a new frame from camera

        if(LeftReady == 1){
            LeftReady = 0;
            std::thread fullResThread = std::thread(pubImage,std::ref(imagePubL),frameL,std::ref(LeftReady),"camera",cinfo_L,1);
            fullResThread.detach();
        }
        if(unDistIm.cols != 0 && correctedPub && frameNum != lastFrame){
            lastFrame = frameNum;
            correctedReady = 0;
            std::thread correctedThread = std::thread(pubImage,std::ref(imagePubR),unDistIm,std::ref(correctedPub),"camera",cinfo_R,0);
            correctedThread.detach();
           // cv::imshow("corrected",unDistIm);
        }


        cv::waitKey(1);
        ros::spinOnce();

    }



    return 0;
}
