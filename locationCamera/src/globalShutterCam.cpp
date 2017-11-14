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
#include <std_msgs/String.h>

#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include<stdio.h>
#include <math.h>
#include <unistd.h>

boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_L,cinfo_R;
void pubImage(image_transport::CameraPublisher& image_pub_, cv::Mat image,int&ready, std::string ID,boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_){
    cv_bridge::CvImage out_msg;

    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));



    out_msg.encoding = sensor_msgs::image_encodings::RGB8;

    out_msg.image = image;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = ID;
    ci->header.frame_id = out_msg.header.frame_id;
    ci->header.stamp = out_msg.header.stamp;


    image_pub_.publish(*out_msg.toImageMsg(),*ci);

    ready = 1;
    return;
}
bool isSwapped = 0;
void swapCB(std_msgs::StringConstPtr msg){
    isSwapped = !isSwapped;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "doubleCamManager");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);


    image_transport::CameraPublisher imagePubL;
    imagePubL = it_.advertiseCamera("left/image/", 1);
    image_transport::CameraPublisher imagePubR;
    imagePubR = it_.advertiseCamera("right/image/", 1);
    ros::Subscriber swapSub = node.subscribe("/swap",1,&swapCB);
    cv::VideoCapture cap(0);

    cv::VideoCapture cap2(1);

    if(!cap.isOpened() && !cap2.isOpened())  // check if we succeeded
        return -1;


    std::string camera_info_urlL, camera_name_L, frame_id_L,camera_info_urlR, camera_name_R, frame_id_R;

    node.param<std::string>("camera_info_url",camera_info_urlL,"");
    node.param<std::string>("frame_id", frame_id_L, "left");
    node.param("camera_name", camera_name_L, std::string("left"));
    std::stringstream cinfo_nameL;
    cinfo_nameL << "Left";
    cinfo_L.reset(new camera_info_manager::CameraInfoManager(ros::NodeHandle("left"), camera_name_L, camera_info_urlL));

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
    node.param<std::string>("frame_id", frame_id_R, "Right");
    node.param("camera_name", camera_name_R, std::string("Right"));
    std::stringstream cinfo_nameR;
    cinfo_nameR << "Right";
    cinfo_R.reset(new camera_info_manager::CameraInfoManager(ros::NodeHandle("right"), camera_name_R, camera_info_urlR));

    if (!cinfo_R->isCalibrated())
       {
         cinfo_R->setCameraName(camera_name_R);
         sensor_msgs::CameraInfo camera_info;
         camera_info.header.frame_id = frame_id_R;
         camera_info.width = 640;
         camera_info.height = 480;
         cinfo_R->setCameraInfo(camera_info);
       }


    cv::Mat frameL,frameR;

    cv::waitKey(1);
    int LeftReady = 1;
    int RightReady = 1;
    int frameNum = 0;
    cv::Mat test, test2,test3, test4;
    while(ros::ok()){
        frameNum++;
        if(isSwapped){
        cap >> frameL; // get a new frame from camera
        cap2 >> frameR ;
        }
        else{
            cap >> frameR; // get a new frame from camera
            cap2 >> frameL ;
        }
//        cv::imshow("frameL", frameL);
//        cv::imshow("frameR", frameR);

        if(LeftReady == 1){
            LeftReady = 0;
            std::thread fullResThread = std::thread(pubImage,std::ref(imagePubL),frameL,std::ref(LeftReady),"Left",cinfo_L);
            fullResThread.detach();
        }
        if(RightReady == 1){
            RightReady =0;
            std::thread fullResThread = std::thread(pubImage,std::ref(imagePubR),frameR,std::ref(RightReady),"Right",cinfo_R);
            fullResThread.detach();
        }
//        if(frameNum == 299){

//            frameL.copyTo(test);
//            frameR.copyTo(test2);

//        }
//        if(frameNum == 300){
//            frameL.copyTo(test3);
//            frameR.copyTo(test4);
//            cv::imshow("frameL299", test);
//            cv::imshow("frameR299", test2);
//            cv::imshow("frameL300", test3);
//            cv::imshow("frameR300", test4);
//            cv::waitKey(1000);
//        }
        cv::waitKey(1);
        ros::spinOnce();

    }



    return 0;
}
