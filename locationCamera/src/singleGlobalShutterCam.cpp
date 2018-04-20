#include <ros/ros.h>
#include <ros/package.h>
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

void InitmyUndistort(cv::Mat input, boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_, cv::Mat &map1,cv::Mat &map2){
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));

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

    intrinsic.at<float>(2,2)= 1;
    cv::Mat newCamMatrix = cv::getOptimalNewCameraMatrix(intrinsic,distCoeff,input.size(),0);
    cv::Mat R;
    cv::initUndistortRectifyMap(intrinsic,distCoeff, R, newCamMatrix, input.size(), CV_16SC2, map1, map2);
}

cv::Mat myUndistort(cv::Mat &input,cv::Mat &map1,cv::Mat &map2){
    cv::Mat output;
    cv::Mat R;
    remap(input, output, map1,map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);
//    for(int i = 0 ; i < 10000000; i++){

//    }
    return output;
}
void pubImage(image_transport::CameraPublisher& image_pub_, cv::Mat image,int&ready, std::string ID,boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_, int &buffer){
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
    buffer = 1-buffer;
    return;
}

void undistortThread(cv::Mat image,cv::Mat &out,cv::Mat &map1,cv::Mat &map2, int &ready, int &buffer){
    out = myUndistort(image,map1,map2);
    ready = 1;
    buffer = 1-buffer;
}

void collectImThread(cv::VideoCapture vid,cv::Mat &image, int &ready, int &frameNumber){
    vid >> image;
    frameNumber++;
    ready = 1;
}

void serialThread(image_transport::CameraPublisher& image_pub_, cv::Mat image,int&ready, std::string ID,boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_,cv::Mat &map1,cv::Mat &map2,int &threadCount,ros::Time sampleT,int shouldCorrect ){
    threadCount++;
    cv::Mat imCopy;
    image.copyTo(imCopy);
    cv_bridge::CvImage out_msg;

    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));



    out_msg.encoding = sensor_msgs::image_encodings::RGB8;
    if(shouldCorrect){
        out_msg.image = myUndistort(imCopy,map1,map2);
    }
    else out_msg.image = imCopy;
    out_msg.header.stamp =sampleT ;
    out_msg.header.frame_id = ID;
    ci->header.frame_id = out_msg.header.frame_id;
    ci->header.stamp = out_msg.header.stamp;


    image_pub_.publish(*out_msg.toImageMsg(),*ci);

    ready = 1;
    threadCount --;
    return;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "singleCamManager");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);

    bool shouldRecordToFile = 0;
    std::string recPath;
    image_transport::CameraPublisher imagePubL,imagePubR;
    imagePubL = it_.advertiseCamera("camera/image/", 1);
    imagePubR = it_.advertiseCamera("camera_corrected/image/", 1);

    if(argc > 1){
        shouldRecordToFile = 1;
        recPath = ros::package::getPath("locationCamera");
    }

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



    cv::Mat frame0, frame1; // double buffered read.
    cv::Mat convert0, convert1;

    cv::waitKey(1);

    int publishComplete = 1;
    int threadCount =0;
    int shouldCorrect = 0;
    int count = 0;
    if(argc > 1){
        shouldCorrect = atoi(argv[1]);
    }
    cv::Mat map1;
    cv::Mat map2;
    //one serial implenetation to fill mats
    cap >> frame0;
    cap >> frame1;
    InitmyUndistort(frame0,cinfo_L,map1,map2);
    unDistIm = myUndistort(frame0,map1,map2);
    ros::Rate rate(31);
    while(ros::ok()){


        if(publishComplete && threadCount < 2){
            count++;
            cap >> frame0;
            ros::Time sampleT = ros::Time::now();
            if(shouldCorrect){
                if(count %2 == 0){
                std::thread publishThread = std::thread(serialThread,std::ref(imagePubR), frame0,std::ref(publishComplete), "camera", cinfo_R,std::ref(map1),std::ref(map2), std::ref(threadCount),sampleT,shouldCorrect);
                publishThread.detach();
                }
                if(count %2 == 1){
                std::thread publishThread = std::thread(serialThread,std::ref(imagePubL), frame0,std::ref(publishComplete), "camera", cinfo_R,std::ref(map1),std::ref(map2), std::ref(threadCount),sampleT,shouldCorrect);
                publishThread.detach();
                }
            }
            else {
                std::thread publishThread = std::thread(serialThread,std::ref(imagePubL), frame0,std::ref(publishComplete), "camera", cinfo_L,std::ref(map1),std::ref(map2), std::ref(threadCount),sampleT,shouldCorrect);
                publishThread.detach();

            }
            if(shouldRecordToFile == 1){
                std::stringstream ss;
                ss <<recPath<< "/savedimgBG/" << count << ".jpg";
                //cv::cvtColor(frame0, frame0, cv::COLOR_GRAY2BGR);
                cv::imwrite(ss.str(),frame0);
                cv::imshow("liveView",frame0);
                cv::waitKey(1);
                ros::Duration sleep(0.1);
                sleep.sleep();
            }



        }

        rate.sleep();
        ros::spinOnce();


    }



    return 0;
}
