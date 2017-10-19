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


#include <iostream>
#include <ctype.h>
#include <stdlib.h>
#include<stdio.h>
#include <math.h>
#include <unistd.h>

tf::TransformListener* listenerPtr;
cv::Mat recentIm;
ros::Time recentImageTime;
ros::Time sentImageTime;
ros::Time finishedProcTime;
bool sent = 0;

void imageCB(sensor_msgs::ImageConstPtr im){
    //get the image from the message
    cv_bridge::toCvShare(im, "bgr8")->image.copyTo(recentIm);
    recentImageTime = im->header.stamp;

}

void processingFinished(std_msgs::TimeConstPtr msg){
    finishedProcTime = msg->data;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_limiter");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    image_transport::Subscriber imSub = it_.subscribe("/camera/image_raw",1, imageCB);
    ros::Subscriber finsishedSub = node.subscribe("/procFinished",1, processingFinished);
    image_transport::Publisher imPub = it_.advertise("limited",1);
    listenerPtr = new tf::TransformListener();

    while(ros::ok()){
        ros::spinOnce();
        if(!sent && recentIm.cols != 0){
            sent = 1;
            sentImageTime = recentImageTime;
            cv_bridge::CvImage msg;
            msg.image = recentIm;
            msg.encoding = "bgr8";
            msg.header.frame_id = "camera";
            msg.header.stamp = sentImageTime;
            imPub.publish(msg.toImageMsg());
        }
        std::string infoMsg,from, to;
        from = "camera";
        to = "ar_marker_3";
        bool gotMarker;
        try{
            static int good =0;
        gotMarker = listenerPtr->waitForTransform("/camera","/ar_marker_3",sentImageTime,ros::Duration(0.2));
        if(finishedProcTime != sentImageTime){
            //ros::Duration(0.2).sleep();
        }
        else{
            good++;
        }
        gotMarker = 1;
        }
        catch(...){
            gotMarker = 1;
        }

        if(gotMarker){
            gotMarker = 0;
            sent = 0;
        }
    }



    return 0;
}
