#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace cv;
using namespace aruco;
using namespace std;

    float markerSize = 0.04;
    aruco::CameraParameters params;
    int haveParams = 0;
    tf::TransformBroadcaster* br;
    tf::TransformListener* tl;

 tf::Transform   getTf(const cv::Mat &Rvec, const cv::Mat &Tvec)
    {
        cv::Mat rot(3, 3, CV_32FC1);
        cv::Rodrigues(Rvec, rot);

        cv::Mat rotate_to_sys(3, 3, CV_32FC1);
        /**
        /* Fixed the rotation to meet the ROS system
        /* Doing a basic rotation around X with theta=PI
        /* By Sahloul
        /* See http://en.wikipedia.org/wiki/Rotation_matrix for details
        */

        //	1	0	0
        //	0	-1	0
        //	0	0	-1
        rotate_to_sys.at<float>(0,0) = 1.0;
        rotate_to_sys.at<float>(0,1) = 0.0;
        rotate_to_sys.at<float>(0,2) = 0.0;
        rotate_to_sys.at<float>(1,0) = 0.0;
        rotate_to_sys.at<float>(1,1) = -1.0;
        rotate_to_sys.at<float>(1,2) = 0.0;
        rotate_to_sys.at<float>(2,0) = 0.0;
        rotate_to_sys.at<float>(2,1) = 0.0;
        rotate_to_sys.at<float>(2,2) = -1.0;
        rot = rot*rotate_to_sys.t();

        tf::Matrix3x3 tf_rot(rot.at<float>(0,0), rot.at<float>(0,1), rot.at<float>(0,2),
            rot.at<float>(1,0), rot.at<float>(1,1), rot.at<float>(1,2),
            rot.at<float>(2,0), rot.at<float>(2,1), rot.at<float>(2,2));

        tf::Vector3 tf_orig(Tvec.at<float>(0,0), Tvec.at<float>(1,0), Tvec.at<float>(2,0));

        return tf::Transform(tf_rot, tf_orig);
    }

void camInfoCB(sensor_msgs::CameraInfoConstPtr msg){
    cv::Mat intrinsic = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distCoeff = cv::Mat(msg->D.size(),1,CV_32FC1);
    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++){
            intrinsic.at<float>(i,j) = msg->K[3*i+j];
        }
    }
    for(int i = 0; i < msg->D.size(); i++){
        distCoeff.at<float>(i) = msg->D[i];
    }
    params.setParams(intrinsic,distCoeff,cv::Size(640,480));
    haveParams = 1;

}

void imageCB(sensor_msgs::ImageConstPtr msg){
    cv::Mat InImage =  cv_bridge::toCvShare(msg, "bgr8")->image;
    if(haveParams){
        try
        {
            MarkerDetector MDetector;
            vector<Marker> Markers;
            //read the input image
         //Ok, let's detect
            MDetector.detect(InImage,Markers,params,0.04,false);
            //for each marker, draw info and its boundaries in the image
            for (unsigned int i=0;i<Markers.size();i++) {
                cout<<Markers[i]<<endl;
                Markers[i].draw(InImage,Scalar(0,0,255),2);
                std::stringstream ss;
                ss << "marker" << Markers[i].id;
                tf::StampedTransform trans;
                trans.child_frame_id_ = ss.str();
                trans.setData(getTf(Markers[i].Rvec,Markers[i].Tvec));
                trans.frame_id_ = msg->header.frame_id;
                trans.stamp_ = msg->header.stamp;
                br->sendTransform(trans);

            }
            cv::imshow("in",InImage);
            cv::waitKey(1);//wait for key to be pressed
        } catch (std::exception &ex)
        {
            cout<<"Exception :"<<ex.what()<<endl;
        }
    }
}





int main(int argc,char **argv)
{

    ros::init(argc, argv, "aruco_node");
    ros::NodeHandle node;
    br = new tf::TransformBroadcaster;
    image_transport::ImageTransport it_(node);
    image_transport::Subscriber imSub = it_.subscribe("/camera/image",1, imageCB);
    ros::Subscriber infoSub = node.subscribe("/camera/camera_info",1,camInfoCB);

    while(1){
        ros::spinOnce();
    }
}
