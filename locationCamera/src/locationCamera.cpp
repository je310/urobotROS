
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

int FLowH = 74;
int FHighH = 131;

int FLowS = 9;
int FHighS = 255;

int FLowV = 103;
int FHighV = 255;

int LLowH = 0;
int LHighH = 119;

int LLowS = 0;
int LHighS = 255;

int LLowV = 0;
int LHighV = 255;
tf::Vector3 cvToVec(cv::Point3d in);
float findAngle(tf::Vector3 one, tf::Vector3 two);
tf::TransformListener* listenerPtr;
tf::TransformBroadcaster* brPtr;
//sensor_msgs::CameraInfo storedInfo;
bool gotInfo = 0;
int good = 0;
int bad  = 0;

//variables for the tracking section.
int numberRobots = 2;
float trackingErrLinear = 0.03;
float trackingErrAng = (3.14*2)/8;
bool initialising = 1;

struct robotTrackingInfo{
    int VisualID;
    tf::StampedTransform lastMeasuredPos;
    tf::StampedTransform guessPos;
    bool isTracked;
};

std::vector<robotTrackingInfo> trackingList;

sensor_msgs::CameraInfoConstPtr savedCamInfo;
void imageInfoCB(sensor_msgs::CameraInfoConstPtr camInfo){
    savedCamInfo = camInfo;
    gotInfo = 1;
}
ros::Publisher marker_pub,marker_pubLED;
ros::Publisher finishedPub;

void imageCB(sensor_msgs::ImageConstPtr im){
    if(!gotInfo) return;
    image_geometry::PinholeCameraModel model_;
    model_.fromCameraInfo(savedCamInfo);
    //get the image from the message
    cv::Mat image =  cv_bridge::toCvShare(im, "bgr8")->image;
    cv::imshow("in",image);
    cv::waitKey(1);

    // threshold image for red and blue
    cv::Mat imgHSV,FimgThresholded, LimgThresholded;
    cv::cvtColor(image, imgHSV, cv::COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
    cv::inRange(imgHSV, cv::Scalar(FLowH, FLowS, FLowV), cv::Scalar(FHighH,FHighS, FHighV), FimgThresholded); //Threshold the image
    cv::inRange(imgHSV, cv::Scalar(LLowH, LLowS, LLowV), cv::Scalar(LHighH,LHighS, LHighV), LimgThresholded); //Threshold the image
    // Create a structuring element
    int erosion_size = 1;
    cv::Mat element = getStructuringElement(cv::MORPH_CROSS,
                                            cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
                                            cv::Point(erosion_size, erosion_size) );
//    cv::erode(FimgThresholded,FimgThresholded,element);
  //  cv::dilate(FimgThresholded,FimgThresholded,element);
  //  cv::erode(LimgThresholded,LimgThresholded,element);
   // cv::dilate(LimgThresholded,LimgThresholded,element);
//     cv::threshold(FimgThresholded, FimgThresholded, 0.5, 255,cv::THRESH_BINARY_INV);
//     cv::threshold(LimgThresholded, LimgThresholded, 0.5, 255,cv::THRESH_BINARY_INV);
    cv::imshow("Front",FimgThresholded);
    cv::imshow("Light",LimgThresholded);
    cv::waitKey(1);

    // find the blobs.

    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

    // Change thresholds
//    params.minThreshold = 1;
//    params.maxThreshold = 2;


    // Filter by Area.
    params.filterByArea =false;
    params.minArea = 5;

    // Filter by Circularity
    params.filterByCircularity = false;
    params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = false;
    params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = false;
    params.minInertiaRatio = 0.01;

    params.filterByColor = true;
    params.blobColor = 255;
    //params.thresholdStep = 1;
//    // Set up the detector with default parameters.
    params.filterByArea = true;
    params.minArea = 1;
    params.maxArea = 1000;
    params.filterByColor = true;
    params.blobColor = 255;


    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

    std::vector<cv::KeyPoint> Fkeypoints,Lkeypoints;
    std::vector<tf::Vector3> frontsVec, ledsVec;

     detector->detect(  FimgThresholded, Fkeypoints);
     detector->detect(  LimgThresholded, Lkeypoints);

    //find location on the plane of the red and blue blobs
     if(Fkeypoints.size()> 0){
         for(int i =0; i < Fkeypoints.size(); i++){
            frontsVec.push_back(cvToVec(model_.projectPixelTo3dRay(Fkeypoints[i].pt)));
         }
     }
     if(Lkeypoints.size()> 0){
         for(int i =0; i < Lkeypoints.size(); i++){
            ledsVec.push_back(cvToVec(model_.projectPixelTo3dRay(Lkeypoints[i].pt)));
         }
     }

    //get transform for this timestamp to the world marker
    tf::StampedTransform transform;
    bool useIteration = 0;
    try{
         bool gotMarker = listenerPtr->waitForTransform("/camera","/ar_marker_3",im->header.stamp,ros::Duration(0.1));
        listenerPtr->lookupTransform("/ar_marker_3", "/camera",
                                     im->header.stamp, transform);
        useIteration = 1;
    }catch(...){

    }
    if(useIteration){
        if(1){
            //transform all the vectors to be relative to the marker;
            for(int i = 0; i < frontsVec.size(); i++ ){
                frontsVec[i] = transform(frontsVec[i]);
            }
            for(int i = 0; i < ledsVec.size(); i++ ){
                ledsVec[i] = transform(ledsVec[i]);
            }

            //extend the vectors to the 'floor', they now represent the 3d position on the blobs.
            for(int i = 0; i < frontsVec.size(); i++ ){
                tf::Vector3 cam = transform.getOrigin();
                tf::Vector3 dir = frontsVec[i] -cam ;
                float ratio = cam.getZ()/(dir.getZ());
                frontsVec[i] = cam - dir*ratio;
            }
            for(int i = 0; i < ledsVec.size(); i++ ){
                tf::Vector3 cam = transform.getOrigin();
                tf::Vector3 dir = ledsVec[i] -cam ;
                float ratio = cam.getZ()/(dir.getZ());
                ledsVec[i] = cam - dir*ratio;
            }

            visualization_msgs::Marker sphere_list;
            sphere_list.header.frame_id= "/ar_marker_3";
            sphere_list.header.stamp= transform.stamp_;
            sphere_list.ns= "spheres";
            sphere_list.action= visualization_msgs::Marker::ADD;
            sphere_list.pose.orientation.w= 1.0;
            sphere_list.id = 0;
            sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;
            sphere_list.scale.x = 0.005;
            sphere_list.scale.y =0.005;
            sphere_list.scale.z = 0.005;
            sphere_list.color.r = 1.0f;
            sphere_list.color.a = 1.0;
            visualization_msgs::Marker sphere_listLED;
            sphere_listLED.header.frame_id= "/ar_marker_3";
            sphere_listLED.header.stamp=transform.stamp_;
            sphere_listLED.ns= "spheres";
            sphere_listLED.action= visualization_msgs::Marker::ADD;
            sphere_listLED.pose.orientation.w= 1.0;
            sphere_listLED.id = 0;
            sphere_listLED.type = visualization_msgs::Marker::SPHERE_LIST;
            sphere_listLED.scale.x = 0.005;
            sphere_listLED.scale.y =0.005;
            sphere_listLED.scale.z = 0.005;
            sphere_listLED.color.b = 1.0f;
            sphere_listLED.color.a = 1.0;
            for(int i = 0; i < frontsVec.size(); i++ ){
                geometry_msgs::Point p;
                p.x =  frontsVec[i].getX();
                p.y =  frontsVec[i].getY();
                p.z =  frontsVec[i].getZ();
                sphere_list.points.push_back(p);
            }
            for(int i = 0; i < ledsVec.size(); i++ ){
                geometry_msgs::Point p;
                p.x =  ledsVec[i].getX();
                p.y =  ledsVec[i].getY();
                p.z =  ledsVec[i].getZ();
                sphere_listLED.points.push_back(p);
            }
            marker_pub.publish(sphere_list);
            marker_pubLED.publish(sphere_listLED);
            good ++;

            //pair up the found markers. If the points are 13.6mm away from each other then they are likely robots.
            float separation = 0.0136;
            float allowedErr = 0.004;
            std::vector<tf::StampedTransform> potentialRobots;
            // potential LEDs are rare, so loop through those.
            for(int i = 0; i < ledsVec.size(); i ++){
                for(int j =0; j < frontsVec.size(); j++){
                    float measured = ledsVec[i].distance(frontsVec[j]);
                    if(measured < separation+allowedErr && measured > separation - allowedErr){
                        //This is likely a robot, publish a transform, disambiguation will be done elsewhere.
                        tf::Vector3 middleVec = frontsVec[j] - ledsVec[i];
                        tf::StampedTransform robot;
                        robot.setOrigin(ledsVec[i]+ 5.57/1000* middleVec.normalized());
                        robot.stamp_ = transform.stamp_;
                        robot.frame_id_ = "ar_marker_3";
                        std::stringstream ss;
                        ss << "unnamed" << i;
                        robot.child_frame_id_ = ss.str();
                        float angle = findAngle(tf::Vector3(1,0,0),middleVec);
                        //if(middleVec.dot(tf::Vector3(1,0,0))< 0)angle =  (3.1415926/2)+ angle;
                        robot.setRotation(tf::Quaternion(tf::Vector3(0,0,1),angle));
                        potentialRobots.push_back(robot);
                        //brPtr->sendTransform(robot);
                    }
                }
            }
            if(initialising && potentialRobots.size() >= numberRobots){
                initialising = 0;
                for(int i = 0; i < numberRobots; i++){
                    trackingList[i].lastMeasuredPos = potentialRobots[i];
                    trackingList[i].guessPos = potentialRobots[i];
                    trackingList[i].isTracked = true;
                }
            }
            if(!initialising){
                //now we have a list of potential robots, we should now allocate them to the conceptual robots, tracked ones first.
                for(int i = 0; i < numberRobots; i ++){
                    if(trackingList[i].isTracked==true){
                        float dist = 10000;
                        int best = 10000;
                        for(int j = 0 ; j < potentialRobots.size(); j++){
                            float thisDist = potentialRobots[j].getOrigin().distance(trackingList[i].guessPos.getOrigin()) + 0.01*potentialRobots[j].getRotation().angle(trackingList[i].guessPos.getRotation());
                            if(thisDist < dist){
                                dist = thisDist;
                                best = j;
                            }
                        }
                        if(dist < trackingErrLinear+ 0.01*trackingErrAng){
                            trackingList[i].lastMeasuredPos = potentialRobots[best];
                            trackingList[i].isTracked = true;
                            trackingList[i].guessPos = potentialRobots[best]; // only for testing, this should be updated with values based on commands given.
                            potentialRobots.erase(potentialRobots.begin() + best);
                        }
                    }
                }
                //left over can be used for finding lost robots.
                for(int i = 0; i < numberRobots; i ++){
                    if(trackingList[i].isTracked==false){
                        float dist = 10000;
                        int best = 10000;
                        for(int j = 0 ; j < potentialRobots.size(); j++){
                            float thisDist = potentialRobots[j].getOrigin().distance(trackingList[i].guessPos.getOrigin()) + 0.01*potentialRobots[j].getRotation().angle(trackingList[i].guessPos.getRotation());
                            if(thisDist < dist){
                                dist = thisDist;
                                best = j;
                            }
                        }
                        if(dist < trackingErrLinear+ 0.01*trackingErrAng){
                            trackingList[i].lastMeasuredPos = potentialRobots[best];
                            trackingList[i].isTracked = true;
                            trackingList[i].guessPos = potentialRobots[best]; // only for testing, this should be updated with values based on commands given.
                            potentialRobots.erase(potentialRobots.begin() + best);
                        }
                    }
                }
            }

        }
        else{
            bad++;
        }


    }



    //pair up red and blue
    frontsVec.clear();
    ledsVec.clear();
    //construct transforms for pairs

    //publish transforms
    if(!initialising){
        for(int i =0; i < trackingList.size(); i ++){
            std::stringstream ss;
            ss << "robot" << i;
            trackingList[i].guessPos.child_frame_id_ = ss.str();
            trackingList[i].guessPos.frame_id_= "ar_marker_3";
            brPtr->sendTransform(trackingList[i].guessPos);
        }
    }


    //let system know we are done with the image.
    std_msgs::Time finTime;
    finTime.data = im->header.stamp;
    finishedPub.publish(finTime);
}

//void imageCalibCB(sensor_msgs::CameraInfoConstPtr info){
//    storedInfo = *info;
//}

float findAngle(tf::Vector3 one, tf::Vector3 two){
    float inner = one.angle(two);
    float det = one.getX()*two.getY() - one.getY()*two.getX();
    if(det > 0){
        return inner;
    }
    else{
        return 3.1415926 *2 - inner;
    }
}

tf::Vector3 cvToVec(cv::Point3d in){
    tf::Vector3 out;
    out.setX(in.x);
    out.setY(in.y);
    out.setZ(in.z);
    return out;
}


int main(int argc, char** argv) {
    for(int i = 0; i < numberRobots; i ++){
        robotTrackingInfo info;
        info.VisualID = i ;
        info.isTracked = 0;
        trackingList.push_back(info);
    }
    ros::init(argc, argv, "camera_location");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    image_transport::Subscriber imSub = it_.subscribe("/limited",1, imageCB);
    listenerPtr = new tf::TransformListener();
    brPtr = new tf::TransformBroadcaster();
    ros::Subscriber calibSub = node.subscribe("/camera/camera_info",1,imageInfoCB);
    finishedPub = node.advertise<std_msgs::Time>("/procFinished",1);
    marker_pub = node.advertise<visualization_msgs::Marker>("fronts", 1);
    marker_pubLED = node.advertise<visualization_msgs::Marker>("LEDs", 1);

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
