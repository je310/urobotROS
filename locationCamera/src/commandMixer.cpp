#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Accel.h>
#include <robot_msgs/rawRobot.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

struct loc{
    Point2f postion;
    Point2f forwardVec;
    bool validDir = false;
};
struct robot{
    loc location;
    ros::Time timeSincePos;
    ros::Time timeSinceRot;
    loc avLocation;
    Point2f estimatedForward;
    vector<ros::Time> listTimePos;
    vector<ros::Time> listTimeRot;
    vector<pair<ros::Time,Point2f>> history;
    vector<pair<ros::Time,geometry_msgs::Twist>> cmdHistory;
    float posConf = 0;
    float rotConf = 0;
    float accumilatedDist = 0;
    int localID = -1;
    int globalID= -1;
};
cv::Point2f operator*(cv::Mat M, const cv::Point2f& p);
void dataCB(const robot_msgs::rawRobotConstPtr msg, int robotNum);
void cmdCB(const geometry_msgs::TwistConstPtr msg, int robotNum);
void updateRobots(vector<robot> &ourRobots, vector<geometry_msgs::Twist> cmds, float scale);
void estimateForwards(vector<robot> &ourRobots, float scale);
void drawRobots(vector<robot> ourRobots, Mat &simulation, float scale);
void cleanUpLists(vector<robot> &ourRobots, ros::Duration time);
void publishRobots(vector<robot> ourRobots, vector<ros::Publisher> &pubs);
float posIIR = 0.4;
float rotIIR = 0.1;
float rotIIR2 = 0.1;

int expectedRobots = 1;
vector<robot> robots(expectedRobots);
vector<geometry_msgs::Twist> currentCommands(expectedRobots);

Mat frame;
bool newFrame = 0;
void imageCB(sensor_msgs::ImageConstPtr im){
    //get the image from the message
    cv_bridge::toCvShare(im, "bgr8")->image.copyTo(frame);
    //newestFrame = im->header.stamp;
    newFrame = 1;

}
float mainScale = 4000; // pix per m
int main( int argc, char** argv )
{
    ros::init(argc, argv, "commandMixer");
    ros::NodeHandle node;

    if(argc>1){
        cout << atoi(argv[1])<< "robots" << endl;
        expectedRobots = atoi(argv[1]);
    }
    vector<ros::Subscriber> subscribers(expectedRobots);
    vector<ros::Subscriber> subscribersCmd(expectedRobots);
    vector<ros::Publisher> publishers(expectedRobots);
    for(int i = 0; i < expectedRobots; i++){
        stringstream ss,ssCmd,ssPub;
        ss << "/robot"<< i <<"/rawPositionData";
        ros::Subscriber thisSub = node.subscribe<robot_msgs::rawRobot>(ss.str(),1,boost::bind(dataCB,_1,i));
        ssCmd << "/rob"<< i <<"/";
        ros::Subscriber thisSubCmd = node.subscribe<geometry_msgs::Twist>(ssCmd.str(),1,boost::bind(cmdCB,_1,i));
        subscribers[i] = thisSub;
        subscribersCmd[i] = thisSubCmd;
        ssPub << "/robot"<<i<<"/pose";
        ros::Publisher thisPub = node.advertise<geometry_msgs::Pose2D>(ssPub.str(),1);
        publishers[i]= thisPub;
    }

    image_transport::ImageTransport it_(node);

    image_transport::Subscriber imSub = it_.subscribe("/camera_image",1, imageCB);
    Size imsize;
    imsize.width = 640;
    imsize.height = 480;

    ros::Rate rate(500);
    while(ros::ok()){
        ros::spinOnce();
        updateRobots(robots,currentCommands,mainScale);
        estimateForwards(robots,mainScale);
        cleanUpLists(robots,ros::Duration(0.5));
        //cv::Mat simulation(imsize,CV_8UC3,Scalar(0,0,0));
        if(frame.rows !=0){
        drawRobots(robots,frame, mainScale);
        line(frame,Point2i(20,440),Point2i(20+(mainScale/10),440),Scalar(0,40,55));
        imshow("simulation",frame);
        publishRobots(robots,publishers);
    }
        waitKey(1);
        rate.sleep();
    }


}
void dataCB(const robot_msgs::rawRobotConstPtr msg, int robotNum){
    Point2f thisPos,thisRot;
    thisPos.x = msg->location.x;
    thisPos.y = msg->location.y;
    thisRot.x = msg->forwardVec.x;
    thisRot.y = msg->forwardVec.y;
    //    Point2f diff = robots[robotNum].location.postion - thisPos;
    //    float dist = Norm(diff);
    //    robots[robotNum].accumilatedDist += dist;
    pair<ros::Time,Point2f> ourPair;
    ourPair.first = msg->header.stamp;
    ourPair.second = thisPos;
    robots[robotNum].history.push_back(ourPair);
    robots[robotNum].location.postion = (1-posIIR)* robots[robotNum].location.postion + posIIR * thisPos;
    robots[robotNum].estimatedForward = (1-rotIIR)* robots[robotNum].estimatedForward + rotIIR * thisRot;
}
void cmdCB(const geometry_msgs::TwistConstPtr msg, int robotNum){
    geometry_msgs::Twist thisMsg = *msg;
    pair<ros::Time,geometry_msgs::Twist> ourPair;
    ourPair.first = ros::Time::now();
    ourPair.second = thisMsg;
    currentCommands[robotNum] = thisMsg;
    robots[robotNum].cmdHistory.push_back(ourPair);
}

void publishRobots(vector<robot> ourRobots, vector<ros::Publisher> &pubs){
    for(int i = 0 ; i < ourRobots.size(); i++){
        geometry_msgs::Pose2D msg;
        msg.x = ourRobots[i].location.postion.x/mainScale;
        msg.y = (frame.rows-ourRobots[i].location.postion.y)/mainScale;
        msg.theta = atan2(-ourRobots[i].estimatedForward.y,ourRobots[i].estimatedForward.x);
        pubs[i].publish(msg);
    }
}

Point2f rotateVec(Point2f in, float angle){
    Mat rotMat(2,2,CV_64F,Scalar(0.0));
    rotMat.at<double>(0,0) = cos(angle);
    rotMat.at<double>(1,0) = -sin(angle);
    rotMat.at<double>(0,1) = sin(angle);
    rotMat.at<double>(1,1) = cos(angle);
    Point2f out = rotMat * in;
    return out;
}

float Norm(Point2f p){
    return sqrt( p.x*p.x + p.y*p.y);
}


const float PI = 3.14159265359;
void updateRobots(vector<robot> &ourRobots, vector<geometry_msgs::Twist> cmds, float scale){
    static ros::Time lastUpdate = ros::Time::now();
    ros::Time now = ros::Time::now();
    ros::Duration elapsed = now -lastUpdate;
    lastUpdate = now;
    for(int i = 0 ; i < ourRobots.size(); i++){

        if(Norm(ourRobots[i].estimatedForward)> 0){
            float AL = cmds[i].linear.x * elapsed.toSec();
            float angle = cmds[i].angular.z * elapsed.toSec();
            angle *= 0.78;
            if(angle!=0){
                float distToCentre = -scale*AL/angle;
                ourRobots[i].estimatedForward = rotateVec(ourRobots[i].estimatedForward, angle);
                Point2f sideVec = rotateVec(ourRobots[i].estimatedForward,PI/2);
                Point2f centre = ourRobots[i].location.postion - distToCentre*sideVec/Norm(sideVec);
                ourRobots[i].location.postion -= centre;
                ourRobots[i].location.postion = rotateVec(ourRobots[i].location.postion,angle);
                ourRobots[i].location.postion += centre;
            }
            else{
                if(AL !=0){
                    ourRobots[i].location.postion = ourRobots[i].location.postion + scale*AL*ourRobots[i].estimatedForward/Norm(ourRobots[i].estimatedForward);
                }
            }
        }
    }

}
cv::Point2f operator*(cv::Mat M, const cv::Point2f& p)
{
    cv::Mat_<double> src(2/*rows*/,1 /* cols */);

    src(0,0)=p.x;
    src(1,0)=p.y;
    // cout << M << endl << p<<endl;
    cv::Mat_<double> dst = M*src; //USE MATRIX ALGEBRA
    return cv::Point2f(dst(0,0),dst(1,0));
}

float robotL = 0.011;
float robotW = 0.009;
void drawRobots(vector<robot> ourRobots, Mat &simulation, float scale){
    for(int i = 0 ; i < ourRobots.size(); i++){
        Point2f forwardVec = ourRobots[i].estimatedForward/Norm(ourRobots[i].estimatedForward);
        Point2f sideVec = rotateVec(forwardVec,PI/2);
        Point2f corners[4];
        corners[0] = ourRobots[i].location.postion - scale*robotW*sideVec + scale*robotL*forwardVec;
        corners[1] = ourRobots[i].location.postion - scale*robotW*sideVec - scale*robotL*forwardVec;
        corners[2] = ourRobots[i].location.postion + scale*robotW*sideVec - scale*robotL*forwardVec;
        corners[3] = ourRobots[i].location.postion + scale*robotW*sideVec + scale*robotL*forwardVec;
        for(int j=0; j<3; j++){
            line(simulation,corners[j],corners[(j+1)%4],Scalar(0,0,255));
        }
        line(simulation,corners[3],corners[0],Scalar(0,255,255));

    }
}


void cleanUpLists(vector<robot> &ourRobots, ros::Duration time){
    for(int i = 0; i < ourRobots.size(); i++){
        for(int j = 0 ; j < ourRobots[i].history.size(); j++){
            ros::Duration diff = ros::Time::now() - ourRobots[i].history[j].first;
            if(diff > time ){
                ourRobots[i].history.erase(ourRobots[i].history.begin() + j);
                j = -1;
            }
        }
        for(int j = 0 ; j < ourRobots[i].cmdHistory.size(); j++){
            ros::Duration diff = ros::Time::now() - ourRobots[i].cmdHistory[j].first;
            if(diff > time ){
                ourRobots[i].cmdHistory.erase(ourRobots[i].cmdHistory.begin() + j);
                j = -1;
            }
        }
    }
}
float sThresh = 0.01;
int mostlyStrait(geometry_msgs::Twist in){
    if(in.angular.z ==0 && in.linear.x > 0) return 1;
    if(in.angular.z ==0 && in.linear.x < 0) return -1;
    float distToCentre = in.linear.x/in.angular.z;
    if(distToCentre > sThresh ||distToCentre < -sThresh){
        if( in.linear.x > 0) return 1;
        if(in.linear.x < 0) return -1;
    }

    return 0;
}

Point2f getPosNearestTime(robot ourRobot, ros::Time time){
    float smallestTime = 100000;
    Point2f result;
    for(int i = 0; i < ourRobot.history.size(); i++){
        if(abs((ourRobot.history[i].first-time).toSec()) < smallestTime){
            smallestTime = abs((ourRobot.history[i].first-time).toSec());
            result = ourRobot.history[i].second;
        }
    }
    return result;
}

void estimateForwards(vector<robot> &ourRobots, float scale){

    for(int i = 0 ; i < ourRobots.size(); i++){
        if(ourRobots[i].cmdHistory.size() < 3 ||ourRobots[i].history.size() < 3) return;
        geometry_msgs::Twist thisTwist = ourRobots[i].cmdHistory[ourRobots[i].cmdHistory.size() - 1].second;
        int initStrVal = mostlyStrait(thisTwist);
        if(initStrVal == 0) continue;
        ros::Time end = ourRobots[i].cmdHistory[ourRobots[i].cmdHistory.size() - 1].first;
        int j = 0;
        for(j = ourRobots[i].cmdHistory.size() - 1; j>=0 ; j--){
            int strVal = mostlyStrait(ourRobots[i].cmdHistory[j].second);
            if(mostlyStrait(ourRobots[i].cmdHistory[j].second) == initStrVal) continue;
            else break;
        }
        ros::Time start = ourRobots[i].cmdHistory[j+1].first;
        Point2f startP = getPosNearestTime(ourRobots[i],start);
        Point2f endP = getPosNearestTime(ourRobots[i],end);
        Point2f vec = endP - startP;
        if(fabs(Norm(vec))> 15){
            if(initStrVal ==1){
                ourRobots[i].estimatedForward = (1-rotIIR2)*ourRobots[i].estimatedForward + rotIIR2*vec;
            }
        if(initStrVal ==-1){
           ourRobots[i].estimatedForward  = (1-rotIIR2)*ourRobots[i].estimatedForward + rotIIR2*(-vec) ;
        }
        }


    }
}
