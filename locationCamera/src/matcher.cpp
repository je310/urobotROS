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


#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

cv::Point2f operator*(cv::Mat M, const cv::Point2f& p)
{
    cv::Mat_<double> src(3/*rows*/,1 /* cols */);

    src(0,0)=p.x;
    src(1,0)=p.y;
    src(2,0)=1.0;

    cv::Mat_<double> dst = M*src; //USE MATRIX ALGEBRA
    return cv::Point2f(dst(0,0),dst(1,0));
}

struct loc{
    Point2f postion;
    Point2f forwardVec;
    bool validDir = false;
};

struct robot{
    loc location;
    ros::Time timeSincePos;
    ros::Time timeSinceRot;
    loc avLocaiton;
    vector<ros::Time> listTimePos;
    vector<ros::Time> listTimeRot;
    float posConf = 0;
    float rotConf = 0;
    int localID = -1;
    int globalID= -1;
};

int localIDcount = 0;
ros::Time newestFrame;
std::vector<robot> robots;
std::vector<robot> globRobots;

/** Function Headers */
std::vector<loc> detectAndDisplay(Mat frame , std::vector<Mat> robotTemplate);
void correspondRobots(std::vector<robot> &existingRobots,std::vector<loc> newRobots,int horizon);
void correspondGlobalRobots(std::vector<robot> &globalRobots,std::vector<robot> localRobots);
void clearRobots(vector<robot> &theseRobots, ros::Duration time);
/** Global variables */
String robotCascadeName = "/home/josh/urobot_ws/src/locationCamera/trained/cascade.xml";
CascadeClassifier robot_cascade;

String window_name = "Capture - robot detection";

/** @function main */

int expectedRobotNum = 3;

int main( int argc, char** argv )
{
    ros::init(argc, argv, "matcher");
    ros::NodeHandle node;
    VideoCapture capture;
    Mat frame;
    std::vector<Mat> templateList;
    string path = ros::package::getPath("locationCamera");
    stringstream ss;
    ss << path << "/images/temp.png";
    Mat robotTemplate = imread( ss.str(), IMREAD_GRAYSCALE );
    templateList.push_back(robotTemplate);
    ss.str("");
    ss << path << "/images/temp1.png";
    robotTemplate = imread( ss.str(), IMREAD_GRAYSCALE );
    templateList.push_back(robotTemplate);
    ss.str("");
    ss << path << "/images/temp2.png";
    robotTemplate = imread( ss.str(), IMREAD_GRAYSCALE );
    templateList.push_back(robotTemplate);
    ss.str("");
    ss << path << "/images/temp3.png";
    robotTemplate = imread( ss.str(), IMREAD_GRAYSCALE );
    templateList.push_back(robotTemplate);
    ss.str("");


    for(int i =0; i < expectedRobotNum; i++){
        robot newRob;
        newRob.globalID = i;
        globRobots.push_back(newRob);
    }

    //Mat robotTemplate = imread( "/home/josh/urobot_ws/src/locationCamera/images/temp.png", CV_LOAD_IMAGE_GRAYSCALE );

    //-- 1. Load the cascades
    if( !robot_cascade.load( robotCascadeName ) ){ printf("--(!)Error loading face cascade\n"); return -1; };


    //-- 2. Read the video stream
    capture.open( -1 );
    if ( ! capture.isOpened() ) { printf("--(!)Error opening video capture\n"); return -1; }
    int count = 0;
    bool debug = true;
    while (  capture.read(frame) )
    {
        newestFrame = ros::Time::now();
        count ++;
        if(count %90 == 0) printf("100\n");
        if( frame.empty() )
        {
            printf(" --(!) No captured frame -- Break!");
            break;
        }

        //-- 3. Apply the classifier to the frame

        std::vector<loc> listOfRobots = detectAndDisplay( frame , templateList);


        correspondRobots(robots,listOfRobots,4000);
        clearRobots(robots,ros::Duration(0.5));
        correspondGlobalRobots(globRobots,robots);
        if(debug){
            if(listOfRobots.size() > 0 ){
                for(int i = 0; i < listOfRobots.size(); i++){
                    line( frame, listOfRobots[i].postion , listOfRobots[i].postion + listOfRobots[i].forwardVec , Scalar(0, 0, 255), 4 );
                }
            }
            for(int i = 0; i < globRobots.size(); i++){

                stringstream number;
                number << "ID:"<<globRobots[i].globalID<<"Pos:"<<globRobots[i].listTimePos.size()<<"Rot:"<<globRobots[i].listTimeRot.size();

                putText(frame, number.str(), globRobots[i].location.postion,
                        FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
                if(newestFrame - globRobots[i].timeSinceRot < ros::Duration(0.15)){
                    Point2f ledLoc;
                    float scale = 0.2;
                    float ledSize = 35;
                    ledLoc = globRobots[i].location.postion - scale* globRobots[i].location.forwardVec;
                    circle(frame, ledLoc, scale*ledSize,cvScalar(200,0,250) );
                }
            }

            imshow("robots",frame);

        }
        cout << globRobots[0].listTimePos.size() << endl;
        int c = waitKey(1);
        if( (char)c == ' ' ) { debug= !debug; } // escape
        if( (char)c == 27 ) { break; } // escape
    }
    return 0;
}

void clearRobots(vector<robot> &theseRobots, ros::Duration time){
    for(int i = 0 ; i < theseRobots.size(); i++){
        ros::Duration diff = ros::Time::now() - theseRobots[i].timeSincePos;
        if(ros::Time::now() - theseRobots[i].timeSincePos > time ){
            theseRobots.erase(theseRobots.begin() + i);
            i = -1;
        }
    }
}

float getDistance(robot existing, loc potential){
    if(existing.localID == -1) return 1e8;
    Point2f distance = existing.location.postion - potential.postion;
    return pow(distance.x,2) + pow(distance.y,2);
}

bool complete(Mat scoreMat, float horizon){
    if(scoreMat.cols == 0 || scoreMat.rows == 0) return true;
    for(int i = 0; i < scoreMat.cols; i++){
        for(int j= 0; j < scoreMat.rows; j++){
            float score = scoreMat.at<float>(j,i) ;
            if(score < horizon){

                return false;
            }
        }
    }
    return true;
}

bool completeLoose(Mat scoreMat, float horizon){
    if(scoreMat.cols == 0 || scoreMat.rows == 0) return true;
    for(int i = 0; i < scoreMat.cols; i++){
        for(int j= 0; j < scoreMat.rows; j++){
            float current = scoreMat.at<float>(j,i) ;
            if(current == 1e9 || current == 1e8 ||current <= 1e8 ) return false;
        }
    }
    return true;
}

std::pair<int,int> getNextBest(Mat &scoreMat){
    std::pair<int,int> match;
    cv::Point min_loc, max_loc;
    double min, max;
    cv::minMaxLoc(scoreMat, &min, &max, &min_loc, &max_loc);
    match.first = min_loc.x;
    match.second = min_loc.y;
    for(int i = 0; i < scoreMat.cols; i ++ ){
        scoreMat.at<float>(min_loc.y,i) = 1e10;
    }
    for(int i = 0; i < scoreMat.rows; i ++ ){
        scoreMat.at<float>(i,min_loc.x) = 1e10;
    }
    //cout << scoreMat << endl;
    return match;

}

void tidyTimes(vector<ros::Time> &list){
    for(int i = 0 ; i < list.size(); i++){
        ros::Duration diff = ros::Time::now() - list[i];
        if(diff > ros::Duration(3.0) ){
            list.erase(list.begin() + i);
            i = -1;
        }
    }
}

void updateRobot(robot &toUpdate, loc value){
    toUpdate.timeSincePos = newestFrame;
    toUpdate.listTimePos.push_back(newestFrame);
    tidyTimes(toUpdate.listTimePos);
    toUpdate.location.postion = value.postion;
    if(value.validDir == true) {
        toUpdate.timeSinceRot = newestFrame;
        toUpdate.listTimeRot.push_back(newestFrame);
        tidyTimes(toUpdate.listTimeRot);
        toUpdate.location.forwardVec = value.forwardVec;

    }
}

void assignGlobal(robot &glob, robot local){
    glob.localID = local.localID;
    glob.location = local.location;
    glob.timeSincePos = local.timeSincePos;
    glob.timeSinceRot = local.timeSinceRot;
    glob.listTimePos = local.listTimePos;
    glob.listTimeRot = local.listTimeRot;
    glob.posConf = local.posConf;
    glob.rotConf = local.rotConf;
    tidyTimes(glob.listTimePos);
    tidyTimes(glob.listTimeRot);

}

bool allSatisfied(vector<bool> toCheck){
    for(int i = 0; i < toCheck.size(); i ++){
        if(toCheck[i]== false) return false;
    }
    return true;
}

float globGetDistance(robot glob, robot local){
    Point2f distance = glob.location.postion - local.location.postion;
    return pow(distance.x,2) + pow(distance.y,2);
}

void correspondGlobalRobots(std::vector<robot> &globalRobots,std::vector<robot> localRobots){
    //stuff for the first time.
    bool brandNew = true;
    for(int i = 0; i < globalRobots.size();i++){
        if(globalRobots[i].localID != -1){
            brandNew = false;
        }
    }
    if(brandNew && localRobots.size() == globalRobots.size()){
        for(int i = 0; i < globalRobots.size();i++){
            assignGlobal(globalRobots[i], localRobots[i]);
        }
        return;
    }
    // general case ;
    vector<bool> globalSatisfied, localSatisfied;
    for(int i = 0; i < globalRobots.size();i++){
        globalSatisfied.push_back(false);
    }
    for(int i = 0; i < localRobots.size();i++){
        localSatisfied.push_back(false);
    }
    for(int i = 0; i < globalRobots.size();i++){
        for(int f = 0; f < localRobots.size();f++){
            if (globalRobots[i].localID == localRobots[f].localID){
                assignGlobal(globalRobots[i], localRobots[f]);
                globalSatisfied[i] = true;
                localSatisfied[f] = true;
            }
        }
    }
    if(allSatisfied(globalSatisfied) ||allSatisfied(localSatisfied) ) return ;

    Mat scoreMat(globalRobots.size(),localRobots.size(),CV_32F,1e9);
    for(int existing = 0 ; existing < globalRobots.size(); existing ++){
        for(int incomming = 0; incomming < localRobots.size(); incomming ++){
            if(!globalSatisfied[existing] && !localSatisfied[incomming]){
                float score = globGetDistance(globalRobots[existing], localRobots[incomming]);

                scoreMat.at<float>(existing,incomming) = globGetDistance(globalRobots[existing], localRobots[incomming]);
            }
        }
    }

    while(!allSatisfied(globalSatisfied) &&!allSatisfied(localSatisfied) && !complete(scoreMat,1e9)){
        std::pair<int,int> lowestPair;
        lowestPair = getNextBest(scoreMat);
        assignGlobal(globalRobots[lowestPair.second], localRobots[lowestPair.first]);
        globalSatisfied[lowestPair.second] = true;
        localSatisfied[lowestPair.first] = true;
    }

}

//this function will check the new potential robots and match them with existing robots
void correspondRobots(std::vector<robot> &existingRobots,std::vector<loc> newRobots,int horizon){
    // do greedy asignment for all existing robots (as long as less than horizon.
    // any left over newRobots get new instances of robots made for them.


    //initially there is no robots at all
    if(existingRobots.size()==0 && newRobots.size() != 0){
        for(int i = 0; i < newRobots.size(); i++){
            robot newRobot;

            newRobot.localID= localIDcount;
            localIDcount++;
            updateRobot(newRobot,newRobots[i]);
            existingRobots.push_back(newRobot);
        }
    }
    vector<bool> existingSatisfied, newSatisfied;
    for(int i = 0; i < existingRobots.size();i++){
        existingSatisfied.push_back(false);
    }
    for(int i = 0; i < newRobots.size();i++){
        newSatisfied.push_back(false);
    }
    Mat scoreMat(existingRobots.size(),newRobots.size(),CV_32F,1e9); // this will hold a complete score matrix
    for(int existing = 0 ; existing < existingRobots.size(); existing ++){
        for(int incomming = 0; incomming < newRobots.size(); incomming ++){
            float score = getDistance(existingRobots[existing], newRobots[incomming]);
            scoreMat.at<float>(existing,incomming) = getDistance(existingRobots[existing], newRobots[incomming]);
        }
    }
    //if(scoreMat.cols > 0)cout << scoreMat<< endl<<endl;
    while(!complete(scoreMat,horizon)){
        std::pair<int,int> lowestPair;
        lowestPair = getNextBest(scoreMat);

        updateRobot(existingRobots[lowestPair.second],newRobots[lowestPair.first]);
        existingSatisfied[lowestPair.second] = true;
        newSatisfied[lowestPair.first] = true;
    }
    while(!completeLoose(scoreMat,horizon)){
        std::pair<int,int> lowestPair;
        lowestPair = getNextBest(scoreMat);
        robot newRobot;
        newRobot.localID= localIDcount;
        localIDcount++;
        updateRobot(newRobot,newRobots[lowestPair.first]);
        existingSatisfied[lowestPair.second] = true;
        newSatisfied[lowestPair.first] = true;
        existingRobots.push_back(newRobot);
    }
    while(!allSatisfied(newSatisfied)){
        for(int i = 0; i < newRobots.size(); i++){
            if(newSatisfied[i] == false){
                robot newRobot;
                newRobot.localID= localIDcount;
                localIDcount++;
                updateRobot(newRobot,newRobots[i]);
                newSatisfied[i] = true;
                existingRobots.push_back(newRobot);
            }
        }
    }

}

/** @function detectAndDisplay */
std::vector<loc> detectAndDisplay( Mat frame , std::vector<Mat> robotTemplate)
{
    std::vector<loc> listOfRobotPos;
    std::vector<Rect> faces;
    Mat frame_gray;

    cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
    //equalizeHist( frame_gray, frame_gray );

    //-- Detect faces
    robot_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CASCADE_SCALE_IMAGE, Size(30, 30) );
    int robotCount = -1;
    Ptr<SURF> detector = SURF::create();
    int minHessian = 400;
    detector->setHessianThreshold(minHessian);
    FlannBasedMatcher matcher;
    std::vector<Mat> descriptors_2(robotTemplate.size());
    std::vector<std::vector<KeyPoint>> keypoints_2(robotTemplate.size());

    for(int i = 0; i < robotTemplate.size(); i ++){
        detector->detectAndCompute( robotTemplate[i], Mat(), keypoints_2[i], descriptors_2[i] );
    }

    for( size_t i = 0; i < faces.size(); i++ )
    {
        robotCount++ ;
        loc justPosRobot;
        justPosRobot.postion = faces[i].tl() +0.5*(faces[i].br() -  faces[i].tl() );
        listOfRobotPos.push_back(justPosRobot);
        Mat faceROI = frame_gray( faces[i] );
        std::vector<Rect> eyes;
        float prevBestMatches = 1e9;



        std::vector<KeyPoint> keypoints_1;
        Mat descriptors_1;
        detector->detectAndCompute( frame_gray(faces[i]), Mat(), keypoints_1, descriptors_1 );
        for( int templateCounter = 0; templateCounter < robotTemplate.size(); templateCounter++){ // try a few different templates
            //-- Step 2: Matching descriptor vectors using FLANN matcher

            std::vector< DMatch > matches;
            matcher.match( descriptors_1, descriptors_2, matches );
            double max_dist = 0; double min_dist = 100;
            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < descriptors_1.rows; i++ )
            { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }

            std::vector< DMatch > good_matches;
            for( int i = 0; i < descriptors_1.rows; i++ )
            { if( matches[i].distance <= max(2*min_dist, 0.02) )
                { good_matches.push_back( matches[i]); }
            }

            std::vector<Point2f> obj;
            std::vector<Point2f> scene;
            for( size_t i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_2[templateCounter][ good_matches[i].trainIdx ].pt );
            }
            if(good_matches.size() > 2 ){

                Mat H = estimateRigidTransform( scene, obj, false );
                if(H.dims != 0){
                    std::vector<Point2f> scene_reproject;
                    cv::transform(scene,scene_reproject,H);
                    float totError = 0;
                    for(int scene_point = 0; scene_point < obj.size(); scene_point++){
                        float error = pow((obj[scene_point] - scene_reproject[scene_point]).x,2) + pow((obj[scene_point] - scene_reproject[scene_point]).y,2);
                        totError += error;
                    }
                    totError /= scene.size();
                    //-- Get the corners from the image_1 ( the object to be "detected" )
                    std::vector<Point2f> obj_corners(4);
                    obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( robotTemplate[templateCounter].cols, 0 );
                    obj_corners[2] = cvPoint( robotTemplate[templateCounter].cols, robotTemplate[templateCounter].rows ); obj_corners[3] = cvPoint( 0, robotTemplate[templateCounter].rows );
                    std::vector<Point2f> scene_corners(4);
                    cv::transform(obj_corners,scene_corners,H);
                    Point2f offset = faces[i].tl();
                    loc thisLoc;
                    thisLoc.postion = offset + (scene_corners[0] + scene_corners[2])/2 ;
                    thisLoc.forwardVec = scene_corners[2] - scene_corners[1];
                    if(prevBestMatches == 1e9 && totError < 100000){
                        prevBestMatches = totError;
                        thisLoc.validDir = true;
                        listOfRobotPos[robotCount] = thisLoc;

                    }
                    else{
                        if(totError < prevBestMatches && robotCount >= 0 && totError < 100000){
                            thisLoc.validDir = true;
                            listOfRobotPos[robotCount] = thisLoc;
                            prevBestMatches = totError;
                        }
                    }
                    //good_matches.size();
                    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
                    //              line( frame, scene_corners[0] + offset , scene_corners[1]+ offset , Scalar(0, 0, 255), 4 );
                    //              line( frame, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
                    //              line( frame, scene_corners[2]+ offset, scene_corners[3]+ offset , Scalar( 0, 255, 0), 4 );
                    //              line( frame, scene_corners[3] + offset, scene_corners[0]+ offset, Scalar( 0, 255, 0), 4 );
                    //-- Show detected matches
                    //imshow( "Good Matches & Object detection", img_matches );
                }
            }
        }


    }
    //-- Show what you got
    //imshow( window_name, frame );
    return listOfRobotPos;
}




