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
    int localID = -1;
    int globalID= -1;
};

int localIDcount = 0;

std::vector<robot> robots;

/** Function Headers */
std::vector<loc> detectAndDisplay(Mat frame , std::vector<Mat> robotTemplate);
void correspondRobots(std::vector<robot> &existingRobots,std::vector<loc> newRobots,int horizon);
void clearRobots(vector<robot> &theseRobots, ros::Duration time);
/** Global variables */
String robotCascadeName = "/home/josh/urobot_ws/src/locationCamera/trained/cascade.xml";
CascadeClassifier robot_cascade;

String window_name = "Capture - robot detection";

/** @function main */



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



    //Mat robotTemplate = imread( "/home/josh/urobot_ws/src/locationCamera/images/temp.png", CV_LOAD_IMAGE_GRAYSCALE );

    //-- 1. Load the cascades
    if( !robot_cascade.load( robotCascadeName ) ){ printf("--(!)Error loading face cascade\n"); return -1; };


    //-- 2. Read the video stream
    capture.open( -1 );
    if ( ! capture.isOpened() ) { printf("--(!)Error opening video capture\n"); return -1; }
    int count = 0;
    while (  capture.read(frame) )
    {
        count ++;
        if(count %90 == 0) printf("100\n");
        if( frame.empty() )
        {
            printf(" --(!) No captured frame -- Break!");
            break;
        }

        //-- 3. Apply the classifier to the frame

        std::vector<loc> listOfRobots = detectAndDisplay( frame , templateList);
        if(listOfRobots.size() > 0 ){
            for(int i = 0; i < listOfRobots.size(); i++){
                line( frame, listOfRobots[i].postion , listOfRobots[i].postion + listOfRobots[i].forwardVec , Scalar(0, 0, 255), 4 );
            }
        }

        correspondRobots(robots,listOfRobots,1000);
        clearRobots(robots,ros::Duration(1));
        for(int i = 0; i < robots.size(); i++){
            stringstream number;
            number << robots[i].localID;

            putText(frame, number.str(), robots[i].location.postion,
                FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
        }
        imshow("robots",frame);
        cout << robots.size() << endl;

        int c = waitKey(1);
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

void updateRobot(robot &toUpdate, loc value){
    toUpdate.timeSincePos = ros::Time::now();
    toUpdate.location.postion = value.postion;
    if(value.validDir == true) {
        toUpdate.timeSinceRot = ros::Time::now();
        toUpdate.location.forwardVec = value.forwardVec;
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
    Mat scoreMat(existingRobots.size(),newRobots.size(),CV_32F,1e9); // this will hold a complete score matrix
    for(int existing = 0 ; existing < existingRobots.size(); existing ++){
        for(int incomming = 0; incomming < newRobots.size(); incomming ++){
            float score = getDistance(existingRobots[existing], newRobots[incomming]);
            if(score > 10000){
                cout <<"hello"<<endl;
            }
            scoreMat.at<float>(existing,incomming) = getDistance(existingRobots[existing], newRobots[incomming]);
        }
    }
    if(scoreMat.cols > 0)cout << scoreMat<< endl<<endl;
    while(!complete(scoreMat,horizon)){
        std::pair<int,int> lowestPair;
        lowestPair = getNextBest(scoreMat);

        updateRobot(existingRobots[lowestPair.second],newRobots[lowestPair.first]);
    }
    while(!completeLoose(scoreMat,horizon)){
        std::pair<int,int> lowestPair;
        lowestPair = getNextBest(scoreMat);
        robot newRobot;
        newRobot.localID= localIDcount;
        localIDcount++;
        updateRobot(newRobot,newRobots[lowestPair.first]);
        existingRobots.push_back(newRobot);
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
    for( size_t i = 0; i < faces.size(); i++ )
    {
        Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
        ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
        Mat faceROI = frame_gray( faces[i] );
        std::vector<Rect> eyes;
        float prevBestMatches = 1e9;
        for( int templateCounter = 0; templateCounter < robotTemplate.size(); templateCounter++){ // try a few different templates

            int minHessian = 400;
            Ptr<SURF> detector = SURF::create();
            detector->setHessianThreshold(minHessian);
            std::vector<KeyPoint> keypoints_1, keypoints_2;
            Mat descriptors_1, descriptors_2;
            detector->detectAndCompute( frame_gray(faces[i]), Mat(), keypoints_1, descriptors_1 );
            detector->detectAndCompute( robotTemplate[templateCounter], Mat(), keypoints_2, descriptors_2 );
            //-- Step 2: Matching descriptor vectors using FLANN matcher
            FlannBasedMatcher matcher;
            std::vector< DMatch > matches;
            matcher.match( descriptors_1, descriptors_2, matches );
            double max_dist = 0; double min_dist = 100;
            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < descriptors_1.rows; i++ )
            { double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }

            //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
            //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
            //-- small)
            //-- PS.- radiusMatch can also be used here.
            std::vector< DMatch > good_matches;
            for( int i = 0; i < descriptors_1.rows; i++ )
            { if( matches[i].distance <= max(2*min_dist, 0.02) )
                { good_matches.push_back( matches[i]); }
            }
            //-- Draw only "good" matches
            Mat img_matches;
            drawMatches( frame_gray(faces[i]), keypoints_1, robotTemplate[templateCounter], keypoints_2,
                         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;
            for( size_t i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
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
                    if(prevBestMatches == 1e9 && totError < 200){
                        prevBestMatches = totError;
                        listOfRobotPos.push_back(thisLoc);
                        robotCount++ ;
                    }
                    else{
                        if(totError < prevBestMatches && robotCount >= 0 && totError < 200){
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




