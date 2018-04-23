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

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


float IIR = 0.02;
float PosIIR = 0.1;
float findTransform(std::vector<std::pair<cv::Mat, cv::Mat>> CP, cv::Mat &R,
                    cv::Mat &t) {
    cv::Mat ac = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
    cv::Mat bc = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
    int samples =CP.size();
    for (int i = 0; i < samples; i++) {
        ac += CP.at(i).first;
        bc += CP.at(i).second;
    }
    ac /= samples;
    bc /= samples;

    cv::Mat H = cv::Mat(3, 3, CV_32F, cv::Scalar(0));

    for (int i = 0; i < samples; i++) {
        cv::Mat bT;
        cv::transpose(CP.at(i).second - bc, bT);
        H += (CP.at(i).first - ac) * (bT);
    }

    cv::SVD svd(H);
    cv::Mat v;
    cv::transpose(svd.vt, v);
    cv::Mat ut;
    cv::transpose(svd.u, ut);
    do{
        R = v * ut;
        // std::cout << cv::determinant(v)<< std::endl;
        t = -R * ac + bc;

        if(cv::determinant(R) < 0){
            v.at<float>(0,2) = -v.at<float>(0,2);
            v.at<float>(1,2) = -v.at<float>(1,2);
            v.at<float>(2,2) = -v.at<float>(2,2);
        }
    }
    while(cv::determinant(R) < 0);

    float val = 0;
    for (int i = 0; i < samples; i++) {
        cv::Mat thePoint = (R * CP.at(i).first) + t;
        cv::Mat er = thePoint - CP.at(i).second;
        float thisVal = sqrt(pow(er.at<float>(0), 2) + pow(er.at<float>(1), 2) +
                             pow(er.at<float>(2), 2));
        // std::cout<< i << "  :" << thePoint << std::endl;
        val += thisVal;
    }
    val /= samples;
    // std::cout <<"averageVal :"<< val << std::endl;
    return val;
}

//massage data into correct format for the SVD method above
float getTransform(std::vector<Point2f> scene,std::vector<Point2f> obj,Mat &r,Mat &t){
    std::vector<std::pair<cv::Mat,cv::Mat>> pairedPoints;
    for(int i =0; i < scene.size(); i++){
        cv::Mat pointScene = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
        cv::Mat pointObj = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
        pointScene.at<float>(0) =scene[i].x;
        pointScene.at<float>(1) =scene[i].y;
        pointScene.at<float>(2) =0;
        pointObj.at<float>(0) =obj[i].x;
        pointObj.at<float>(1) =obj[i].y;
        pointObj.at<float>(2) =0;
        std::pair<cv::Mat,cv::Mat> ourPair;
        ourPair.first = pointScene;
        ourPair.second = pointObj;
        pairedPoints.push_back(ourPair);

    }
    float error = findTransform(pairedPoints,r,t);
    return error;
}

void transformRT(std::vector<Point2f> inList,std::vector<Point2f> &outList, Mat r, Mat t){
    for(int i = 0 ; i < inList.size(); i++){
        cv::Mat matPoint = cv::Mat(3, 1, CV_32F, cv::Scalar(0));
        matPoint.at<float>(0) =inList[i].x;
        matPoint.at<float>(1) =inList[i].y;
        matPoint.at<float>(2) =0;
        matPoint = (r*matPoint) + t;
        Point2f outPoint;
        outPoint.x = matPoint.at<float>(0);
        outPoint.y = matPoint.at<float>(1);
        outList.push_back(outPoint);
    }
    // cout << "r:" << r << "  t:" << t << endl;
    return;
}

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
    loc avLocation;
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
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
float getDistance(robot existing, loc potential);
vector<Point2f> findCornersThreshold(Mat image);
/** Global variables */
String robotCascadeName = "/home/josh/urobotWS/src/locationCamera/trained/cascade.xml";
CascadeClassifier robot_cascade;

String window_name = "Capture - robot detection";

/** @function main */

int expectedRobotNum = 2;
bool orderRobots = 0;
int robotOrderCounter  =0;
bool isClicked = false;
vector<Point2f> clickedLocation;
Mat frame;
bool newFrame = 0;
void imageCB(sensor_msgs::ImageConstPtr im){
    //get the image from the message
    cv_bridge::toCvShare(im, "bgr8")->image.copyTo(frame);
    newestFrame = im->header.stamp;
    newFrame = 1;

}
int threshold_value = 100;
void barCB(int, void* ){

}

geometry_msgs::Point convertPoint(Point2f in){
    geometry_msgs::Point out;
    out.x = in.x;
    out.y = in.y;
    out.z = 0;
    return out;
}

void publishRobots (std::vector<robot> robToPub, std::vector<ros::Publisher> &publishers ){
    for(int i = 0 ; i < robToPub.size(); i++){
        robot_msgs::rawRobot thisBot;

        thisBot.forwardVec = convertPoint(robToPub[i].location.forwardVec);
        thisBot.location = convertPoint(robToPub[i].location.postion);
        thisBot.robotID = robToPub[i].globalID;
        thisBot.header.stamp = robToPub[i].timeSincePos;
        thisBot.lastRotLock = robToPub[i].timeSinceRot;
        publishers[robToPub[i].globalID].publish(thisBot);
    }
}
int main( int argc, char** argv )
{
    ros::init(argc, argv, "matcher");
    ros::NodeHandle node;
    image_transport::ImageTransport it_(node);
    image_transport::Subscriber imSub = it_.subscribe("/camera/image",1, imageCB);
    char* window_name = "TrexhodControl";
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
    createTrackbar( window_name,
                    window_name, &threshold_value,
                    255,barCB);
    barCB(0,0);

    VideoCapture capture;
    //Mat frame;
    std::vector<Mat> templateList;
    string path = ros::package::getPath("locationCamera");
    stringstream ss;
//    for(int i = 0; i < 1; i++){
//        ss << path << "/images/angular/0-" <<i<<".png";
//        cout <<ss.str();
//        Mat robotTemplate = imread( ss.str(), IMREAD_GRAYSCALE );
//        imshow(path,robotTemplate);
//        templateList.push_back(robotTemplate);
//        ss.str("");
//    }
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

    vector<ros::Publisher> publishers(expectedRobotNum);
    for(int i =0; i < expectedRobotNum; i++){

        ss << "/robot"<< i <<"/rawPositionData";
        string str= ss.str();
        ros::Publisher thisPub =  node.advertise<robot_msgs::rawRobot>(str,1);
        ss.str("");
        publishers[i] = thisPub;
        robot newRob;
        newRob.globalID = i;
        globRobots.push_back(newRob);
    }

    //Mat robotTemplate = imread( "/home/josh/urobot_ws/src/locationCamera/images/temp.png", CV_LOAD_IMAGE_GRAYSCALE );

    //-- 1. Load the cascades
    if( !robot_cascade.load( robotCascadeName ) ){ printf("--(!)Error loading face cascade\n"); return -1; };


    //-- 2. Read the video stream
    //    capture.open( 1 );
    //    if ( ! capture.isOpened() ) { printf("--(!)Error opening video capture\n"); return -1; }
    int count = 0;
    bool debug = true;
    namedWindow("robots", 1);
    setMouseCallback("robots", CallBackFunc, NULL);
    while(frame.rows ==0){
        ros::spinOnce();
    }
    while (1)
    {
        ros::spinOnce();
        if(!newFrame)continue;
        newFrame = 0;
        //newestFrame = ros::Time::now();
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
        publishRobots(globRobots, publishers);
        if(debug){
            if(listOfRobots.size() > 0 ){
                for(int i = 0; i < listOfRobots.size(); i++){
                   // line( frame, listOfRobots[i].postion , listOfRobots[i].postion + listOfRobots[i] , Scalar(0, 0, 255), 4 );
                }
            }
            for(int i = 0; i < globRobots.size(); i++){

                stringstream number;
                number << "ID:"<<globRobots[i].globalID<<"Pos:"<<globRobots[i].listTimePos.size()<<"Rot:"<<globRobots[i].listTimeRot.size();
                line( frame, globRobots[i].avLocation.postion , globRobots[i].avLocation.postion + globRobots[i].avLocation.forwardVec , Scalar(0, 0, 255), 4 );
                putText(frame, number.str(), globRobots[i].avLocation.postion,
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
        //cout << globRobots[0].listTimePos.size() << endl;
        int c = waitKey(1);
        if( (char)c == ' ' ) { debug= !debug; } // escape
        if( (char)c == 27 ) { break; } // escape
        if( (char)c == 'c') {
            orderRobots = 1;
        }
    }
    return 0;
}

void reallocateRobots(vector<Point2f> points, vector<robot> &ourRobots){
    //robots.clear();
    for(int i = 0; i < ourRobots.size(); i++){
        //get the robot closest to each click

        int bestPick = 0;
        float closestDist =10000000;
        for(int j = 0; j < points.size();j++){
            loc newLock;
            newLock.postion = points[j];
            float dist = getDistance(ourRobots[i],newLock);
            if(dist< closestDist){
                bestPick = j;
                closestDist = dist;
            }
        }
        ourRobots[i].globalID = bestPick;

    }


}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
         if(orderRobots){
         isClicked = true;
         clickedLocation.push_back(Point2f(x,y));
         robotOrderCounter++;
         if(robotOrderCounter == expectedRobotNum){
             orderRobots = false;
             robotOrderCounter = 0;
             reallocateRobots(clickedLocation,globRobots);
             clickedLocation.clear();
         }
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     }

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
    toUpdate.avLocation.postion = (1-PosIIR)* toUpdate.avLocation.postion + PosIIR*value.postion;
    if(value.validDir == true) {
        toUpdate.timeSinceRot = newestFrame;
        toUpdate.listTimeRot.push_back(newestFrame);
        tidyTimes(toUpdate.listTimeRot);
        toUpdate.location.forwardVec = value.forwardVec;
        toUpdate.avLocation.forwardVec = (1-IIR) *toUpdate.avLocation.forwardVec + IIR*value.forwardVec;

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
    glob.avLocation = local.avLocation;
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
    int minHessian = 0;
    detector->setHessianThreshold(minHessian);
    FlannBasedMatcher matcher;
    std::vector<Mat> descriptors_2(robotTemplate.size());
    std::vector<std::vector<KeyPoint>> keypoints_2(robotTemplate.size());

    for(int i = 0; i < robotTemplate.size(); i ++){
        detector->detectAndCompute( robotTemplate[i], Mat(), keypoints_2[i], descriptors_2[i] );
    }

    for( size_t i = 0; i < faces.size(); i++ )
    {
//start boxes method
//    std::vector<Point2f> vertices = findCornersThreshold(frame_gray(faces[i]));

//    for (int j = 0; j < 4; j++)
//    {
//        line(frame_gray(faces[i]), vertices[j], vertices[(j + 1) % 4], Scalar(0), 2, CV_AA);
//    }
//    }
//    imshow("boxes",frame_gray);


    //start the key point matcher
        //imshow("patch",frame_gray(faces[i]));
        //waitKey(1);
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
        int tempMatch = 0;
        Point2f forwardVecAv(0,0);
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

//            Mat img_matches;
//            drawMatches( frame_gray(faces[i]), keypoints_1, robotTemplate[i], keypoints_2[i],
//                         good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//                         vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

//            //-- Show detected matches
//            imshow( "Good Matches", img_matches );
//            cv::waitKey(1);

            std::vector<Point2f> obj;
            std::vector<Point2f> scene;
            for( size_t i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_1[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_2[templateCounter][ good_matches[i].trainIdx ].pt );
            }
            if(good_matches.size() >= 2 ){
                //cout << "goodMatches:" << good_matches.size()<< endl;
                Mat r,t;
                float error = getTransform(scene,obj,r,t);
                //Mat H = estimateRigidTransform( scene, obj, false );
                if(r.dims != 0){
                    std::vector<Point2f> scene_reproject;
                    transformRT(scene,scene_reproject,r,t);
                    //cv::transform(scene,scene_reproject,H);
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
                    std::vector<Point2f> scene_corners;
                    //cv::transform(obj_corners,scene_corners,H);
                    transformRT(obj_corners,scene_corners,r,t);
                    Point2f offset = faces[i].tl();
                    loc thisLoc;
                    thisLoc.postion = offset + (scene_corners[0] + scene_corners[2])/2 ;
                    thisLoc.forwardVec = -scene_corners[2] + scene_corners[1];
//                    if(prevBestMatches == 1e9 && totError < 100){
//                        prevBestMatches = totError;
//                        thisLoc.validDir = true;
//                        listOfRobotPos[robotCount] = thisLoc;
//                        tempMatch++;

//                    }
 //                   else{
                        if((totError < prevBestMatches && robotCount >= 0 && totError < 100)||( prevBestMatches == 1e9 && totError < 100)){
                            thisLoc.validDir = true;
                            listOfRobotPos[robotCount] = thisLoc;
                            forwardVecAv += thisLoc.forwardVec;
                            prevBestMatches = totError;
                            tempMatch++;
                        }
                        else{
                            thisLoc.validDir = true;
                            //thisLoc.forwardVec += Point2f(0,0);
                            listOfRobotPos[robotCount] = thisLoc;
                            //tempMatch++;
                            //prevBestMatches = totError;
                        }
 //                   }
                   // good_matches.size();
                    //-- Draw lines between the corners (the mapped object in the scene - image_2 )
//                                  line( frame, scene_corners[0] + offset , scene_corners[1]+ offset , Scalar(0, 0, 255), 4 );
//                                  line( frame, scene_corners[1] + offset, scene_corners[2] + offset, Scalar( 0, 255, 0), 4 );
//                                  line( frame, scene_corners[2]+ offset, scene_corners[3]+ offset , Scalar( 0, 255, 0), 4 );
//                                  line( frame, scene_corners[3] + offset, scene_corners[0]+ offset, Scalar( 0, 255, 0), 4 );
                    //-- Show detected matches
                   // imshow( "Good Matches & Object detection", img_matches );
                }
            }
        }
        listOfRobotPos[i].forwardVec = forwardVecAv;
        cout <<"perframe matches"<< tempMatch << endl;

    }
    //-- Show what you got
    //imshow( window_name, frame );
    return listOfRobotPos;
}

vector<Point2f> findCornersThreshold(Mat image){
    vector<cv::Point2f> vertices(4);
    int erosion_size = 1;
    Mat element = getStructuringElement( 0,
                                         Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                         Point( erosion_size, erosion_size ) );
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    Mat dst;
    threshold( image, dst, threshold_value, 255,1 );
    imshow("thresh",dst);
    //erode(dst,dst,element);
    erode(dst,dst,element);
    //    /erode(dst,dst,element);
    //dilate(dst,dst,element);
    dilate(dst,dst,element);
    dilate(dst,dst,element);
    erode(dst,dst,element);
    imshow("erode", dst);

    // findContours( dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
    RotatedRect bestR;
    vector<Point2f> pixels;
    for(int j = 0; j < dst.rows; j++){
        for(int k = 0; k < dst.cols; ++k){
            if(dst.at<char>(j,k) != 0){
                Point2f thisPoint(k,j);
                pixels.push_back(thisPoint);
            }
        }
    }
    if(pixels.size() >2){
        bestR = minAreaRect(pixels);
        cv::Point2f vertices2f[4];
        bestR.points(vertices2f);
        // Convert them so we can use them in a fillConvexPoly

        for(int k = 0; k < 4; ++k){
            vertices[k] = vertices2f[k];
        }
    }
    return vertices;
}

