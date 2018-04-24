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
#include <tf/tf.h>
#include <iostream>
#include <stdio.h>

struct robot{
    tf::Vector3 position;
    float theta;
    int ID;
    geometry_msgs::Twist command;

    //behavior in goTo
    float angleGain = 1;
    float posGain = 1;
};


using namespace std;
using namespace cv;



void poseCB(const geometry_msgs::Pose2DConstPtr msg, int robotNum);
void sendCommands(vector<robot> robots, vector<ros::Publisher> pubs);
bool goTo(robot &thisRobot, tf::Point point, float eps);
tf::Point randomPoint(float arenaSize, float offset);
int expectedRobots = 1;
vector<robot> ourRobots;
int main( int argc, char** argv )
{
    if(argc>1){
        cout << atoi(argv[1])<< "robots" << endl;
        expectedRobots = atoi(argv[1]);
    }

    //setup ros
    ros::init(argc, argv, "userland");
    ros::NodeHandle node;
    vector<ros::Subscriber> robotPosSubs(expectedRobots);
    vector<ros::Publisher> robotCmdPubs(expectedRobots);


    // init robots, and setup the topics to recieve and send information about them.
    for(int i = 0 ; i < expectedRobots; i++){
        robot newRobot;
        newRobot.ID = i;
        ourRobots.push_back(newRobot);
        stringstream ssPos,ssCmd;
        ssPos << "/robot" << i << "/pose";
        ssCmd << "/rob" << i;

        robotPosSubs[i] = node.subscribe<geometry_msgs::Pose2D>(ssPos.str(),1,boost::bind(poseCB,_1,i));
        robotCmdPubs[i] = node.advertise<geometry_msgs::Twist>(ssCmd.str(),1);
    }

    //the loop rate does not need to be particularly fast, the robots will not recieve much faster than 10Hz
    ros::Rate rate(10);

    tf::Point currentPoint(0.15,0.15,0);
    while(ros::ok()){
        ros::spinOnce();
        //////// user loop begins here !!!!
        /// read the location of the robots from ourRobots[robot_number].position
        /// angle from ourRobots[robot_number].position
        /// to set a command put value in ourRobots[robot_number].command

        //simple angle faceing demo.
//        if(ourRobots[0].theta > 0){
//            ourRobots[0].command.angular.z = 0.5;
//        }
//        else{
//            ourRobots[0].command.angular.z = -0.5;
//        }
        bool arrived = goTo(ourRobots[0], currentPoint, 0.01);
        if(arrived) currentPoint = randomPoint(0.2,0.04);




        // send the commands that were calclulated
        sendCommands(ourRobots,robotCmdPubs);
        rate.sleep();
    }


}


//sets appropriate speeds to get to location, returns true if arrived;
float PI = 3.14159265359;
bool goTo(robot &thisRobot, tf::Point point, float eps){
    tf::Vector3 vec = point - thisRobot.position;

    float angle = atan2(vec.getY(),vec.getX())- thisRobot.theta;
    if(angle < -PI){
        angle =  angle + 2* PI;
    }
    if(angle > PI) {
        angle = angle - 2* PI;
    }
    cout << "angle" << angle << endl;
    thisRobot.command.angular.z = angle*thisRobot.angleGain;
    float speed = 0;
    float dist = vec.length();
    if(fabs(angle) < PI/2){
        speed =dist*thisRobot.posGain;
    }
    thisRobot.command.linear.x = speed;
    if(dist < eps)return true;
    return false;
}

tf::Point randomPoint(float arenaSize, float offset){
    tf::Point out;
    out.setX(offset +arenaSize* (float)rand()/RAND_MAX);
    out.setY(offset + arenaSize* (float)rand()/RAND_MAX);
    return out;
}

float maxSpeed = 0.1;
float maxRot = 3.14;
void sendCommands(vector<robot> robots, vector<ros::Publisher> pubs){
    for(int i =0; i < robots.size(); i++){
        geometry_msgs::Twist thisCmd =robots[i].command;
        thisCmd.linear.y = 0;
        thisCmd.linear.z = 0;
        thisCmd.angular.x = 0;
        thisCmd.angular.y = 0;
        if(thisCmd.linear.x > maxSpeed)thisCmd.linear.x  = maxSpeed;
        if(thisCmd.linear.x < -maxSpeed)thisCmd.linear.x  = -maxSpeed;
        if(thisCmd.angular.z > maxRot)thisCmd.angular.z = maxRot;
        if(thisCmd.angular.z < -maxRot)thisCmd.angular.z = -maxRot;
        pubs[i].publish(thisCmd);
    }
}

void poseCB(const geometry_msgs::Pose2DConstPtr msg, int robotNum){
    ourRobots[robotNum].position.setX(msg->x);
    ourRobots[robotNum].position.setY(msg->y);
    ourRobots[robotNum].theta = msg->theta;
}
