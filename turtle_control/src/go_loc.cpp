// Includes
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <tf/transform_broadcaster.h>

//struct
struct turtleInfo{
    float x = 0;
    float y = 0;
    float theta =0;
    float linVel =0;
    float angVel =0;

    //target;
    float targetX = 2;
    float targetY = 2;
    float error = 0.15;
    float speed = 0.5;
    float angleCost = 0.2;

    //where we publish commands for the turtle
    ros::Publisher turtleCmdPub;
    ros::Subscriber turtlePoseSub;
};

//prototypes
void turtlePoseCB(const turtlesim::PoseConstPtr &msg, const std::string &topic);
void updateSpeed(turtleInfo &turtle, turtleInfo player);



std::vector<turtleInfo> turtleList;

int specialRobot =0;


//global variables (in the future we will not use these!)



//main function
int main(int argc, char** argv) {
    //number of turtles for managing for loops.
    int turtleNum = 10;

    //setup ros
    ros::init(argc, argv, "go_loc");
    ros::NodeHandle node;

    //make more turtle
    for(int i =0; i<turtleNum; i++){
        ros::ServiceClient client= node.serviceClient<turtlesim::Spawn>("spawn");
        turtlesim::Spawn spawnVar;
        client.call(spawnVar);
    }






    //make list of robot info
    turtleList = std::vector<turtleInfo>(turtleNum);

    //subscribe to the location of the turtle.
    for(int i= 1; i<= turtleNum; i++){
        std::stringstream ss;
        std::stringstream ss2;
        std::stringstream ss3;
        ss << "turtle" << i << "/pose";
        ss2 << i;
        ss3 << "turtle" << i << "/cmd_vel";
        turtleList[i-1].turtlePoseSub = node.subscribe<turtlesim::Pose>(ss.str(),1,boost::bind(turtlePoseCB, _1, ss2.str()));
        turtleList[i-1].turtleCmdPub = node.advertise<geometry_msgs::Twist>(ss3.str(),1);
        ss.clear();
        ss2.clear();
        ss3.clear();

        int randInt = rand() % 10;
        float randFloat = (rand()% 10)/10;
        float randFloat2 = 100* (rand()/RAND_MAX);

        turtleList[i-1].speed =3* ((float)rand()/RAND_MAX);
        turtleList[i-1].angleCost =3* ((float)rand()/RAND_MAX);





    }
//    ros::Subscriber turtlePoseSub = node.subscribe<turtlesim::Pose>("turtle1/pose",1,boost::bind(turtlePoseCB, _1, "1"));
//    ros::Subscriber turtlePoseSub2 = node.subscribe<turtlesim::Pose>("turtle2/pose",1,boost::bind(turtlePoseCB, _1, "2"));
//    turtleList[0].turtleCmdPub = node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1);
//    turtleList[1].turtleCmdPub = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel",1);



//    turtleInfo turtle0 , turtle1, turtle2;
//    std::vector<turtleInfo> listOfTurtles;
//    for(int i = 0;i < 1000 ; i++){
//        turtleInfo turtle;
//        listOfTurtles.push_back(turtle);
//    }
//    for(int i = 0;i < 1000 ; i++){
//        turtleInfo turtle;
//        listOfTurtles.push_back(turtle);
//    }
//    listOfTurtles.push_back(turtle0);
//     listOfTurtles.push_back(turtle1);
//      listOfTurtles.push_back(turtle2);
//   listOfTurtles[3].x =2;



    //loop forever.
    ros::Rate rate(15);
    while(ros::ok()){
        //check for any messages.
        ros::spinOnce();
        for(int i = 1; i < turtleNum; i++){
            updateSpeed(turtleList[i], turtleList[0]);
        }


        //make sure we don't whiz around too quickly.
        rate.sleep();
    }



}

////function implementations
///
void updateSpeed(turtleInfo &turtle, turtleInfo player){
    //funny movement behavior


    //find angle towards target. don't worry about the messy maths.
    tf::Vector3 toTarget, reference;
    toTarget.setX(player.x-turtle.x);
    toTarget.setY(player.y-turtle.y);
//    toTarget.setX(turtle.targetX-turtle.x);
//    toTarget.setY(turtle.targetY-turtle.y);
    reference.setX(1);
    reference.setY(0);

    float angle = toTarget.angle(reference);
    if(toTarget.getY()<0) angle = -angle;

    angle -= turtle.theta;
    if(angle > 3.14) angle -= 2* 3.14;
    if(angle < -3.14) angle += 2*3.14;


    //the command we are building
    geometry_msgs::Twist twist;

    //which way should we turn.
    if(angle > 0){
        twist.angular.z = 1;
    }
    else{
        twist.angular.z = -1;
    }

    //angle term
    float angleCorrection = 1;
    if(angle!=0){
        angleCorrection = turtle.angleCost/(fabs(angle)*fabs(angle));
    }
    if(angleCorrection > 1.0){
        angleCorrection = 1.0;
    }

    //how fast should we go.
    float distance = toTarget.length();
    if(distance < turtle.error){
//        turtle.targetX =rand()%10;
//        turtle.targetY =rand()%10;
    }
    twist.linear.x = turtle.speed*distance*angleCorrection;
    turtle.turtleCmdPub.publish(twist);


}

void turtlePoseCB(const turtlesim::PoseConstPtr&msg,const std::string &topic){
    int turtleNum = atoi(topic.c_str());
    turtleList[turtleNum-1].x = msg->x;
    turtleList[turtleNum-1].y = msg->y;
    turtleList[turtleNum-1].theta = msg->theta;
    if(turtleList[turtleNum-1].theta > 3.14) turtleList[turtleNum-1].theta -= 2* 3.14;
    if(turtleList[turtleNum-1].theta < -3.14) turtleList[turtleNum-1].theta += 2*3.14;


    turtleList[turtleNum-1].linVel = msg->linear_velocity;
    turtleList[turtleNum-1].angVel = msg->angular_velocity;
}
