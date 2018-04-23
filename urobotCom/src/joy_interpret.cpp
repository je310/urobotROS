#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include "math.h"
#include "geometry_msgs/Twist.h"

#include <sstream>

ros::Publisher rob0pub,rob1pub,rob2pub,rob3pub,fightpub,linePub,special1Pub;
float rob0Lin, rob0Ang,rob1Lin, rob1Ang,rob2Lin, rob2Ang,rob3Lin, rob3Ang;
void joyCB(sensor_msgs::JoyConstPtr msg){
    // robot0 left d pad.

    if(msg->buttons[5]==1){
        std_msgs::String fightMsg;
        fightMsg.data = "fight";
        fightpub.publish(fightMsg);
    }

    if(msg->buttons[4]==1){
        std_msgs::String lineMsg;
        lineMsg.data = "follow";
        linePub.publish(lineMsg);
    }
    if(msg->buttons[6]==1){
        std_msgs::String special1Msg;
        special1Msg.data = "special1";
        special1Pub.publish(special1Msg);
    }

    rob2Lin = msg->axes[6];
    rob2Ang =msg ->axes[5];

    // robot1 left analogue
    rob0Lin = msg->axes[1];
    rob0Ang = msg->axes[0];

    // robot2 right analogue
    rob1Lin = msg->axes[4];
    rob1Ang = msg->axes[3];

    // robot1 right buttons
    rob3Lin = msg->buttons[0] - msg->buttons[2];
    rob3Ang =  msg->buttons[3] - msg->buttons[1];



}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "interpret");

  ros::NodeHandle n;

  ros::Subscriber chatter_pub = n.subscribe("/joy", 1, joyCB);
  rob0pub = n.advertise<geometry_msgs::Twist>("rob0",1);
  rob1pub = n.advertise<geometry_msgs::Twist>("rob1",1);
  rob2pub = n.advertise<geometry_msgs::Twist>("rob2",1);
  rob3pub = n.advertise<geometry_msgs::Twist>("rob3",1);
  fightpub = n.advertise<std_msgs::String>("fight",1);
  linePub = n.advertise<std_msgs::String>("follow",1);
  special1Pub = n.advertise<std_msgs::String>("special1",1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
      ros::spinOnce();
      geometry_msgs::Twist twistOut;
      twistOut.angular.z = rob0Ang * 2 * 3.14;
      twistOut.linear.x = rob0Lin * 0.1;
      rob0pub.publish(twistOut);

      geometry_msgs::Twist twistOut1;
      twistOut1.angular.z = rob1Ang* 2 * 3.14;;
      twistOut1.linear.x  = rob1Lin* 0.1;
      rob1pub.publish(twistOut1);

      geometry_msgs::Twist twistOut2;
      twistOut2.angular.z = rob2Ang;
      twistOut2.linear.x  = rob2Lin;
      rob2pub.publish(twistOut2);

      geometry_msgs::Twist twistOut3;
      twistOut3.angular.z = rob3Ang;
      twistOut3.linear.x  = rob3Lin;
      rob3pub.publish(twistOut3);
      loop_rate.sleep();

  }


  return 0;
}
