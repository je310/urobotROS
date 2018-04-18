#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>

bool AreQuaternionsClose(tf::Quaternion q1, tf::Quaternion q2);
tf::Quaternion InverseSignQuaternion(tf::Quaternion q);

int main(int argc, char** argv) {
    ros::init(argc, argv, "calibrateMarkers");
    ros::NodeHandle node;

    tf::TransformListener tl;
    tf::TransformBroadcaster tb;

    ros::Publisher marker_pub;
    marker_pub = node.advertise<visualization_msgs::Marker>("corners", 1);

    if(argc !=2){
        ROS_INFO("argument should be the master marker number");
        return -1;
    }
    int master = atoi(argv[1]);
    const int markersToScan= 10;

    int dataCounter[markersToScan];
    int dataToCollect = 30;
    std::vector<std::vector<tf::StampedTransform>> transList;
    for(int i =0; i < markersToScan; i++){
        std::vector<tf::StampedTransform> sublist;
        transList.push_back(sublist);
        dataCounter[i] = 0;
    }

    //generate master name;
    std::stringstream ssMaster;
    ssMaster << "/ar_marker_" << master;

    //collect transfrorms.
    while(ros::ok()){

        for(int i =0; i < markersToScan; i++){
            if(i != master){
                //generate names
                std::stringstream ssTarget;
                ssTarget << "/ar_marker_" << i;
                tf::StampedTransform transformToRef,transformToOther;
                try{


                    tl.lookupTransform(ssTarget.str(), ssMaster.str(),
                                       ros::Time(0), transformToRef);
                    //            tf::StampedTransform transform;
                    //            transform.mult( transformToRef,transformToOther.inverse());
                    transList[i].push_back(transformToRef);
                    dataCounter[i] ++;
                    ros::Duration(0.05).sleep();

                }
                catch(tf::TransformException ex){
                    // ROS_ERROR("%s",ex.what());
                }
                //if this one has enough data publish the average
                if(dataCounter[i]>dataToCollect){
                    //find average origin, simply the average of the vector.
                    tf::Vector3 avOrigin(0,0,0);
                    for (int j = 0; j < transList[i].size(); j++){
                        tf::Vector3 av;
                        avOrigin = avOrigin + transList[i][j].getOrigin();
                    }
                    avOrigin /= transList[i].size();

                    //find average of the rotation quaternion, as per the unity example.
                    float w = 0.0f;
                    float x = 0.0f;
                    float y = 0.0f;
                    float z = 0.0f;
                    for(int j = 0; j < transList[i].size(); j++){

                        //check the next quaternion is near.
                        if(!AreQuaternionsClose(transList[i][0].getRotation(),transList[i][j].getRotation())){
                            transList[i][j].setRotation(InverseSignQuaternion(transList[i][j].getRotation()));
                        }

                        //Average the values
                        x += transList[i][j].getRotation().getX();
                        y += transList[i][j].getRotation().getY();
                        z += transList[i][j].getRotation().getZ();
                        w += transList[i][j].getRotation().getW();
                    }

                    x = x/transList[i].size();
                    y = y/transList[i].size();
                    z = z/transList[i].size();
                    w = w/transList[i].size();

                    tf::Quaternion averageQ(x,y,z,w);
                    averageQ.normalize();

                    std::cout << ssTarget.str() << " " << avOrigin.getX() << " " << avOrigin.getY() << " " << avOrigin.getZ() << " " << averageQ.getX() << " " << averageQ.getY() << " " << averageQ.getZ() << " " << averageQ.getW()<< std::endl;
                    tf::StampedTransform averageTransform;
                    averageTransform.setOrigin(avOrigin);
                    averageTransform.setRotation(averageQ);
                    std::stringstream ssOut;
                    ssOut << "ar_out_" << i;
                    averageTransform.child_frame_id_= ssOut.str();
                    averageTransform.frame_id_ = ssTarget.str();
                    averageTransform.stamp_ = transformToRef.stamp_;

                    tb.sendTransform(averageTransform);
                }
            }
        }

    }




}
tf::Quaternion InverseSignQuaternion(tf::Quaternion q){
    tf::Quaternion res(-q.getX(), -q.getY(), -q.getZ(), -q.getW());
    return res;
}


bool AreQuaternionsClose(tf::Quaternion q1, tf::Quaternion q2){

    float dot = q1.dot(q2);

    if(dot < 0.0f){

        return false;
    }

    else{

        return true;
    }
}
