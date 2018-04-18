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

    const int markersToScan= 10;


    ros::Rate rate(1000);

    std::string camstr = "camera";
    //collect transfrorms.
    while(ros::ok()){
        std::vector<tf::StampedTransform> transList;
        ros::Time stamp;
        for(int i =0; i < markersToScan; i++){

            //generate names
            std::stringstream ssTarget;
            ssTarget << "/ar_marker_aug_" << i;
            tf::StampedTransform transformToRef;
            try{


                tl.lookupTransform(camstr,ssTarget.str(),
                                   ros::Time(0), transformToRef);
                //            tf::StampedTransform transform;
                //            transform.mult( transformToRef,transformToOther.inverse());
                transList.push_back(transformToRef);
                stamp = transformToRef.stamp_;

            }
            catch(tf::TransformException ex){
                // ROS_ERROR("%s",ex.what());
            }
        }
        //if this one has enough data publish the average

        if(transList.size() > 0){
            //check for newest time.
            ros::Time newestTime = transList[0].stamp_;
            for(int i = 0; i < transList.size(); i++){
                if(transList[i].stamp_ > newestTime){
                    newestTime = transList[i].stamp_;
                }
            }
            std::vector<tf::StampedTransform> transListTemp;
            for(int i = 0; i < transList.size(); i++){
                if(transList[i].stamp_ == newestTime){
                    transListTemp.push_back(transList[i]);
                }
            }
            transList = transListTemp;
            //find average origin, simply the average of the vector.
            tf::Vector3 avOrigin(0,0,0);
            for (int j = 0; j < transList.size(); j++){
                avOrigin = avOrigin + transList[j].getOrigin();
            }
            avOrigin /= transList.size();

            //find average of the rotation quaternion, as per the unity example.
            float w = 0.0f;
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            for(int j = 0; j < transList.size(); j++){

                //check the next quaternion is near.
                if(!AreQuaternionsClose(transList[0].getRotation(),transList[j].getRotation())){
                    transList[j].setRotation(InverseSignQuaternion(transList[j].getRotation()));
                }

                //Average the values
                x += transList[j].getRotation().getX();
                y += transList[j].getRotation().getY();
                z += transList[j].getRotation().getZ();
                w += transList[j].getRotation().getW();
            }

            x = x/transList.size();
            y = y/transList.size();
            z = z/transList.size();
            w = w/transList.size();

            tf::Quaternion averageQ(x,y,z,w);
            averageQ.normalize();


            tf::StampedTransform averageTransform;
            averageTransform.setOrigin(avOrigin);
            averageTransform.setRotation(averageQ);
            std::stringstream ssOut;
            ssOut << "markerAgr";
            averageTransform.child_frame_id_= ssOut.str();
            averageTransform.frame_id_ = camstr;
            averageTransform.stamp_ =newestTime;

            tb.sendTransform(averageTransform);
        }


        rate.sleep();
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
