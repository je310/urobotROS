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

    int dataCounter = 0;
    int dataToCollect = 20;
    std::vector<tf::StampedTransform> transList;
    //collect transfrorms.
    while(dataCounter < dataToCollect){
        try{
            tf::StampedTransform transformToRef,transformToOther;

            tl.lookupTransform("/ar_marker_3", "/Left",
                               ros::Time(0), transformToRef);
            tl.lookupTransform("/ar_marker_5", "/Left",
                               ros::Time(0), transformToOther);
            tf::StampedTransform transform;
            transform.mult( transformToRef,transformToOther.inverse());
            transList.push_back(transform);
            dataCounter ++;
            ros::Duration(0.2).sleep();

        }
        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }
    }

    //find average origin, simply the average of the vector.
    tf::Vector3 avOrigin;
    for (int i = 0; i < transList.size(); i++){
        tf::Vector3 av;
        avOrigin = avOrigin + transList[i].getOrigin();
    }
    avOrigin /= transList.size();


    //find average of the rotation quaternion, as per the unity example.
    float w = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    for(int i = 0; i < transList.size(); i++){

        //check the next quaternion is near.
        if(!AreQuaternionsClose(transList[0].getRotation(),transList[i].getRotation())){
            transList[i].setRotation(InverseSignQuaternion(transList[i].getRotation()));
        }

        //Average the values
        x += transList[i].getRotation().getX();
        y += transList[i].getRotation().getY();
        z += transList[i].getRotation().getZ();
        w += transList[i].getRotation().getW();
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
    averageTransform.stamp_ = ros::Time::now();
    averageTransform.frame_id_ = "ar_marker_3";
    averageTransform.child_frame_id_ = "interMarker";


    //visualise result

    visualization_msgs::Marker sphere_list;
    sphere_list.header.frame_id= "/ar_marker_3";
    sphere_list.header.stamp= ros::Time::now();
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

    //each corner
    float edge = 0.02471;
    std::vector<tf::Vector3> corners;
    corners.push_back(tf::Vector3(-0.5*edge,-0.5*edge,0));
    corners.push_back(tf::Vector3(0.5*edge,-0.5*edge,0));
    corners.push_back(tf::Vector3(0.5*edge,0.5*edge,0));
    corners.push_back(tf::Vector3(-0.5*edge,0.5*edge,0));

    for(int i = 0; i < 4; i++ ){
        geometry_msgs::Point p;
        p.x =  corners[i].getX();
        p.y =  corners[i].getY();
        p.z =  corners[i].getZ();
        sphere_list.points.push_back(p);

    }
    for(int i = 0; i < 4; i++ ){
        geometry_msgs::Point p;
        tf::Vector3 transP = averageTransform(corners[i]);
        p.x =  transP.getX();
        p.y =  transP.getY();
        p.z =  transP.getZ();
        sphere_list.points.push_back(p);
    }
    //    <multimarker markers="9">
    //    <marker index="13" status="1">
    //       <corner x="-2.6" y = "-2.6" z = "0" />
    //       <corner x="2.6" y = "-2.6" z = "0" />
    //       <corner x="2.6" y = "2.6" z = "0" />
    //       <corner x="-2.6" y = "2.6" z = "0" />
    //    </marker>
    std::vector<std::string> nameMarkers;
    nameMarkers.push_back("3");
    nameMarkers.push_back("5");
    std::ofstream myfile;
    std::stringstream ss;
    ss << ros::package::getPath("locationCamera") << "/bundle/calibrated.xml";
    myfile.open (ss.str().c_str());
    myfile << "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>" << std::endl;
    myfile << "<multimarker markers=\"2\">" <<std::endl;
    for(int i = 0; i < 2; i++){
        myfile << "<marker index=\"" << nameMarkers[i] << "\" status=\"2\">"<< std::endl;
        for(int j = 0; j < 4; j++){
            myfile << "<corner x=\" "<<100*sphere_list.points[4*i +j].x<<" \" y = \" "<< 100*sphere_list.points[4*i +j].y<<"\" z = \" "<< 100*sphere_list.points[4*i +j].z<< "\" />" << std::endl;
        }
        myfile << "</marker>" << std::endl;
    }
    myfile << "</multimarker>" << std::endl;
    myfile.close();
    while(ros::ok()){
        averageTransform.stamp_ = ros::Time::now();
        sphere_list.header.stamp = ros::Time::now();
        marker_pub.publish(sphere_list);
        tb.sendTransform(averageTransform);
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
