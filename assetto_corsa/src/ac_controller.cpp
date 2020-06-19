/*
    ac_controller.cpp

    Jinsun Park
    (zzangjinsun@3secondz.com)
*/

#include <iostream>
#include <string>

#include <ros/ros.h>

#include <assetto_corsa/ACRaw.h>
#include <std_msgs/Float64MultiArray.h>

using namespace std;

// Global variables
ros::Publisher pub;


void callback(assetto_corsa::ACRawConstPtr msg){

    // CONTROLLER CODE HERE

    // ex) Input data : msg->[ITEM NAME].data



    // CONTROLLER CODE HERE

    ROS_INFO_STREAM("speed_Kmh : " << msg->speed_Kmh.data);
    ROS_INFO_STREAM("steer : " << msg->steer.data);
    ROS_INFO_STREAM("brake : " << msg->brake.data);

    std_msgs::Float64MultiArray output;

    output.layout.dim.push_back(std_msgs::MultiArrayDimension());
    output.layout.dim[0].label = "dim";
    output.layout.dim[0].size = 3;
    output.layout.dim[0].stride = 1;

    output.data.push_back(msg->speed_Kmh.data);
    output.data.push_back(msg->steer.data);
    output.data.push_back(msg->brake.data);

    pub.publish(output);

}


int main(int argc, char** argv){
    ros::init(argc, argv, "ac_controller");
    ros::NodeHandle nh("~");

    int freq, queue_size;
    string topic_input, topic_output;

    nh.param("freq", freq, 20);
    nh.param("queue_size", queue_size, 1);
    nh.param("topic_input", topic_input, string("/ac_pub/ACRaw"));
    nh.param("topic_output", topic_output, string("output"));

    ROS_INFO_STREAM("freq : " << freq);
    ROS_INFO_STREAM("queue_size : " << queue_size);
    ROS_INFO_STREAM("topic_input : " << topic_input);
    ROS_INFO_STREAM("topic_output : " << topic_output);

    ros::Subscriber sub = nh.subscribe(topic_input, 1, callback);
    pub = nh.advertise<std_msgs::Float64MultiArray>(topic_output, 1);

    ros::Rate rate(freq);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}